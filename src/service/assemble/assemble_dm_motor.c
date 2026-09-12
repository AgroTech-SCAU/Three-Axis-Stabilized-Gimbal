#include "assemble.h"

#include "bus_motor/bus_motor.h"
#include "bus_motor/dm_motor.h"
#include "delay.h"
#include "log.h"
#include "main.h"
#include "stm32_hal_can.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * DMG6220 Yaw 电机。
 *
 * 驱动层、V4 协议、registry/profile 和模式 ACK 处理直接来自成功示例 dm_ok。
 * 本文件只把成功示例的 motor_test 生命周期适配成“自动进入 POS_VEL + IMU Yaw”。
 */
#define DM_GIMBAL_LOGICAL_ID             1u
#define DM_GIMBAL_CAN_ID                 0x01u
#define DM_GIMBAL_MASTER_ID              0x00u
#define DM_CAN_ENABLE_DELAY_MS            10u
#define DM_INIT_CLEAR_DELAY_MS            20u
#define DM_AUTO_MODE_DELAY_MS             100u
#define DM_MODE_RETRY_DELAY_MS            500u
#define DM_MODE_MAX_RETRY                 3u
#define DM_TARGET_TX_PERIOD_MS            10u
#define DM_FEEDBACK_STALE_TIMEOUT_MS      500u
#define DM_FEEDBACK_LOG_PERIOD_MS         1000u
#define DM_STALE_LOG_PERIOD_MS            1000u
#define DM_DEFAULT_TARGET_SPEED_RAD_S     1.0f

typedef enum {
    DM_RUNTIME_UNINITIALIZED = 0u,
    DM_RUNTIME_WAIT_AUTO_MODE,
    DM_RUNTIME_WAIT_FEEDBACK,
    DM_RUNTIME_READY,
    DM_RUNTIME_FAILED,
} DmRuntimeState;

static bool dm_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static void dm_motor_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                     const FDCAN_RxHeaderTypeDef* header,
                                     const uint8_t data[64],
                                     void* user);
static bool dm_feedback_is_fresh(uint32_t now_ms);
static bool dm_send_current_target(void);
static bool dm_enter_pos_vel_reference(void);
static void dm_try_enter_ready(uint32_t now_ms, const BusMotorFeedback* feedback);
static void dm_log_rx_diagnostic(void);

static const DmMotorConfig s_dm_config = {
    .can_id = DM_GIMBAL_CAN_ID,
    .master_id = DM_GIMBAL_MASTER_ID,
    .model = DM_MOTOR_MODEL_DMG6220,
    .firmware = {
        .major = DM_MOTOR_FIRMWARE_V4,
    },
    .default_mode = DM_MOTOR_MODE_POS_VEL,
};

/* 与成功示例一样，由 assemble 提供实例存储。 */
static DmMotorInstance s_dm_instances[1];

static const BusMotorPortOps s_dm_motor_ops = {
    .send = dm_motor_can_send,
    .read = NULL,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
    .flush_rx = NULL,
};

static volatile bool s_feedback_updated = false;
static volatile bool s_fault_pending = false;
static volatile uint8_t s_fault_code = 0u;
static BusMotorFeedback s_feedback;
static bool s_feedback_seen = false;
static bool s_enabled = false;
static bool s_fault_latched = false;
static bool s_hold_target_ready = false;
static float s_target_position = 0.0f;
static float s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
static uint32_t s_last_feedback_seen_ms = 0u;
static uint32_t s_last_target_tx_ms = 0u;
static uint32_t s_last_feedback_log_ms = 0u;
static uint32_t s_last_stale_log_ms = 0u;
static uint32_t s_auto_mode_start_ms = 0u;
static uint32_t s_auto_mode_wait_ms = DM_AUTO_MODE_DELAY_MS;
static uint8_t s_mode_attempt = 0u;
static DmRuntimeState s_runtime_state = DM_RUNTIME_UNINITIALIZED;

/* 只用于失败诊断；ISR 只复制，不打印。 */
static volatile uint32_t s_raw_rx_count = 0u;
static volatile uint32_t s_last_raw_rx_id = 0u;
static volatile uint8_t s_last_raw_rx_data[8];

SystemStatus assemble_dm_motor(void) {
    BusMotorStatus status;
    BspCanStatus can_status;
    uint32_t now_ms;

    /* 以下三步与成功示例 assemble_motor() 完全相同。 */
    status = bus_motor.init();
    if(status != MOTOR_STATUS_OK) {
        log_error("DM bus_motor init failed: %s", bus_motor.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }

    status = dm_motor_init(&s_dm_motor_ops, s_dm_instances,
                           (uint16_t)(sizeof(s_dm_instances) / sizeof(s_dm_instances[0])));
    if(status != MOTOR_STATUS_OK) {
        log_error("DM driver init failed: %s", bus_motor.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }

    status = dm_motor_bind((BusMotorId)DM_GIMBAL_LOGICAL_ID, &s_dm_config);
    if(status != MOTOR_STATUS_OK) {
        log_error("DM bind failed: %s", bus_motor.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }

    now_ms = HAL_GetTick();
    s_feedback_updated = false;
    s_fault_pending = false;
    s_fault_code = 0u;
    s_feedback = (BusMotorFeedback){ 0 };
    s_feedback_seen = false;
    s_enabled = false;
    s_fault_latched = false;
    s_hold_target_ready = false;
    s_target_position = 0.0f;
    s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
    s_last_feedback_seen_ms = now_ms;
    s_last_target_tx_ms = now_ms;
    s_last_feedback_log_ms = now_ms;
    s_last_stale_log_ms = now_ms;
    s_auto_mode_start_ms = now_ms;
    s_auto_mode_wait_ms = DM_AUTO_MODE_DELAY_MS;
    s_mode_attempt = 0u;
    s_runtime_state = DM_RUNTIME_UNINITIALIZED;
    s_raw_rx_count = 0u;
    s_last_raw_rx_id = 0u;
    memset((void*)s_last_raw_rx_data, 0, sizeof(s_last_raw_rx_data));

    /* 成功示例：CAN1 收发器先上电并稳定 10 ms，再启动 FDCAN。 */
    HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    delay_ms(DM_CAN_ENABLE_DELAY_MS);

    if(can_register_rx_callback(&hfdcan1, dm_motor_can_rx_callback, NULL) != STM32_HAL_CAN_OK) {
        log_error("DM rx callback register failed");
        return SYSTEM_STATUS_ERROR;
    }
    can_status = can_filter_init();
    if(can_status != STM32_HAL_CAN_OK) {
        log_error("DM CAN filter failed: %s", can_error_code_to_str(can_status));
        return SYSTEM_STATUS_ERROR;
    }
    can_status = can_start(&hfdcan1);
    if(can_status != STM32_HAL_CAN_OK) {
        log_error("DM CAN start failed: %s", can_error_code_to_str(can_status));
        return SYSTEM_STATUS_ERROR;
    }

    /* 成功示例 motor_test_init(): disable -> 20 ms -> clear_error。 */
    status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    if(status != MOTOR_STATUS_OK) {
        log_error("DM init disable failed: %s", bus_motor.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("DM init disable ok");

    delay_ms(DM_INIT_CLEAR_DELAY_MS);
    status = dm_motor_clear_error((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    if(status != MOTOR_STATUS_OK) {
        log_error("DM init clear error failed: %s", bus_motor.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("DM init clear error ok");

    /* 与示例相同：清错后原子丢弃初始化阶段的旧反馈/故障事件。 */
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_feedback_updated = false;
        s_fault_pending = false;
        s_fault_code = 0u;
        s_feedback = (BusMotorFeedback){ 0 };
        if(primask == 0u) {
            __enable_irq();
        }
    }

    s_feedback_seen = false;
    s_enabled = false;
    s_fault_latched = false;
    s_hold_target_ready = false;
    s_runtime_state = DM_RUNTIME_WAIT_AUTO_MODE;
    s_auto_mode_start_ms = HAL_GetTick();
    s_auto_mode_wait_ms = DM_AUTO_MODE_DELAY_MS;

    log_info("DM init ready: dm_ok driver/profile stack active; POS_VEL deferred to main loop");
    return SYSTEM_STATUS_OK;
}

void assemble_dm_motor_process(void) {
    uint32_t now = HAL_GetTick();
    BusMotorFeedback feedback = { 0 };
    bool updated = false;
    bool fault_pending = false;
    uint8_t fault_code = 0u;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if(s_feedback_updated) {
        feedback = s_feedback;
        s_feedback_updated = false;
        updated = true;
    }
    if(s_fault_pending) {
        fault_pending = true;
        fault_code = s_fault_code;
        s_fault_pending = false;
    }
    if(primask == 0u) {
        __enable_irq();
    }

    /* 与成功示例 motor_test_process() 相同：故障只在主循环执行安全失能。 */
    if(fault_pending) {
        BusMotorStatus disable_status;

        s_fault_latched = true;
        s_enabled = false;
        s_hold_target_ready = false;
        s_runtime_state = DM_RUNTIME_FAILED;
        disable_status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        log_error("DM fault latched: code=0x%02X disable=%s",
                  fault_code, bus_motor.status_str(disable_status));
        return;
    }

    if(s_runtime_state == DM_RUNTIME_WAIT_AUTO_MODE &&
       (uint32_t)(now - s_auto_mode_start_ms) >= s_auto_mode_wait_ms) {
        s_mode_attempt++;
        log_info("DM runtime POS_VEL entry attempt=%u/%u", s_mode_attempt, DM_MODE_MAX_RETRY);

        if(dm_enter_pos_vel_reference()) {
            s_enabled = true;
            s_runtime_state = DM_RUNTIME_WAIT_FEEDBACK;
            s_last_feedback_seen_ms = now;
            s_last_target_tx_ms = now;
            log_info("DM POS_VEL enabled; waiting feedback to lock current position");
        }
        else if(s_mode_attempt < DM_MODE_MAX_RETRY) {
            s_auto_mode_start_ms = now;
            s_auto_mode_wait_ms = DM_MODE_RETRY_DELAY_MS;
            log_warn("DM POS_VEL entry failed; retry in %u ms", DM_MODE_RETRY_DELAY_MS);
        }
        else {
            s_enabled = false;
            s_hold_target_ready = false;
            s_runtime_state = DM_RUNTIME_FAILED;
            log_error("DM POS_VEL entry failed after %u attempts", DM_MODE_MAX_RETRY);
            dm_log_rx_diagnostic();
        }
    }

    if(updated) {
        s_feedback_seen = true;
        s_last_feedback_seen_ms = now;

        if(s_runtime_state == DM_RUNTIME_WAIT_FEEDBACK && s_enabled && !s_fault_latched) {
            dm_try_enter_ready(now, &feedback);
        }

        if((uint32_t)(now - s_last_feedback_log_ms) >= DM_FEEDBACK_LOG_PERIOD_MS) {
            s_last_feedback_log_ms = now;
            log_info("DM fb pos=%.3f spd=%.3f tor=%.3f err=0x%02X ready=%u",
                     (double)feedback.position,
                     (double)feedback.velocity,
                     (double)feedback.torque,
                     feedback.error_code,
                     assemble_dm_motor_is_ready() ? 1u : 0u);
        }
    }

    if(s_runtime_state == DM_RUNTIME_READY && s_enabled && !s_fault_latched &&
       s_hold_target_ready &&
       (uint32_t)(now - s_last_target_tx_ms) >= DM_TARGET_TX_PERIOD_MS) {
        s_last_target_tx_ms = now;
        if(!dm_send_current_target()) {
            log_warn("DM target resend failed");
        }
    }

    if(s_feedback_seen && !dm_feedback_is_fresh(now) &&
       (uint32_t)(now - s_last_stale_log_ms) >= DM_STALE_LOG_PERIOD_MS) {
        s_last_stale_log_ms = now;
        log_warn("DM feedback stale age=%lu ms; reject new yaw target",
                 (unsigned long)(now - s_last_feedback_seen_ms));
    }
}

bool assemble_dm_motor_has_feedback(void) {
    return s_feedback_seen;
}

bool assemble_dm_motor_is_ready(void) {
    return s_runtime_state == DM_RUNTIME_READY &&
           s_enabled &&
           !s_fault_latched &&
           s_hold_target_ready &&
           dm_feedback_is_fresh(HAL_GetTick());
}

bool assemble_dm_motor_get_position(float* position) {
    uint32_t primask;

    if(position == NULL || !s_feedback_seen) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *position = s_feedback.position;
    if(primask == 0u) {
        __enable_irq();
    }
    return true;
}

bool assemble_dm_motor_set_target_position(float position, float speed) {
    if(!assemble_dm_motor_is_ready() || !isfinite(position) || !isfinite(speed) || speed < 0.0f) {
        return false;
    }

    /*
     * Only update the desired target here. assemble_dm_motor_process() is the
     * single 100 Hz CAN producer for runtime POS_VEL commands. This prevents
     * the gimbal controller and the resend path from generating two DM frames
     * in the same 10 ms window.
     */
    s_target_position = position;
    s_target_speed = speed;
    return true;
}

static bool dm_feedback_is_fresh(uint32_t now_ms) {
    return s_feedback_seen &&
           (uint32_t)(now_ms - s_last_feedback_seen_ms) <= DM_FEEDBACK_STALE_TIMEOUT_MS;
}

static bool dm_send_current_target(void) {
    return dm_motor_set_pos_vel((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                s_target_position,
                                s_target_speed) == MOTOR_STATUS_OK;
}

static bool dm_enter_pos_vel_reference(void) {
    BusMotorStatus status;
    BusMotorStatus cleanup_status;

    /* 成功示例 motor_test_set_mode(POS_VEL) 的原始顺序。 */
    status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    if(status == MOTOR_STATUS_OK) {
        status = bus_motor.profile.activate((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                            BUS_MOTOR_PROFILE_POSITION);
    }
    if(status == MOTOR_STATUS_OK) {
        status = bus_motor.basic.enable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    }

    if(status != MOTOR_STATUS_OK) {
        cleanup_status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        log_error("DM mode switch failed: status=%s cleanup_disable=%s",
                  bus_motor.status_str(status),
                  bus_motor.status_str(cleanup_status));
        dm_log_rx_diagnostic();
        return false;
    }

    return true;
}

static void dm_try_enter_ready(uint32_t now_ms, const BusMotorFeedback* feedback) {
    BusMotorStatus status;

    if(feedback == NULL || s_runtime_state != DM_RUNTIME_WAIT_FEEDBACK ||
       !s_enabled || s_fault_latched) {
        return;
    }

    /* 第一帧使能后反馈作为安全保持点，再交给 IMU Yaw 改目标。 */
    s_target_position = feedback->position;
    s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
    status = dm_motor_set_pos_vel((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                  s_target_position,
                                  s_target_speed);
    if(status != MOTOR_STATUS_OK) {
        s_hold_target_ready = false;
        s_enabled = false;
        s_runtime_state = DM_RUNTIME_FAILED;
        (void)bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        log_error("DM initial hold failed: %s", bus_motor.status_str(status));
        return;
    }

    s_hold_target_ready = true;
    s_last_target_tx_ms = now_ms;
    s_runtime_state = DM_RUNTIME_READY;
    log_info("DM ready: hold current pos=%.3f rad", (double)feedback->position);
}

static bool dm_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    /* dm_ok 的 can_send() 始终发送 Classical CAN + BRS OFF。 */
    return can_send_classic(&hfdcan1, id, data, len) == STM32_HAL_CAN_OK;
}

static void dm_motor_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                     const FDCAN_RxHeaderTypeDef* header,
                                     const uint8_t data[64],
                                     void* user) {
    BusMotorFeedback feedback;
    uint8_t i;

    (void)hcan;
    (void)user;

    if(header == NULL || data == NULL) {
        return;
    }

    s_raw_rx_count++;
    s_last_raw_rx_id = header->Identifier;
    for(i = 0u; i < 8u; ++i) {
        s_last_raw_rx_data[i] = data[i];
    }

    /* 成功示例 assemble_motor_can_rx() 原始顺序：参数 ACK 必须先解析。 */
    if(dm_motor_parse_parameter_frame(header->Identifier, data)) {
        return;
    }

    if(dm_motor_parse_feedback_frame(header->Identifier, data, &feedback) == MOTOR_STATUS_OK &&
       feedback.id == (BusMotorId)DM_GIMBAL_LOGICAL_ID) {
        s_feedback = feedback;
        s_feedback_updated = true;

        if(feedback.error_code >= 0x08u && feedback.error_code <= 0x0Eu) {
            s_fault_code = feedback.error_code;
            s_fault_pending = true;
        }
    }
}

static void dm_log_rx_diagnostic(void) {
    uint32_t primask;
    uint32_t count;
    uint32_t id;
    uint8_t data[8];
    uint8_t i;

    primask = __get_PRIMASK();
    __disable_irq();
    count = s_raw_rx_count;
    id = s_last_raw_rx_id;
    for(i = 0u; i < 8u; ++i) {
        data[i] = s_last_raw_rx_data[i];
    }
    if(primask == 0u) {
        __enable_irq();
    }

    log_error("DM RX diag count=%lu last_id=0x%03lX data=%02X %02X %02X %02X %02X %02X %02X %02X ack_seq=%lu ack_val=%lu",
              (unsigned long)count,
              (unsigned long)id,
              data[0], data[1], data[2], data[3],
              data[4], data[5], data[6], data[7],
              (unsigned long)s_dm_instances[0].mode_ack_sequence,
              (unsigned long)s_dm_instances[0].mode_ack_value);
}
