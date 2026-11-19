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
 * DMG6220 Yaw 电机
 *
 * 驱动层、V4 协议、registry/profile 和模式 ACK 处理直接来自成功示例 dm_ok
 * 本文件只把成功示例的 motor_test 生命周期适配成“自动进入 POS_VEL + IMU Yaw”
 */
#define DM_GIMBAL_LOGICAL_ID             1u
#define DM_GIMBAL_CAN_ID                 0x01u
#define DM_GIMBAL_MASTER_ID              0x00u
#define DM_CAN_ENABLE_DELAY_MS            10u
#define DM_INIT_CLEAR_DELAY_MS            20u
#define DM_AUTO_MODE_DELAY_MS             100u
#define DM_MODE_RETRY_DELAY_MS            500u
#define DM_MODE_MAX_RETRY                 3u
#define DM_MODE_RETRY_FOREVER_DELAY_MS    2000u
#define DM_CAN_DIAG_LOG_PERIOD_MS          1000u
#define DM_TARGET_TX_PERIOD_MS            3u
#define DM_RELEASE_RETRY_MS               100u
#define DM_RELEASE_QUERY_MS               20u
#define DM_RELEASE_FEEDBACK_MAX_AGE_MS     200u
#define DM_RESUME_FEEDBACK_MAX_AGE_MS      20u
#define DM_RESUME_FEEDBACK_TIMEOUT_MS      200u
#define DM_RESUME_TIMEOUT_MS               400u
#define DM_RESUME_RETRY_MS                 10u
#define DM_RESUME_MODE_BUDGET_MS           110u
#define DM_FEEDBACK_STALE_TIMEOUT_MS      500u
#define DM_FEEDBACK_LOG_PERIOD_MS         1000u
#define DM_STALE_LOG_PERIOD_MS            1000u
#define DM_DEFAULT_TARGET_SPEED_RAD_S     1.0f
volatile float g_yaw_velocity = 0.0f;
typedef enum {
    DM_RUNTIME_UNINITIALIZED = 0u,
    DM_RUNTIME_WAIT_AUTO_MODE,
    DM_RUNTIME_WAIT_RESUME_FEEDBACK,
    DM_RUNTIME_WAIT_FEEDBACK,
    DM_RUNTIME_READY,
    DM_RUNTIME_RELEASED,
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
static void dm_process_released(uint32_t now_ms);
static bool dm_request_feedback(void);
static void dm_process_resume(uint32_t now_ms);
static void dm_abort_resume(const char* reason);
static void dm_try_enter_ready(uint32_t now_ms, const BusMotorFeedback* feedback);
static void dm_log_rx_diagnostic(void);
static void dm_log_can_diagnostic(const char* action);
static void dm_restart_runtime_handshake(uint32_t now_ms, uint32_t delay_ms);

static const DmMotorConfig s_dm_config = {
    .can_id = DM_GIMBAL_CAN_ID,
    .master_id = DM_GIMBAL_MASTER_ID,
    .model = DM_MOTOR_MODEL_DMG6220,
    .firmware = {
        .major = DM_MOTOR_FIRMWARE_V4,
    },
    .default_mode = DM_MOTOR_MODE_POS_VEL,
};

/* 与成功示例一样，由 assemble 提供实例存储 */
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
static bool s_external_target_active;
static uint32_t s_external_target_ms, s_external_target_timeout_ms;
static volatile uint32_t s_feedback_rx_ms;
static volatile uint32_t s_feedback_sequence;
static uint32_t s_enable_feedback_sequence;
static bool s_release_requested, s_release_command_sent, s_rearm_pending;
static bool s_resume_limits_valid;
static float s_resume_min_position, s_resume_max_position;
static uint32_t s_release_feedback_sequence;
static uint32_t s_last_release_tx_ms, s_last_release_query_ms;
static uint32_t s_resume_started_ms, s_resume_stage_ms, s_resume_last_tx_ms;
static uint32_t s_resume_feedback_sequence;
static bool s_resume_query_sent;
static const char* s_resume_wait_reason;
static float s_target_position = 0.0f;
static float s_last_sent_target;
static bool s_last_sent_target_valid;
static float s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
static uint32_t s_last_feedback_seen_ms = 0u;
static uint32_t s_last_target_tx_ms = 0u;
static uint32_t s_last_feedback_log_ms = 0u;
static uint32_t s_last_stale_log_ms = 0u;
static uint32_t s_last_can_diag_log_ms = 0u;
static uint32_t s_seen_can_recovery_count = 0u;
static uint32_t s_auto_mode_start_ms = 0u;
static uint32_t s_auto_mode_wait_ms = DM_AUTO_MODE_DELAY_MS;
static uint8_t s_mode_attempt = 0u;
static DmRuntimeState s_runtime_state = DM_RUNTIME_UNINITIALIZED;

/* 只用于失败诊断；ISR 只复制，不打印 */
static volatile uint32_t s_raw_rx_count = 0u;
static volatile uint32_t s_last_raw_rx_id = 0u;
static volatile uint8_t s_last_raw_rx_data[8];

SystemStatus assemble_dm_motor(void) {
    BusMotorStatus status;
    BspCanStatus can_status;
    uint32_t now_ms;

    /* 以下三步与成功示例 assemble_motor() 完全相同 */
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
    s_external_target_active = false;
    s_last_sent_target_valid = false;
    s_feedback_rx_ms = 0u;
    s_feedback_sequence = s_enable_feedback_sequence = 0u;
    s_release_requested = s_release_command_sent = s_rearm_pending = false;
    s_resume_limits_valid = false;
    s_release_feedback_sequence = 0u;
    s_last_release_tx_ms = s_last_release_query_ms = now_ms;
    s_resume_started_ms = s_resume_stage_ms = s_resume_last_tx_ms = now_ms;
    s_resume_feedback_sequence = 0u;
    s_resume_query_sent = false;
    s_resume_wait_reason = "mode";
    s_target_position = 0.0f;
    s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
    s_last_feedback_seen_ms = now_ms;
    s_last_target_tx_ms = now_ms;
    s_last_feedback_log_ms = now_ms;
    s_last_stale_log_ms = now_ms;
    s_last_can_diag_log_ms = now_ms - DM_CAN_DIAG_LOG_PERIOD_MS;
    s_seen_can_recovery_count = can_get_recovery_count(&hfdcan1);
    s_auto_mode_start_ms = now_ms;
    s_auto_mode_wait_ms = DM_AUTO_MODE_DELAY_MS;
    s_mode_attempt = 0u;
    s_runtime_state = DM_RUNTIME_UNINITIALIZED;
    s_raw_rx_count = 0u;
    s_last_raw_rx_id = 0u;
    memset((void*)s_last_raw_rx_data, 0, sizeof(s_last_raw_rx_data));

    /* 成功示例：CAN1 收发器先上电并稳定 10 ms，再启动 FDCAN */
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

    /*
     * Do not transmit motor commands during boot. The MCU can reach this point
     * before the motor rail/transceivers are ready; with CAN auto-retransmit a
     * single unacknowledged frame can drive TEC to Bus-Off before entry_loop()
     * starts. clear-error/disable/mode/enable are therefore performed by the
     * runtime handshake after the system-wide 500 ms settle delay
     */
    log_info("DM init transport ready; motor commands deferred to runtime");

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
    uint32_t feedback_sequence = 0u;
    uint32_t primask;
    uint32_t recovery_count = can_get_recovery_count(&hfdcan1);

    if(recovery_count != s_seen_can_recovery_count) {
        s_seen_can_recovery_count = recovery_count;
        if(s_release_requested || s_rearm_pending) {
            if(s_rearm_pending) log_warn("DM resume stop reason=CAN recovery");
            s_release_requested = false;
            assemble_dm_motor_set_released(true);
            s_feedback_seen = false;
            s_feedback_updated = false;
            (void)dm_motor_invalidate_mode_confirmation((BusMotorId)DM_GIMBAL_LOGICAL_ID);
            (void)bus_motor_driver_reset_profile((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        }
        else if(!s_fault_latched) {
            log_warn("DM CAN recovery observed count=%lu; restart POS_VEL handshake",
                     (unsigned long)recovery_count);
            dm_restart_runtime_handshake(now, DM_AUTO_MODE_DELAY_MS);
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if(s_feedback_updated) {
        feedback = s_feedback;
        feedback_sequence = s_feedback_sequence;
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

    if(updated) {
        s_feedback_seen = true;
        s_last_feedback_seen_ms = s_feedback_rx_ms;
    }

    /* 与成功示例 motor_test_process() 相同：故障只在主循环执行安全失能 */
    if(fault_pending) {
        BusMotorStatus disable_status;

        s_fault_latched = true;
        s_enabled = false;
        s_hold_target_ready = false;
        s_runtime_state = DM_RUNTIME_FAILED;
        disable_status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        log_error("DM fault latched: code=0x%02X disable=%s",
                  fault_code, bus_motor.status_str(disable_status));
        if(s_rearm_pending) {
            dm_abort_resume("motor fault");
            return;
        }
        if(!s_release_requested) return;
    }

    if(s_release_requested) {
        dm_process_released(now);
        return;
    }
    if(s_rearm_pending) {
        dm_process_resume(now);
        return;
    }

    if(can_tx_ready(&hfdcan1) &&
       s_runtime_state == DM_RUNTIME_WAIT_AUTO_MODE &&
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
            log_error("DM POS_VEL entry failed after %u attempts; retry cycle in %u ms",
                      DM_MODE_MAX_RETRY,
                      DM_MODE_RETRY_FOREVER_DELAY_MS);
            dm_log_rx_diagnostic();
            dm_log_can_diagnostic("POS_VEL entry");
            dm_restart_runtime_handshake(now, DM_MODE_RETRY_FOREVER_DELAY_MS);
        }
    }

    now = HAL_GetTick();
    if(updated) {
        s_feedback_seen = true;
        s_last_feedback_seen_ms = s_feedback_rx_ms;

        if(s_runtime_state == DM_RUNTIME_WAIT_FEEDBACK && s_enabled && !s_fault_latched &&
           feedback.error_code == 0x01u &&
           (int32_t)(feedback_sequence - s_enable_feedback_sequence) > 0) {
            dm_try_enter_ready(now, &feedback);
            if(s_release_requested) return;
        }

        if((uint32_t)(now - s_last_feedback_log_ms) >= DM_FEEDBACK_LOG_PERIOD_MS) {
            s_last_feedback_log_ms = now;
            log_info("DM fb pos=%.3f spd=%.3f tor=%.3f state=0x%02X ready=%u",
                     (double)feedback.position,
                     (double)feedback.velocity,
                     (double)feedback.torque,
                     feedback.error_code,
                     assemble_dm_motor_is_ready() ? 1u : 0u);
        }
    }

    if(s_external_target_active &&
       ((uint32_t)(now - s_external_target_ms) > s_external_target_timeout_ms ||
        !dm_feedback_is_fresh(now))) {
        assemble_dm_motor_clear_control_target();
    }
    if(can_tx_ready(&hfdcan1) &&
       s_runtime_state == DM_RUNTIME_READY && s_enabled && !s_fault_latched &&
       s_hold_target_ready &&
       (uint32_t)(now - s_last_target_tx_ms) >= DM_TARGET_TX_PERIOD_MS) {
        s_last_target_tx_ms = now;
        if(!dm_send_current_target()) {
            dm_log_can_diagnostic("target resend");
        }
    }

    if(s_feedback_seen && !dm_feedback_is_fresh(now)) {
        if((uint32_t)(now - s_last_stale_log_ms) >= DM_STALE_LOG_PERIOD_MS) {
            s_last_stale_log_ms = now;
            log_warn("DM feedback stale; restart POS_VEL handshake age=%lu ms",
                     (unsigned long)(now - s_last_feedback_seen_ms));
            dm_log_can_diagnostic("feedback stale");
        }
        if(!s_fault_latched) {
            dm_restart_runtime_handshake(now, DM_MODE_RETRY_DELAY_MS);
        }
    }
}

void assemble_dm_motor_set_position_limits(float min_position, float max_position) {
    s_resume_limits_valid = isfinite(min_position) && isfinite(max_position) &&
        min_position <= max_position;
    if(s_resume_limits_valid) {
        s_resume_min_position = min_position;
        s_resume_max_position = max_position;
    }
}

void assemble_dm_motor_set_released(bool released) {
    uint32_t now = HAL_GetTick();

    if(released == s_release_requested) return;
    if(!released && s_fault_latched) return;

    s_release_requested = released;
    s_enabled = false;
    s_hold_target_ready = false;
    s_external_target_active = false;
    s_last_sent_target_valid = false;
    s_target_position = 0.0f;
    s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
    s_mode_attempt = 0u;
    s_rearm_pending = !released;
    if(released) {
        s_release_command_sent = false;
        s_release_feedback_sequence = s_feedback_sequence;
        s_last_release_tx_ms = now - DM_RELEASE_RETRY_MS;
        s_last_release_query_ms = now - DM_RELEASE_QUERY_MS;
        s_runtime_state = DM_RUNTIME_RELEASED;
        dm_process_released(now);
    }
    else {
        s_runtime_state = DM_RUNTIME_WAIT_AUTO_MODE;
        s_resume_started_ms = s_resume_stage_ms = now;
        s_resume_last_tx_ms = now - DM_RESUME_RETRY_MS;
        s_resume_query_sent = false;
        s_resume_wait_reason = "mode";
        log_info("DM resume mode");
        s_auto_mode_start_ms = now;
        s_auto_mode_wait_ms = 0u;
        (void)dm_motor_invalidate_mode_confirmation((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        (void)bus_motor_driver_reset_profile((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    }
}

bool assemble_dm_motor_is_released(void) {
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    bool released = s_release_requested && s_release_command_sent && s_feedback_seen &&
        (int32_t)(s_feedback_sequence - s_release_feedback_sequence) > 0 &&
        (uint32_t)(HAL_GetTick() - s_feedback_rx_ms) <= DM_RELEASE_FEEDBACK_MAX_AGE_MS &&
        s_feedback.error_code == 0x00u &&
        (s_feedback.valid & BUS_MOTOR_FEEDBACK_POSITION) && isfinite(s_feedback.position);
    __set_PRIMASK(key);
    return released && can_tx_ready(&hfdcan1) &&
        s_seen_can_recovery_count == can_get_recovery_count(&hfdcan1);
}

static void dm_process_released(uint32_t now_ms) {
    if(!can_tx_ready(&hfdcan1)) return;
    if((uint32_t)(now_ms - s_last_release_tx_ms) >= DM_RELEASE_RETRY_MS) {
        uint32_t sequence = s_feedback_sequence;
        s_last_release_tx_ms = now_ms;
        if(bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID) == MOTOR_STATUS_OK &&
           !s_release_command_sent) {
            s_release_feedback_sequence = sequence;
            s_release_command_sent = true;
        }
    }
    if((uint32_t)(now_ms - s_last_release_query_ms) >= DM_RELEASE_QUERY_MS) {
        s_last_release_query_ms = now_ms;
        (void)dm_request_feedback();
    }
}

static bool dm_request_feedback(void) {
    /* 官方 motor-sdk 的 0xCC 状态读取，失能时也更新角度 */
    const uint8_t request[8] = { DM_GIMBAL_CAN_ID & 0xFFu,
        (DM_GIMBAL_CAN_ID >> 8) & 0xFFu, 0xCCu, 0u, 0u, 0u, 0u, 0u };
    return dm_motor_can_send(0x7FFu, request, sizeof(request));
}

static void dm_abort_resume(const char* reason) {
    const char* phase = s_runtime_state == DM_RUNTIME_WAIT_AUTO_MODE ? "mode" :
        s_runtime_state == DM_RUNTIME_WAIT_RESUME_FEEDBACK ? "position" : "enabled";
    log_warn("DM resume stop phase=%s reason=%s", phase, reason);
    assemble_dm_motor_set_released(true);
}

static void dm_process_resume(uint32_t now_ms) {
    BusMotorStatus status;
    BusMotorFeedback current;
    uint32_t sequence, age, key;
    bool valid;

    if((uint32_t)(now_ms - s_resume_started_ms) >= DM_RESUME_TIMEOUT_MS) {
        dm_abort_resume(s_resume_wait_reason);
        return;
    }
    if(!can_tx_ready(&hfdcan1)) {
        s_resume_wait_reason = "CAN busy";
        return;
    }
    if(s_runtime_state == DM_RUNTIME_WAIT_AUTO_MODE) {
        /* 给现有阻塞模式 ACK 留出预算，不延长总恢复时间 */
        if((uint32_t)(now_ms - s_resume_last_tx_ms) < DM_RESUME_RETRY_MS ||
           (uint32_t)(now_ms - s_resume_started_ms) >
               DM_RESUME_TIMEOUT_MS - DM_RESUME_MODE_BUDGET_MS) return;
        status = dm_motor_clear_error((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        if(status == MOTOR_STATUS_OK)
            status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        if(status == MOTOR_STATUS_OK)
            status = bus_motor.profile.activate((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                                BUS_MOTOR_PROFILE_POSITION);
        now_ms = HAL_GetTick();
        s_resume_last_tx_ms = now_ms;
        if(s_fault_pending || s_fault_latched) {
            dm_abort_resume("motor fault");
            return;
        }
        if(status != MOTOR_STATUS_OK) {
            s_resume_wait_reason = bus_motor.status_str(status);
            return;
        }
        if((uint32_t)(now_ms - s_resume_started_ms) >= DM_RESUME_TIMEOUT_MS) {
            dm_abort_resume("mode timeout");
            return;
        }
        /* ACK 前的角度不可接管，先查询并等待新序号 */
        s_resume_feedback_sequence = s_feedback_sequence;
        s_resume_stage_ms = now_ms;
        s_runtime_state = DM_RUNTIME_WAIT_RESUME_FEEDBACK;
        s_resume_wait_reason = "position feedback";
        s_last_release_query_ms = now_ms;
        s_resume_query_sent = dm_request_feedback();
        log_info("DM resume wait position");
        return;
    }

    if((uint32_t)(now_ms - s_resume_stage_ms) >= DM_RESUME_FEEDBACK_TIMEOUT_MS) {
        dm_abort_resume(s_resume_wait_reason);
        return;
    }
    if((uint32_t)(now_ms - s_last_release_query_ms) >= DM_RELEASE_QUERY_MS) {
        s_last_release_query_ms = now_ms;
        if(dm_request_feedback()) s_resume_query_sent = true;
    }
    key = __get_PRIMASK();
    __disable_irq();
    current = s_feedback;
    sequence = s_feedback_sequence;
    age = HAL_GetTick() - s_feedback_rx_ms;
    valid = s_feedback_seen;
    __set_PRIMASK(key);
    if(s_fault_pending || s_fault_latched) {
        dm_abort_resume("motor fault");
        return;
    }
    if(!valid || !s_resume_query_sent || age > DM_RESUME_FEEDBACK_MAX_AGE_MS) return;
    if((int32_t)(sequence - (s_runtime_state == DM_RUNTIME_WAIT_RESUME_FEEDBACK ?
        s_resume_feedback_sequence : s_enable_feedback_sequence)) <= 0) return;
    if(!(current.valid & BUS_MOTOR_FEEDBACK_POSITION) || !isfinite(current.position) ||
       !s_resume_limits_valid || current.position < s_resume_min_position ||
       current.position > s_resume_max_position) {
        dm_abort_resume("position limit");
        return;
    }
    now_ms = HAL_GetTick();
    if((uint32_t)(now_ms - s_resume_last_tx_ms) < DM_RESUME_RETRY_MS) return;
    s_resume_last_tx_ms = now_ms;
    if(s_runtime_state == DM_RUNTIME_WAIT_RESUME_FEEDBACK && current.error_code != 0x00u) {
        (void)bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        s_resume_wait_reason = "disabled feedback";
        return;
    }
    if(s_runtime_state == DM_RUNTIME_WAIT_FEEDBACK && current.error_code != 0x01u) return;

    /* 每次重试都取当前新反馈，绝不复用上次预置位置 */
    status = dm_motor_set_pos_vel((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                  current.position, DM_DEFAULT_TARGET_SPEED_RAD_S);
    if(status != MOTOR_STATUS_OK) {
        s_resume_wait_reason = s_runtime_state == DM_RUNTIME_WAIT_RESUME_FEEDBACK ?
            "preload busy" : "hold busy";
        return;
    }
    if(s_runtime_state == DM_RUNTIME_WAIT_RESUME_FEEDBACK) {
        if(s_fault_pending || !can_tx_ready(&hfdcan1)) return;
        s_enable_feedback_sequence = s_feedback_sequence;
        status = bus_motor.basic.enable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        if(status != MOTOR_STATUS_OK) {
            s_resume_wait_reason = "enable busy";
            return;
        }
        s_enabled = true;
        s_runtime_state = DM_RUNTIME_WAIT_FEEDBACK;
        s_resume_stage_ms = HAL_GetTick();
        s_resume_last_tx_ms = s_resume_stage_ms - DM_RESUME_RETRY_MS;
        s_last_release_query_ms = s_resume_stage_ms - DM_RELEASE_QUERY_MS;
        s_resume_wait_reason = "enabled feedback";
        log_info("DM resume wait enabled");
        return;
    }
    s_target_position = current.position;
    s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
    s_last_sent_target = current.position;
    s_last_sent_target_valid = true;
    s_hold_target_ready = true;
    s_rearm_pending = false;
    s_last_target_tx_ms = HAL_GetTick();
    s_runtime_state = DM_RUNTIME_READY;
    log_info("DM resume ready pos=%.3f rad", (double)current.position);
}

bool assemble_dm_motor_has_feedback(void) {
    return dm_feedback_is_fresh(HAL_GetTick());
}

bool assemble_dm_motor_is_ready(void) {
    return !s_release_requested && can_tx_ready(&hfdcan1) &&
           s_runtime_state == DM_RUNTIME_READY &&
           s_enabled &&
           !s_fault_latched &&
           s_hold_target_ready &&
           dm_feedback_is_fresh(HAL_GetTick());
}

bool assemble_dm_motor_get_position(float* position) {
    uint32_t primask;

    if(position == NULL || !dm_feedback_is_fresh(HAL_GetTick())) {
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
     * in the same 10 ms window
     */
    s_target_position = position;
    s_target_speed = speed;
    return true;
}

static bool dm_feedback_is_fresh(uint32_t now_ms) {
    return s_feedback_seen &&
           (uint32_t)(now_ms - s_feedback_rx_ms) <= DM_FEEDBACK_STALE_TIMEOUT_MS;
}

bool assemble_dm_motor_get_last_sent_target(float *position) {
    if(!position || !s_last_sent_target_valid || !assemble_dm_motor_is_ready()) return false;
    *position=s_last_sent_target;
    return true;
}
static bool dm_send_current_target(void) {
    bool ok=dm_motor_set_pos_vel((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                s_target_position,s_target_speed)==MOTOR_STATUS_OK;
    if(ok) { s_last_sent_target=s_target_position; s_last_sent_target_valid=true; }
    return ok;
}

static bool dm_enter_pos_vel_reference(void) {
    BusMotorStatus status;
    BusMotorStatus cleanup_status;

    /* Runtime sequence: clear fault -> disable -> select POS_VEL -> enable */
    status = dm_motor_clear_error((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    if(status == MOTOR_STATUS_OK) {
        status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    }
    if(status == MOTOR_STATUS_OK) {
        status = bus_motor.profile.activate((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                            BUS_MOTOR_PROFILE_POSITION);
    }
    if(status == MOTOR_STATUS_OK) {
        s_enable_feedback_sequence = s_feedback_sequence;
        status = bus_motor.basic.enable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    }

    if(status != MOTOR_STATUS_OK) {
        cleanup_status = bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        log_error("DM mode switch failed: status=%s cleanup_disable=%s",
                  bus_motor.status_str(status),
                  bus_motor.status_str(cleanup_status));
        dm_log_rx_diagnostic();
        dm_log_can_diagnostic("mode switch");
        return false;
    }

    return true;
}

static void dm_try_enter_ready(uint32_t now_ms, const BusMotorFeedback* feedback) {
    BusMotorStatus status;

    if(feedback == NULL || !can_tx_ready(&hfdcan1) ||
       s_runtime_state != DM_RUNTIME_WAIT_FEEDBACK ||
       !s_enabled || s_fault_latched || s_release_requested) {
        return;
    }

    /* 第一帧使能后反馈作为安全保持点，再交给 IMU Yaw 改目标 */
    s_target_position = feedback->position;
    s_target_speed = DM_DEFAULT_TARGET_SPEED_RAD_S;
    status = dm_motor_set_pos_vel((BusMotorId)DM_GIMBAL_LOGICAL_ID,
                                  s_target_position,
                                  s_target_speed);
    if(status != MOTOR_STATUS_OK) {
        s_hold_target_ready = false;
        s_enabled = false;
        (void)bus_motor.basic.disable((BusMotorId)DM_GIMBAL_LOGICAL_ID);
        log_error("DM initial hold failed: %s; restart handshake", bus_motor.status_str(status));
        dm_log_can_diagnostic("initial hold");
        dm_restart_runtime_handshake(now_ms, DM_MODE_RETRY_DELAY_MS);
        return;
    }

    s_last_sent_target=s_target_position;
    s_last_sent_target_valid=true;
    s_hold_target_ready = true;
    s_rearm_pending = false;
    s_last_target_tx_ms = now_ms;
    s_runtime_state = DM_RUNTIME_READY;
    log_info("DM ready: hold current pos=%.3f rad", (double)feedback->position);
}

static void dm_restart_runtime_handshake(uint32_t now_ms, uint32_t delay_ms) {
    s_external_target_active = false;
    s_last_sent_target_valid = false;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_feedback_updated = false;
    s_feedback_seen = false;
    if(primask == 0u) {
        __enable_irq();
    }

    s_enabled = false;
    s_hold_target_ready = false;
    s_runtime_state = DM_RUNTIME_WAIT_AUTO_MODE;
    s_mode_attempt = 0u;
    s_auto_mode_start_ms = now_ms;
    s_auto_mode_wait_ms = delay_ms;
    (void)dm_motor_invalidate_mode_confirmation((BusMotorId)DM_GIMBAL_LOGICAL_ID);
    (void)bus_motor_driver_reset_profile((BusMotorId)DM_GIMBAL_LOGICAL_ID);
}

static void dm_log_can_diagnostic(const char* action) {
    BspCanHealth health;
    uint32_t now = HAL_GetTick();

    if((uint32_t)(now - s_last_can_diag_log_ms) < DM_CAN_DIAG_LOG_PERIOD_MS) {
        return;
    }
    s_last_can_diag_log_ms = now;

    if(can_get_health(&hfdcan1, &health) == STM32_HAL_CAN_OK) {
        log_warn("DM CAN %s can=%s lec=%lu fault_lec=%lu fault_irq=0x%lX busoff=%lu txerr=%lu rxerr=%lu free=%lu fail=%lu recovery=%lu",
                 action,
                 can_error_code_to_str(health.last_tx_status),
                 (unsigned long)health.last_error_code,
                 (unsigned long)health.last_fault_error_code,
                 (unsigned long)health.last_fault_irq_flags,
                 (unsigned long)health.bus_off,
                 (unsigned long)health.tx_error_count,
                 (unsigned long)health.rx_error_count,
                 (unsigned long)health.tx_fifo_free_level,
                 (unsigned long)health.consecutive_tx_failures,
                 (unsigned long)health.recovery_count);
    }
}

static bool dm_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    /* dm_ok 的 can_send() 始终发送 Classical CAN + BRS OFF */
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

    /* 成功示例 assemble_motor_can_rx() 原始顺序：参数 ACK 必须先解析 */
    if(dm_motor_parse_parameter_frame(header->Identifier, data)) {
        return;
    }

    if(dm_motor_parse_feedback_frame(header->Identifier, data, &feedback) == MOTOR_STATUS_OK &&
       feedback.id == (BusMotorId)DM_GIMBAL_LOGICAL_ID) {
        s_feedback_rx_ms = HAL_GetTick();
        s_feedback = feedback;
        ++s_feedback_sequence;
        g_yaw_velocity = feedback.velocity;  /* rad/s */
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

/* Read-only coherent snapshot; return false for absent/stale feedback */
bool assemble_dm_motor_get_feedback_snapshot(BusMotorFeedback* feedback) {
    if(feedback == NULL) return false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    bool fresh = dm_feedback_is_fresh(HAL_GetTick());
    if(fresh) {
        *feedback = s_feedback;
    }
    __set_PRIMASK(primask);
    return fresh;
}

/* Interface plumbing only; control tuning lives in gimbal_control.c */
bool assemble_dm_motor_get_feedback_snapshot_age(BusMotorFeedback* feedback, uint32_t* age_ms) {
    if(feedback == NULL || age_ms == NULL) return false;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    bool valid = s_feedback_seen;
    *age_ms = HAL_GetTick() - s_feedback_rx_ms;
    if(valid) *feedback = s_feedback;
    __set_PRIMASK(key);
    return valid;
}
bool assemble_dm_motor_set_target_position_guarded(float position, float speed, uint32_t timeout_ms) {
    if(timeout_ms == 0u || !assemble_dm_motor_set_target_position(position, speed)) return false;
    s_external_target_ms = HAL_GetTick();
    s_external_target_timeout_ms = timeout_ms;
    s_external_target_active = true;
    return true;
}
void assemble_dm_motor_clear_control_target(void) {
    if(s_release_requested) {
        s_external_target_active = false;
        s_last_sent_target_valid = false;
        return;
    }
    if(!s_external_target_active) return;
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    /* Stop advancing the corrective target; hold the last received position
     * CAN loss can prevent delivery: this is not a hardware emergency stop
     */
    if(s_feedback_seen && (s_feedback.valid & BUS_MOTOR_FEEDBACK_POSITION) && isfinite(s_feedback.position))
        s_target_position = s_feedback.position;
    s_external_target_active = false;
    __set_PRIMASK(key);
}
