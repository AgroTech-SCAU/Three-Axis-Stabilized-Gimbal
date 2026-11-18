#include "assemble.h"

#include "attitude.h"
#include "gimbal_config.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"
#include "stm32_hal_uart.h"
#include "user_io.h"

#include <stdio.h>

// ! ========================= 变 量 声 明 ========================= ! //

static ms_t s_gimbal_motor_bus_wait_log_timer;
static bool s_gimbal_motor_bus_ht_ready_logged;
static ms_t s_gimbal_can1_recovery_error_log_timer;
static ms_t s_gimbal_can1_recovery_progress_log_timer;
static bool s_gimbal_can1_recovery_active;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool gimbal_motor_bus_ht_ready(void);
static void gimbal_process_motor_bus(void);
static bool gimbal_read_motor(GimbalAxis axis, GimbalMotorFeedback* motor);
static bool gimbal_set_motor_target(GimbalAxis axis, float position, float speed, uint32_t timeout_ms);
static void gimbal_clear_motor_target(GimbalAxis axis);
static void gimbal_format_led_diagnostics(char* text, uint32_t capacity);

// ! ========================= 变 量 定 义 ========================= ! //

static const GimbalPortOps gimbal_ops = {
    .process_motor_bus = gimbal_process_motor_bus,
    .read_motor = gimbal_read_motor,
    .set_motor_target = gimbal_set_motor_target,
    .clear_motor_target = gimbal_clear_motor_target,
    .read_console_byte = uart1_read_byte,
    .take_console_rx_reset = uart1_take_rx_reset,
    .write_console = uart1_write_it,
    .format_led_diagnostics = gimbal_format_led_diagnostics,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_gimbal(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK || assemble_log() != SYSTEM_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    log_info("BOOT gimbal-v1.4-20261118");
    if(assemble_user_io() != SYSTEM_STATUS_OK || assemble_imu() != SYSTEM_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    AttitudeConfig attitude_config = attitude.default_config();
    if(attitude.init(&attitude_config) != ATTITUDE_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    delay_ms(100u);
    if(assemble_dm_motor() != SYSTEM_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    delay_ms(100u);
    if(assemble_ht_motor() != SYSTEM_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    delay_ms(100u);
    if(!uart1_console_init()) {
        log_warn("UART RX retry");
    }
    delay_ms(500u);
    GimbalConfig config = gimbal_project_config(&gimbal_ops);
    return gimbal.init(&config) == GIMBAL_STATUS_OK ? SYSTEM_STATUS_OK : SYSTEM_STATUS_ERROR;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/** @brief 灯态摘要，tx 为完成数/接受数 */
static void gimbal_format_led_diagnostics(char* text, uint32_t capacity) {
    if(text == NULL || capacity == 0u) {
        return;
    }
    static const char* const indicator_names[] = { "WAIT", "OFF", "FAULT" };
    const UserIoState* state = user_io.get_state();
    const char* target = "INIT";
    if(state->initialized && (unsigned)state->indicator < sizeof(indicator_names) / sizeof(indicator_names[0])) {
        target = indicator_names[state->indicator];
    }
    const char* submit = !state->submit_attempted ? "NONE" : state->last_submit_status == USER_IO_STATUS_OK     ? "OK"
                                                         : state->last_submit_status == USER_IO_STATUS_LED_BUSY ? "BUSY"
                                                                                                                : "ERR";
    (void)snprintf(text, capacity,
                   "state=%s init=%s send=%s tx=%lu/%lu err=%lu/%lu",
                   target, state->led_init_ok ? "OK" : "FAIL", submit,
                   (unsigned long)state->tx.completed_count, (unsigned long)state->tx.started_count,
                   (unsigned long)state->error_count, (unsigned long)state->tx.error_count);
}

/**
 * @brief CAN 恢复及 HT 到 DM 的启动顺序
 */
static void gimbal_process_motor_bus(void) {
    {
        bool can1_recovered = false;
        BspCanStatus can1_status = can_service_recovery(&hfdcan1, &can1_recovered);
        BspCanHealth health;
        bool have_health = can_get_health(&hfdcan1, &health) == STM32_HAL_CAN_OK;

        if(have_health && health.recovery_in_progress && !s_gimbal_can1_recovery_active) {
            s_gimbal_can1_recovery_active = true;
            log_warn("CAN1 recovery started cause_lec=%lu fault_irq=0x%lX fault_lec=%lu fault_act=%lu fault_ep=%lu fault_warn=%lu fault_busoff=%lu fault_txerr=%lu fault_rxerr=%lu free=%lu backoff=%lu ms",
                     (unsigned long)health.last_recovery_error_code,
                     (unsigned long)health.last_fault_irq_flags,
                     (unsigned long)health.last_fault_error_code,
                     (unsigned long)health.last_fault_activity,
                     (unsigned long)health.last_fault_error_passive,
                     (unsigned long)health.last_fault_warning,
                     (unsigned long)health.last_fault_bus_off,
                     (unsigned long)health.last_fault_tx_error_count,
                     (unsigned long)health.last_fault_rx_error_count,
                     (unsigned long)health.last_recovery_tx_fifo_free_level,
                     (unsigned long)health.recovery_backoff_ms);
        }

        if(have_health && health.recovery_in_progress && s_gimbal_can1_recovery_active &&
           delay_nb_ms(&s_gimbal_can1_recovery_progress_log_timer, 2000u)) {
            log_warn("CAN1 recovery waiting age=%lu ms lec=%lu act=%lu ep=%lu busoff=%lu txerr=%lu rxerr=%lu free=%lu",
                     (unsigned long)(HAL_GetTick() - health.recovery_started_ms),
                     (unsigned long)health.last_error_code,
                     (unsigned long)health.activity,
                     (unsigned long)health.error_passive,
                     (unsigned long)health.bus_off,
                     (unsigned long)health.tx_error_count,
                     (unsigned long)health.rx_error_count,
                     (unsigned long)health.tx_fifo_free_level);
        }

        if(can1_recovered) {
            s_gimbal_can1_recovery_active = false;
            if(have_health) {
                log_warn("CAN1 recovery complete count=%lu lec=%lu act=%lu busoff=%lu txerr=%lu rxerr=%lu free=%lu",
                         (unsigned long)health.recovery_count,
                         (unsigned long)health.last_error_code,
                         (unsigned long)health.activity,
                         (unsigned long)health.bus_off,
                         (unsigned long)health.tx_error_count,
                         (unsigned long)health.rx_error_count,
                         (unsigned long)health.tx_fifo_free_level);
            }
            else {
                log_warn("CAN1 recovery complete");
            }
        }
        else if(can1_status != STM32_HAL_CAN_OK &&
                delay_nb_ms(&s_gimbal_can1_recovery_error_log_timer, 1000u)) {
            log_error("CAN1 recovery service: %s", can_error_code_to_str(can1_status));
        }
    }

    /* 两台 HT 反馈有效后启动 DM 握手，减少上电总线竞争 */
    assemble_ht_motor_process();
    if(gimbal_motor_bus_ht_ready()) {
        if(!s_gimbal_motor_bus_ht_ready_logged) {
            s_gimbal_motor_bus_ht_ready_logged = true;
            log_info("MOTOR BUS verified by HT feedback; DM handshake released");
        }
        assemble_dm_motor_process();
    }
    else if(delay_nb_ms(&s_gimbal_motor_bus_wait_log_timer, 1000u)) {
        log_info("MOTOR BUS discovery: waiting for HT id=2/8 feedback before DM handshake");
    }
}

static bool gimbal_motor_bus_ht_ready(void) {
    return assemble_ht_motor_has_feedback(GIMBAL_PITCH_MOTOR_ID) &&
           assemble_ht_motor_has_feedback(GIMBAL_ROLL_MOTOR_ID);
}

static bool gimbal_read_motor(GimbalAxis axis, GimbalMotorFeedback* motor) {
    BusMotorFeedback feedback = { 0 };
    bool found;
    if(axis == GIMBAL_AXIS_YAW) {
        found = assemble_dm_motor_get_feedback_snapshot_age(&feedback, &motor->age_ms);
        found = found && assemble_dm_motor_is_ready() && gimbal_motor_bus_ht_ready();
    }
    else {
        uint16_t id = axis == GIMBAL_AXIS_ROLL ? GIMBAL_ROLL_MOTOR_ID : GIMBAL_PITCH_MOTOR_ID;
        found = assemble_ht_motor_get_feedback_snapshot_age(id, &feedback, &motor->age_ms);
    }
    motor->ready = found && (feedback.valid & BUS_MOTOR_FEEDBACK_POSITION) != 0u;
    motor->position = feedback.position;
    motor->velocity = feedback.velocity;
    motor->velocity_valid = found && (feedback.valid & BUS_MOTOR_FEEDBACK_VELOCITY) != 0u;
    return motor->ready;
}

static bool gimbal_set_motor_target(GimbalAxis axis, float position, float speed, uint32_t timeout_ms) {
    if(axis == GIMBAL_AXIS_YAW) {
        return assemble_dm_motor_set_target_position_guarded(position, speed, timeout_ms);
    }
    uint16_t id = axis == GIMBAL_AXIS_ROLL ? GIMBAL_ROLL_MOTOR_ID : GIMBAL_PITCH_MOTOR_ID;
    return assemble_ht_motor_set_control_target(id, position, speed, timeout_ms);
}

static void gimbal_clear_motor_target(GimbalAxis axis) {
    if(axis == GIMBAL_AXIS_YAW) {
        assemble_dm_motor_clear_control_target();
    }
    else {
        uint16_t id = axis == GIMBAL_AXIS_ROLL ? GIMBAL_ROLL_MOTOR_ID : GIMBAL_PITCH_MOTOR_ID;
        assemble_ht_motor_cancel_control(id);
    }
}
