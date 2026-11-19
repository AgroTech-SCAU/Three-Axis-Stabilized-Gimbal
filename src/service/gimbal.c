#include "gimbal.h"

#include "delay.h"
#include "log.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define GIMBAL_IMU_FAULT_MS 200u
#define GIMBAL_RELEASE_TIMEOUT_MS 500u
#define GIMBAL_REARM_RETRY_MS 250u
#define GIMBAL_RAW_DURATION_MS 10000u
#define GIMBAL_RAD_TO_DEG 57.295779513f
#define GIMBAL_LOG_FRAME_CAPACITY 1280u

static GimbalConfig gimbal_config;
static GimbalState gimbal_state;
static GimbalMeasurement measurement;
static float hold_position[GIMBAL_AXIS_COUNT];
static bool hold_captured;
static bool hold_submitted;
static bool telemetry_enabled = true;
static bool raw_enabled;
static bool imu_bad;
static bool imu_fault_active;
static bool rearm_motors_requested;
static uint8_t run_pending_mask;
static GimbalControlStatus run_wait_status;
static GimbalFault run_wait_reason;
static GimbalAxis run_wait_axis;
static uint8_t run_wait_axis_mask;
static bool run_wait_logged;
static GimbalInputDiagnostic run_first_bad;
static uint32_t hold_reject_ms[GIMBAL_AXIS_COUNT];
static uint32_t imu_bad_since_ms;
static uint32_t last_run_good_ms;
static uint32_t raw_started_ms;
static uint32_t mode_started_ms;
static uint32_t rearm_retry_ms[GIMBAL_AXIS_COUNT];
static uint32_t last_hold_ms;
static uint32_t last_log_ms;
static uint32_t last_status_ms;
static char console_line[32];
static uint8_t console_line_length;
static bool console_line_discard;
static bool console_cr_pending;
/* 主循环独占；UART 端口同步复制整包，不引用该缓冲的异步生命周期 */
static char log_frame[GIMBAL_LOG_FRAME_CAPACITY];
static size_t log_frame_length;
static bool log_frame_valid;
static uint32_t log_frame_sequence;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool gimbal_feedback_ready(void);
static bool gimbal_feedback_stationary(void);
static bool gimbal_raw_imu_fresh(uint32_t now_us);
static void gimbal_update_imu_health(AttitudeStatus status, uint32_t now_us);
static void gimbal_update_prepare_fault(void);
static void gimbal_set_fault(GimbalFault reason, GimbalAxis axis, bool latched);
static void gimbal_safety_stop(GimbalFault reason, GimbalAxis axis, GimbalControlStatus status);
static bool gimbal_wait_for_run(GimbalFault reason, GimbalAxis axis, GimbalControlStatus status);
static const char* gimbal_fault_name(GimbalFault reason);
static const char* gimbal_blocker_name(GimbalControlStatus status);
static const char* gimbal_motor_diagnostic_name(GimbalMotorDiagnosticReason reason);
static const char* gimbal_diagnostic_axes(uint8_t mask);
static const char* gimbal_stop_reason(void);
static void gimbal_capture_input(GimbalInputDiagnostic* diagnostic, GimbalFault reason, GimbalControlStatus status, GimbalAxis axis);
static void gimbal_write_wait_event(GimbalFault reason, GimbalControlStatus status);
static void gimbal_write_stop_event(void);
static void gimbal_read_feedback(void);
static void gimbal_update_measurement(void);
static void gimbal_clear_targets(void);
static void gimbal_capture_hold(void);
static void gimbal_process_hold(void);
static void gimbal_process_control(uint32_t now_us);
static void gimbal_process_console(void);
static void gimbal_handle_console_line(void);
static void gimbal_enter_run(void);
static void gimbal_enter_manual(void);
static void gimbal_process_manual(void);
static void gimbal_process_rearm(uint32_t now_us);
static bool gimbal_all_motors_released(void);
static GimbalControlStatus gimbal_check_rearm(uint32_t now_us);
static bool gimbal_rearm_rejected(uint32_t now_us);
static void gimbal_write_telemetry(uint32_t now_us);
static void gimbal_write_status(void);
static void gimbal_write_raw(uint32_t now_us);
static void gimbal_format_motor(char* text, size_t capacity, unsigned axis);
static const char* gimbal_pending_axes(uint8_t mask);
static void gimbal_frame_begin(const char* kind);
static void gimbal_frame_append(const char* format, ...);
static void gimbal_frame_send(void);
static void gimbal_write_line(const char* line, int length, size_t capacity);

// ! ========================= 接 口 变 量 定 义 ========================= ! //

const struct GimbalInterface gimbal_interface = {
#define X(name, str) .name = GIMBAL_STATUS_##name,
    GIMBAL_STATUS_TABLE
#undef X
        .init = gimbal_init,
    .process = gimbal_process,
    .acknowledge_fault = gimbal_acknowledge_fault,
    .toggle_target_mode = gimbal_toggle_target_mode,
    .get_state = gimbal_get_state,
    .is_ready = gimbal_is_ready,
    .status_str = gimbal_status_str,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

GimbalStatus gimbal_init(const GimbalConfig* config) {
    if(config == NULL || config->ops == NULL || config->ops->process_motor_bus == NULL ||
       config->ops->read_motor == NULL || config->ops->set_motor_target == NULL ||
       config->ops->clear_motor_target == NULL || config->ops->set_motor_released == NULL ||
       config->ops->is_motor_released == NULL || config->ops->read_console_byte == NULL ||
       config->ops->take_console_rx_reset == NULL || config->ops->write_console == NULL ||
       !isfinite(config->hold_speed) || config->hold_speed <= 0.0f ||
       config->hold_period_ms == 0u || config->hold_timeout_ms <= config->hold_period_ms ||
       config->prepare_feedback_max_age_ms == 0u || config->prepare_feedback_max_age_ms > 200u ||
       config->run_recovery_ms == 0u || config->status_period_ms == 0u || config->raw_log_period_ms < 100u) {
        return GIMBAL_STATUS_INVALID_PARAM;
    }
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        uint64_t period_ms = ((uint64_t)config->control.axis[i].period_us + 999u) / 1000u;
        uint64_t command_age_ms = (uint64_t)config->run_recovery_ms + period_ms + 1u;
        if(config->control.axis[i].period_us == 0u || command_age_ms >= config->control.axis[i].command_timeout_ms) {
            return GIMBAL_STATUS_INVALID_PARAM;
        }
    }
    gimbal_config = *config;
    memset(&gimbal_state, 0, sizeof(gimbal_state));
    memset(&measurement, 0, sizeof(measurement));
    memset(hold_position, 0, sizeof(hold_position));
    memset(hold_reject_ms, 0, sizeof(hold_reject_ms));
    memset(rearm_retry_ms, 0, sizeof(rearm_retry_ms));
    hold_captured = hold_submitted = false;
    telemetry_enabled = true;
    raw_enabled = imu_bad = imu_fault_active = rearm_motors_requested = false;
    run_pending_mask = 0u;
    run_wait_status = GIMBAL_CONTROL_OK;
    run_wait_reason = GIMBAL_FAULT_NONE;
    run_wait_axis = GIMBAL_AXIS_COUNT;
    run_wait_axis_mask = 0u;
    run_wait_logged = false;
    memset(&run_first_bad, 0, sizeof(run_first_bad));
    console_line_length = 0u;
    console_line_discard = false;
    console_cr_pending = false;
    log_frame_sequence = 0u;
    last_log_ms = last_status_ms = delay_now_ms();
    last_hold_ms = last_status_ms - gimbal_config.hold_period_ms;
    last_run_good_ms = raw_started_ms = imu_bad_since_ms = mode_started_ms = last_status_ms;
    gimbal_state.initialized = true;
    gimbal_state.fault_axis = GIMBAL_AXIS_COUNT;
    gimbal_state.blocker = GIMBAL_CONTROL_IMU_INVALID;
    log_info("GIMBAL starting; keep still");
    log_info("CONSOLE status | log on/off | raw on/off | help");
    log_info("COORDINATES %s", config->control.coordinates_confirmed ? "CONFIRMED" : "PENDING");
    return GIMBAL_STATUS_OK;
}

void gimbal_process(void) {
    uint32_t begin_us = delay_now_us();
    gimbal_state.now_ms = delay_now_ms();
    if(!gimbal_state.initialized) {
        return;
    }

    /* 手动拖动与重新接管期间冻结零偏学习 */
    gimbal_read_feedback();
    bool was_calibrated = gimbal_state.attitude_snapshot.calibrated;
    bool learning = !gimbal_state.running && !gimbal_state.manual && !gimbal_state.rearming &&
                    (!was_calibrated || (!gimbal_state.fault && gimbal_feedback_stationary()));
    attitude.set_bias_learning(learning);
    AttitudeStatus imu_status = attitude.process(begin_us);
    (void)attitude.get_snapshot(&gimbal_state.attitude_snapshot);
    gimbal_update_measurement();

    /* 保持旧入口的顺序：IMU -> CAN恢复 -> HT -> HT就绪门控DM -> 控制 */
    gimbal_config.ops->process_motor_bus();
    gimbal_read_feedback();
    uint32_t now_us = delay_now_us();
    gimbal_state.now_ms = delay_now_ms();
    if((uint32_t)(now_us - measurement.timestamp_us) > gimbal_config.control.imu_max_age_us ||
       (uint32_t)(now_us - measurement.acc_timestamp_us) > gimbal_config.control.imu_max_age_us) {
        measurement.valid = false;
        gimbal_state.attitude_snapshot.valid = false;
    }
    gimbal_update_imu_health(imu_status, now_us);
    gimbal_state.blocker = gimbal_control_check(&gimbal_config.control, &measurement, gimbal_state.motor, now_us);

    if(gimbal_state.running) {
        gimbal_process_control(now_us);
    }
    if(gimbal_state.rearming) {
        gimbal_process_rearm(now_us);
    }
    if(gimbal_state.manual) {
        gimbal_process_manual();
    }
    else if(!gimbal_state.running && !gimbal_state.fault_latched && !gimbal_state.rearming) {
        gimbal_process_hold();
        gimbal_update_prepare_fault();
    }
    if(!was_calibrated && gimbal_state.attitude_snapshot.calibrated) {
        log_info("IMU calibrated");
    }
    bool rearm_ready = gimbal_state.rearming && rearm_motors_requested &&
                       gimbal_state.blocker == GIMBAL_CONTROL_OK && !imu_fault_active &&
                       gimbal_feedback_ready();
    bool prepare_ready = !gimbal_state.rearming && hold_captured && hold_submitted &&
                         gimbal_state.hold_pending_mask == 0u;
    gimbal_state.ready = !gimbal_state.manual && gimbal_state.blocker == GIMBAL_CONTROL_OK &&
                         !gimbal_state.fault && !gimbal_state.run_waiting &&
                         (gimbal_state.running || rearm_ready || prepare_ready);
    if(gimbal_state.ready && !gimbal_state.running) {
        gimbal_enter_run();
    }
    gimbal_process_console();
    gimbal_write_telemetry(now_us);
    uint32_t elapsed = delay_now_us() - begin_us;
    if(elapsed > gimbal_state.max_loop_us) {
        gimbal_state.max_loop_us = elapsed;
    }
}

bool gimbal_acknowledge_fault(void) {
    if(!gimbal_state.initialized || gimbal_state.running || !gimbal_state.fault_latched) {
        return false;
    }
    gimbal_state.fault = false;
    gimbal_state.fault_latched = false;
    gimbal_state.fault_reason = GIMBAL_FAULT_NONE;
    gimbal_state.fault_axis = GIMBAL_AXIS_COUNT;
    gimbal_state.run_waiting = false;
    gimbal_state.ready = false;
    run_pending_mask = 0u;
    gimbal_clear_targets();
    log_info("FAULT acknowledged");
    return true;
}

bool gimbal_toggle_target_mode(void) {
    if(!gimbal_state.initialized || gimbal_state.fault_latched) {
        return false;
    }
    if(gimbal_state.running || gimbal_state.rearming) {
        gimbal_enter_manual();
        return true;
    }
    if(!gimbal_state.manual) {
        return false;
    }
    gimbal_read_feedback();
    uint32_t now_us = delay_now_us();
    gimbal_state.now_ms = delay_now_ms();
    gimbal_state.blocker = gimbal_control_check(&gimbal_config.control, &measurement, gimbal_state.motor, now_us);
    gimbal_state.motors_released = gimbal_all_motors_released();
    gimbal_clear_targets();
    gimbal_set_fault(GIMBAL_FAULT_NONE, GIMBAL_AXIS_COUNT, false);
    gimbal_state.manual = false;
    gimbal_state.motors_released = false;
    gimbal_state.rearming = true;
    rearm_motors_requested = false;
    mode_started_ms = gimbal_state.now_ms;
    memset(rearm_retry_ms, 0, sizeof(rearm_retry_ms));
    last_hold_ms = mode_started_ms - gimbal_config.hold_period_ms;
    log_info("TARGET rearming; waiting inputs");
    return true;
}

const GimbalState* gimbal_get_state(void) {
    return &gimbal_state;
}

bool gimbal_is_ready(void) {
    return gimbal_state.initialized && gimbal_state.ready;
}

const char* gimbal_status_str(GimbalStatus status) {
    switch(status) {
#define X(name, str)           \
    case GIMBAL_STATUS_##name: \
        return str;
        GIMBAL_STATUS_TABLE
#undef X
        default:
            return "Unknown";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool gimbal_all_motors_released(void) {
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        if(!gimbal_config.ops->is_motor_released((GimbalAxis)i)) {
            return false;
        }
    }
    return true;
}

static void gimbal_enter_manual(void) {
    /* 先锁存禁能，清目标接口才不会发送当前位置保持 */
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        gimbal_config.ops->set_motor_released((GimbalAxis)i, true);
    }
    gimbal_control_stop(&gimbal_state.control);
    gimbal_state.running = false;
    gimbal_state.ready = false;
    gimbal_state.run_waiting = false;
    gimbal_state.manual = true;
    gimbal_state.rearming = false;
    rearm_motors_requested = false;
    gimbal_state.motors_released = false;
    gimbal_state.now_ms = delay_now_ms();
    mode_started_ms = gimbal_state.now_ms;
    run_pending_mask = 0u;
    run_wait_axis_mask = 0u;
    run_wait_logged = false;
    attitude.set_bias_learning(false);
    gimbal_clear_targets();
    memset(hold_position, 0, sizeof(hold_position));
    memset(hold_reject_ms, 0, sizeof(hold_reject_ms));
    log_info("TARGET manual; wait for green");
}

static void gimbal_process_manual(void) {
    gimbal_state.motors_released = gimbal_all_motors_released();
    if(gimbal_state.fault_latched) {
        return;
    }
    if(!gimbal_state.motors_released &&
       (uint32_t)(gimbal_state.now_ms - mode_started_ms) >= GIMBAL_RELEASE_TIMEOUT_MS) {
        gimbal_set_fault(GIMBAL_FAULT_RELEASE, GIMBAL_AXIS_COUNT, false);
    }
    else if(imu_fault_active) {
        gimbal_set_fault(GIMBAL_FAULT_IMU, GIMBAL_AXIS_COUNT, false);
    }
    else {
        gimbal_set_fault(GIMBAL_FAULT_NONE, GIMBAL_AXIS_COUNT, false);
    }
}

static void gimbal_process_rearm(uint32_t now_us) {
    /* 一次按键锁存恢复请求，不以普通总线延迟退出切换状态 */
    if(gimbal_rearm_rejected(now_us)) {
        /* 机械或几何条件不满足时停止接管，保持黄色等待而非退绿
           已开始使能的轴先全部失能，按键仍可随时取消 */
        if(rearm_motors_requested) {
            for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
                gimbal_config.ops->set_motor_released((GimbalAxis)i, true);
            }
            gimbal_clear_targets();
            rearm_motors_requested = false;
            log_warn("TARGET waiting safety=%s", gimbal_blocker_name(gimbal_check_rearm(now_us)));
        }
        return;
    }
    if(!rearm_motors_requested) {
        /* 失能确认后等待新鲜姿态和三轴反馈，再提交一次恢复请求 */
        gimbal_state.motors_released = gimbal_all_motors_released();
        if(!gimbal_state.motors_released || gimbal_state.blocker != GIMBAL_CONTROL_OK || imu_fault_active) {
            return;
        }
        for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
            gimbal_config.ops->set_motor_released((GimbalAxis)i, false);
            rearm_retry_ms[i] = gimbal_state.now_ms;
        }
        rearm_motors_requested = true;
        gimbal_state.motors_released = false;
        log_info("TARGET resume requested");
        return;
    }
    /* 驱动握手中断后若重新回到失能态，仅重新请求该轴
       不重置其它轴，也不清除已锁存的恢复请求 */
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        if(gimbal_config.ops->is_motor_released((GimbalAxis)i) &&
           (uint32_t)(gimbal_state.now_ms - rearm_retry_ms[i]) >= GIMBAL_REARM_RETRY_MS) {
            gimbal_config.ops->set_motor_released((GimbalAxis)i, false);
            rearm_retry_ms[i] = gimbal_state.now_ms;
            log_info("TARGET retry motor=%u", (unsigned)gimbal_config.motor_id[i]);
        }
    }
}

static bool gimbal_rearm_rejected(uint32_t now_us) {
    GimbalControlStatus status = gimbal_check_rearm(now_us);
    /* 握手期间允许短暂等数据，机械限位与奇异姿态立即退出 */
    return status != GIMBAL_CONTROL_OK && status != GIMBAL_CONTROL_IMU_INVALID &&
           status != GIMBAL_CONTROL_MOTOR_INVALID;
}

static GimbalControlStatus gimbal_check_rearm(uint32_t now_us) {
    GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT];
    memcpy(motor, gimbal_state.motor, sizeof(motor));
    /* DM 握手时尚未 ready，安全检查仍使用独立的原始位置与时效 */
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        motor[i].ready = isfinite(motor[i].position) && motor[i].age_ms != UINT32_MAX;
    }
    return gimbal_control_check(&gimbal_config.control, &measurement, motor, now_us);
}

static void gimbal_enter_run(void) {
    gimbal_read_feedback();
    uint32_t now_us = delay_now_us();
    gimbal_state.now_ms = delay_now_ms();
    gimbal_state.blocker = gimbal_control_start(&gimbal_state.control, &gimbal_config.control,
                                                &measurement, gimbal_state.motor, now_us);
    if(gimbal_state.blocker != GIMBAL_CONTROL_OK) {
        gimbal_state.ready = false;
        return;
    }
    attitude.set_bias_learning(false);
    gimbal_state.running = true;
    gimbal_state.rearming = false;
    gimbal_state.run_waiting = false;
    run_pending_mask = 0u;
    run_wait_axis_mask = 0u;
    run_wait_logged = false;
    last_run_good_ms = gimbal_state.now_ms;
    hold_captured = false;
    log_info("RUN started");
}

static bool gimbal_feedback_ready(void) {
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalMotorFeedback* motor = &gimbal_state.motor[i];
        const GimbalAxisConfig* axis = &gimbal_config.control.axis[i];
        if(!motor->ready || !isfinite(motor->position) || fabsf(motor->position) > axis->protocol_limit ||
           motor->age_ms > gimbal_config.prepare_feedback_max_age_ms) {
            return false;
        }
    }
    return true;
}

static bool gimbal_raw_imu_fresh(uint32_t now_us) {
    const ImuSample* sample = &gimbal_state.attitude_snapshot.sample;
    const uint32_t required = IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_ACC_VALID;
    return (sample->flags & required) == required &&
           (uint32_t)(now_us - sample->gyro_timestamp_us) <= gimbal_config.control.imu_max_age_us &&
           (uint32_t)(now_us - sample->acc_timestamp_us) <= gimbal_config.control.imu_max_age_us &&
           isfinite(sample->gyro.x) && isfinite(sample->gyro.y) && isfinite(sample->gyro.z) &&
           isfinite(sample->acc.x) && isfinite(sample->acc.y) && isfinite(sample->acc.z);
}

static void gimbal_update_imu_health(AttitudeStatus status, uint32_t now_us) {
    if(status == ATTITUDE_ERROR) {
        ++gimbal_state.imu_error_count;
        measurement.valid = false;
        gimbal_state.attitude_snapshot.valid = false;
    }
    if(status != ATTITUDE_ERROR && gimbal_raw_imu_fresh(now_us)) {
        imu_bad = false;
        imu_fault_active = false;
    }
    else {
        if(!imu_bad) {
            imu_bad = true;
            imu_bad_since_ms = gimbal_state.now_ms;
        }
        imu_fault_active = (uint32_t)(gimbal_state.now_ms - imu_bad_since_ms) >= GIMBAL_IMU_FAULT_MS;
    }
}

static const char* gimbal_fault_name(GimbalFault reason) {
    switch(reason) {
        case GIMBAL_FAULT_NONE:
            return "NONE";
        case GIMBAL_FAULT_IMU:
            return "IMU_DATA";
        case GIMBAL_FAULT_HOLD_COMMAND:
            return "PREPARE_COMMAND";
        case GIMBAL_FAULT_CONTROL:
            return "RUN_INPUT";
        case GIMBAL_FAULT_RUN_COMMAND:
            return "RUN_COMMAND";
        case GIMBAL_FAULT_RELEASE:
            return "MOTOR_RELEASE";
        default:
            return "UNKNOWN";
    }
}

static const char* gimbal_blocker_name(GimbalControlStatus status) {
    switch(status) {
        case GIMBAL_CONTROL_OK:
            return "NONE";
        case GIMBAL_CONTROL_INVALID_PARAM:
            return "CONFIG";
        case GIMBAL_CONTROL_UNCONFIRMED:
            return "COORDINATES";
        case GIMBAL_CONTROL_IMU_INVALID:
            return "IMU";
        case GIMBAL_CONTROL_MOTOR_INVALID:
            return "MOTOR_FEEDBACK";
        case GIMBAL_CONTROL_LIMIT:
            return "MECHANICAL_LIMIT";
        case GIMBAL_CONTROL_SINGULAR:
            return "GEOMETRY";
        case GIMBAL_CONTROL_STALLED:
            return "LOOP_STALL";
        default:
            return "UNKNOWN";
    }
}

static const char* gimbal_motor_diagnostic_name(GimbalMotorDiagnosticReason reason) {
    static const char* const names[] = { "OK", "NOT_READY", "STALE", "NONFINITE", "LIMIT", "COMMAND" };
    return (unsigned)reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "UNKNOWN";
}

static const char* gimbal_diagnostic_axes(uint8_t mask) {
    static const char* const names[] = { "UNKNOWN", "Y", "R", "MULTI(Y/R)", "P", "MULTI(Y/P)", "MULTI(R/P)", "MULTI(Y/R/P)" };
    return names[mask & 0x07u];
}

static const char* gimbal_stop_reason(void) {
    const GimbalStopDiagnostic* stopped = &gimbal_state.stop_diag;
    if(!stopped->valid) {
        return "NONE";
    }
    return stopped->blocker == GIMBAL_CONTROL_OK ? gimbal_fault_name(stopped->reason) : gimbal_blocker_name(stopped->blocker);
}

static void gimbal_capture_input(GimbalInputDiagnostic* diagnostic, GimbalFault reason, GimbalControlStatus status, GimbalAxis axis) {
    memset(diagnostic, 0, sizeof(*diagnostic));
    diagnostic->time_ms = gimbal_state.now_ms;
    diagnostic->since_good_ms = (uint32_t)(gimbal_state.now_ms - last_run_good_ms);
    diagnostic->blocker = status;
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalMotorFeedback* feedback = &gimbal_state.motor[i];
        const GimbalAxisConfig* config = &gimbal_config.control.axis[i];
        GimbalMotorDiagnostic* motor = &diagnostic->motor[i];
        motor->ready = feedback->ready;
        motor->age_ms = feedback->age_ms;
        motor->limit_ms = config->feedback_max_age_ms;
        bool finite = isfinite(feedback->position);
        bool stale = feedback->age_ms > config->feedback_max_age_ms;
        bool limited = false;
        if(feedback->ready && !stale && finite) {
            float joint = config->direction * config->ratio * (feedback->position - config->zero);
            limited = !isfinite(joint) || joint < config->min_angle || joint > config->max_angle ||
                      fabsf(feedback->position) > config->protocol_limit;
        }
        if(limited) {
            motor->reason = GIMBAL_MOTOR_DIAG_LIMIT;
        }
        else if(!feedback->ready) {
            motor->reason = GIMBAL_MOTOR_DIAG_NOT_READY;
        }
        else if(!finite) {
            motor->reason = GIMBAL_MOTOR_DIAG_NONFINITE;
        }
        else if(stale) {
            motor->reason = GIMBAL_MOTOR_DIAG_STALE;
        }
        else if((run_pending_mask & (1u << i)) != 0u) {
            motor->reason = GIMBAL_MOTOR_DIAG_COMMAND;
        }
        bool affected = status == GIMBAL_CONTROL_MOTOR_INVALID && (!feedback->ready || stale || !finite);
        affected = affected || (status == GIMBAL_CONTROL_LIMIT && limited);
        affected = affected || (reason == GIMBAL_FAULT_RUN_COMMAND && (run_pending_mask & (1u << i)) != 0u);
        if(affected || i == (unsigned)axis) {
            diagnostic->axis_mask |= (uint8_t)(1u << i);
        }
    }
}

static void gimbal_write_wait_event(GimbalFault reason, GimbalControlStatus status) {
    static const char* const names[] = { "Y", "R", "P" };
    gimbal_frame_begin("WAIT");
    gimbal_frame_append("[WAIT] reason=%s input=%s axes=%s elapsed=%lu/%lums\r\n",
                        gimbal_fault_name(reason), gimbal_blocker_name(status), gimbal_diagnostic_axes(run_first_bad.axis_mask),
                        (unsigned long)run_first_bad.since_good_ms, (unsigned long)gimbal_config.run_recovery_ms);
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalMotorDiagnostic* motor = &run_first_bad.motor[i];
        gimbal_frame_append("[%s#%u] reason=%s ready=%u age=%lums limit=%lums\r\n", names[i], (unsigned)gimbal_config.motor_id[i],
                            gimbal_motor_diagnostic_name(motor->reason), (unsigned)motor->ready,
                            (unsigned long)motor->age_ms, (unsigned long)motor->limit_ms);
    }
    gimbal_frame_send();
}

static void gimbal_write_stop_event(void) {
    static const char* const names[] = { "Y", "R", "P" };
    const GimbalStopDiagnostic* stopped = &gimbal_state.stop_diag;
    gimbal_frame_begin("FAULT");
    gimbal_frame_append("[STOP] reason=%s stop_reason=%s axes=%s input=%s elapsed=%lu/%lums recovery=PA15_CONFIRM\r\n",
                        gimbal_fault_name(stopped->reason), gimbal_stop_reason(), gimbal_diagnostic_axes(stopped->axis_mask),
                        gimbal_blocker_name(stopped->stopped.blocker), (unsigned long)stopped->stopped.since_good_ms,
                        (unsigned long)gimbal_config.run_recovery_ms);
    gimbal_frame_append("[FIRST_BAD] t=%lums input=%s axes=%s; stopped_t=%lums\r\n",
                        (unsigned long)stopped->first_bad.time_ms, gimbal_blocker_name(stopped->first_bad.blocker),
                        gimbal_diagnostic_axes(stopped->first_bad.axis_mask), (unsigned long)stopped->stopped.time_ms);
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalMotorDiagnostic* first = &stopped->first_bad.motor[i];
        const GimbalMotorDiagnostic* last = &stopped->stopped.motor[i];
        gimbal_frame_append("[%s#%u] first=%s ready=%u age=%lu/%lums | stop=%s ready=%u age=%lu/%lums\r\n",
                            names[i], (unsigned)gimbal_config.motor_id[i], gimbal_motor_diagnostic_name(first->reason),
                            (unsigned)first->ready, (unsigned long)first->age_ms, (unsigned long)first->limit_ms,
                            gimbal_motor_diagnostic_name(last->reason), (unsigned)last->ready,
                            (unsigned long)last->age_ms, (unsigned long)last->limit_ms);
    }
    gimbal_frame_send();
}

static void gimbal_set_fault(GimbalFault reason, GimbalAxis axis, bool latched) {
    bool changed = gimbal_state.fault_reason != reason || gimbal_state.fault_latched != latched ||
                   gimbal_state.fault_axis != axis;
    gimbal_state.fault_reason = reason;
    gimbal_state.fault_axis = axis;
    gimbal_state.fault_latched = latched;
    gimbal_state.fault = reason != GIMBAL_FAULT_NONE;
    if(changed && !latched) {
        if(reason == GIMBAL_FAULT_NONE) {
            log_info("FAULT cleared");
        }
        else {
            if(axis < GIMBAL_AXIS_COUNT) {
                log_warn("FAULT reason=%s id=%u recovery=AUTO", gimbal_fault_name(reason), (unsigned)gimbal_config.motor_id[axis]);
            }
            else {
                log_warn("FAULT reason=%s id=UNKNOWN recovery=AUTO", gimbal_fault_name(reason));
            }
        }
    }
}

static void gimbal_update_prepare_fault(void) {
    if(imu_fault_active) {
        gimbal_set_fault(GIMBAL_FAULT_IMU, GIMBAL_AXIS_COUNT, false);
        return;
    }
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        if((gimbal_state.hold_pending_mask & (1u << i)) != 0u &&
           (uint32_t)(gimbal_state.now_ms - hold_reject_ms[i]) >= gimbal_config.hold_timeout_ms) {
            gimbal_set_fault(GIMBAL_FAULT_HOLD_COMMAND, (GimbalAxis)i, false);
            return;
        }
    }
    gimbal_set_fault(GIMBAL_FAULT_NONE, GIMBAL_AXIS_COUNT, false);
}

static void gimbal_safety_stop(GimbalFault reason, GimbalAxis axis, GimbalControlStatus status) {
    GimbalStopDiagnostic* stopped = &gimbal_state.stop_diag;
    stopped->valid = true;
    stopped->reason = reason;
    stopped->blocker = status;
    gimbal_capture_input(&stopped->stopped, reason, gimbal_state.blocker, axis);
    stopped->first_bad = gimbal_state.run_waiting ? run_first_bad : stopped->stopped;
    stopped->axis_mask = stopped->stopped.axis_mask;
    if(stopped->axis_mask == 0u && gimbal_state.run_waiting) {
        stopped->axis_mask = run_wait_axis_mask;
    }
    gimbal_control_stop(&gimbal_state.control);
    gimbal_state.running = false;
    gimbal_state.run_waiting = false;
    run_pending_mask = 0u;
    attitude.set_bias_learning(false);
    gimbal_clear_targets();
    gimbal_set_fault(reason, axis, true);
    gimbal_write_stop_event();
}

static bool gimbal_wait_for_run(GimbalFault reason, GimbalAxis axis, GimbalControlStatus status) {
    GimbalInputDiagnostic current;
    gimbal_capture_input(&current, reason, status, axis);
    if(!gimbal_state.run_waiting) {
        gimbal_state.run_waiting = true;
        ++gimbal_state.run_wait_count;
        run_first_bad = current;
        /* 正常的低速反馈也可能短暂等待；整次 RUN 只额外打印首个 WAIT */
        if(!run_wait_logged) {
            run_wait_logged = true;
            gimbal_write_wait_event(reason, status);
        }
    }
    run_wait_reason = reason;
    run_wait_axis = axis;
    run_wait_status = status;
    run_wait_axis_mask = current.axis_mask;
    if((uint32_t)(gimbal_state.now_ms - last_run_good_ms) >= gimbal_config.run_recovery_ms) {
        gimbal_safety_stop(reason, axis, status);
        return false;
    }
    return true;
}

static bool gimbal_feedback_stationary(void) {
    if(!gimbal_feedback_ready()) {
        return false;
    }
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalMotorFeedback* motor = &gimbal_state.motor[i];
        float ratio = gimbal_config.control.axis[i].ratio;
        if(!motor->velocity_valid || !isfinite(motor->velocity) || fabsf(motor->velocity * ratio) > 0.05f ||
           (hold_captured && fabsf((motor->position - hold_position[i]) * ratio) > 0.02f)) {
            return false;
        }
    }
    return true;
}

static void gimbal_read_feedback(void) {
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        GimbalMotorFeedback* motor = &gimbal_state.motor[i];
        memset(motor, 0, sizeof(*motor));
        motor->age_ms = UINT32_MAX;
        if(!gimbal_config.ops->read_motor((GimbalAxis)i, motor)) {
            motor->ready = false;
        }
    }
}

static void gimbal_update_measurement(void) {
    const AttitudeSnapshot* snapshot = &gimbal_state.attitude_snapshot;
    measurement = (GimbalMeasurement){
        .roll = snapshot->angle.roll,
        .pitch = snapshot->angle.pitch,
        .yaw = snapshot->angle.yaw,
        .gyro = { snapshot->gyro_corrected.x, snapshot->gyro_corrected.y, snapshot->gyro_corrected.z },
        .timestamp_us = snapshot->sample.gyro_timestamp_us,
        .acc_timestamp_us = snapshot->sample.acc_timestamp_us,
        .sequence = snapshot->sequence,
        .valid = snapshot->valid && snapshot->calibrated,
    };
}

static void gimbal_clear_targets(void) {
    gimbal_config.ops->clear_motor_target(GIMBAL_AXIS_PITCH);
    gimbal_config.ops->clear_motor_target(GIMBAL_AXIS_ROLL);
    gimbal_config.ops->clear_motor_target(GIMBAL_AXIS_YAW);
    hold_captured = hold_submitted = false;
    gimbal_state.hold_pending_mask = 0u;
}

static void gimbal_capture_hold(void) {
    hold_submitted = false;
    hold_captured = gimbal_feedback_ready();
    if(!hold_captured) {
        return;
    }
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        hold_position[i] = gimbal_state.motor[i].position;
    }
}

static void gimbal_process_hold(void) {
    if(!gimbal_feedback_ready()) {
        if(hold_captured) {
            gimbal_clear_targets();
        }
        return;
    }
    if(!hold_captured) {
        gimbal_capture_hold();
    }
    if((uint32_t)(gimbal_state.now_ms - last_hold_ms) < gimbal_config.hold_period_ms) {
        return;
    }
    last_hold_ms = gimbal_state.now_ms;
    const GimbalAxis order[] = { GIMBAL_AXIS_PITCH, GIMBAL_AXIS_ROLL, GIMBAL_AXIS_YAW };
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        GimbalAxis axis = order[i];
        if(!gimbal_config.ops->set_motor_target(axis, hold_position[axis],
                                                gimbal_config.hold_speed, gimbal_config.hold_timeout_ms)) {
            if((gimbal_state.hold_pending_mask & (1u << axis)) == 0u) {
                hold_reject_ms[axis] = gimbal_state.now_ms;
            }
            gimbal_state.hold_pending_mask |= (uint8_t)(1u << axis);
            ++gimbal_state.command_retry_count[axis];
        }
        else {
            gimbal_state.hold_pending_mask &= (uint8_t)~(1u << axis);
        }
    }
    hold_submitted = true;
    /* 拒绝后保留目标，下一周期重试 */
}

static void gimbal_process_control(uint32_t now_us) {
    GimbalControlStatus status = gimbal_state.blocker;
    if(status == GIMBAL_CONTROL_IMU_INVALID || status == GIMBAL_CONTROL_MOTOR_INVALID) {
        (void)gimbal_wait_for_run(GIMBAL_FAULT_CONTROL, GIMBAL_AXIS_COUNT, status);
        return; /* 等待新数据时不调用控制器，不使用过期输入生成目标 */
    }
    if(status != GIMBAL_CONTROL_OK) {
        gimbal_safety_stop(GIMBAL_FAULT_CONTROL, GIMBAL_AXIS_COUNT, status);
        return;
    }
    if(gimbal_state.run_waiting &&
       (uint32_t)(gimbal_state.now_ms - last_run_good_ms) > gimbal_config.run_recovery_ms) {
        gimbal_safety_stop(run_wait_reason, run_wait_axis, run_wait_status);
        return;
    }
    /* 截止时刻的新鲜输入可尝试一次完整恢复；迟到不再提交，坏输入仍在 >= 上限时停 */
    GimbalControl previous = gimbal_state.control;
    GimbalMotorCommand command[GIMBAL_AXIS_COUNT];
    status = gimbal_control_update(&gimbal_state.control, &gimbal_config.control,
                                   &measurement, gimbal_state.motor, now_us, command);
    if(status != GIMBAL_CONTROL_OK) {
        gimbal_safety_stop(GIMBAL_FAULT_CONTROL, GIMBAL_AXIS_COUNT, status);
        return;
    }
    const GimbalAxis order[] = { GIMBAL_AXIS_YAW, GIMBAL_AXIS_PITCH, GIMBAL_AXIS_ROLL };
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        GimbalAxis axis = order[i];
        if(!command[axis].updated) {
            continue;
        }
        if(gimbal_config.ops->set_motor_target(axis, command[axis].position, command[axis].speed,
                                               command[axis].timeout_ms)) {
            run_pending_mask &= (uint8_t)~(1u << axis);
        }
        else {
            run_pending_mask |= (uint8_t)(1u << axis);
            ++gimbal_state.command_retry_count[axis];
            /* 未接受的目标不计为已执行，但保留本次 last_us/sequence 以限制重试频率 */
            gimbal_state.control.axis[axis].command = previous.axis[axis].command;
            gimbal_state.control.axis[axis].filtered_goal = previous.axis[axis].filtered_goal;
        }
    }
    gimbal_state.now_ms = delay_now_ms();
    if(run_pending_mask != 0u) {
        GimbalAxis axis = GIMBAL_AXIS_YAW;
        while((run_pending_mask & (1u << axis)) == 0u) {
            axis = (GimbalAxis)(axis + 1);
        }
        (void)gimbal_wait_for_run(GIMBAL_FAULT_RUN_COMMAND, axis, GIMBAL_CONTROL_OK);
    }
    else if(gimbal_state.run_waiting &&
            (uint32_t)(gimbal_state.now_ms - last_run_good_ms) > gimbal_config.run_recovery_ms) {
        gimbal_safety_stop(run_wait_reason, run_wait_axis, run_wait_status);
    }
    else {
        gimbal_state.run_waiting = false;
        last_run_good_ms = gimbal_state.now_ms;
    }
}

static void gimbal_process_console(void) {
    if(gimbal_config.ops->take_console_rx_reset()) {
        console_line_length = 0u;
        console_line_discard = false;
        console_cr_pending = false;
        log_warn("CONSOLE RX reset");
    }
    uint8_t byte;
    for(unsigned count = 0; count < 32u && gimbal_config.ops->read_console_byte(&byte); ++count) {
        if(byte == '\n') {
            if(console_line_length != 0u && !console_line_discard) {
                console_line[console_line_length] = '\0';
                gimbal_handle_console_line();
            }
            console_line_length = 0u;
            console_line_discard = false;
            console_cr_pending = false;
        }
        else if(byte == '\r') {
            if(console_cr_pending) {
                console_line_discard = true;
            }
            console_cr_pending = true;
        }
        else if(!console_line_discard) {
            if(console_cr_pending || byte < 32u || byte > 126u || console_line_length >= sizeof(console_line) - 1u) {
                console_line_discard = true;
                console_line_length = 0u;
            }
            else {
                console_line[console_line_length++] = (char)byte;
            }
        }
    }
}

static void gimbal_handle_console_line(void) {
    if(strcmp(console_line, "status") == 0) {
        gimbal_write_status();
    }
    else if(strcmp(console_line, "log on") == 0 || strcmp(console_line, "log off") == 0) {
        telemetry_enabled = strcmp(console_line, "log on") == 0;
        raw_enabled = false;
        last_status_ms = gimbal_state.now_ms;
        log_info("LOG %s", telemetry_enabled ? "summary=1Hz" : "OFF");
    }
    else if(strcmp(console_line, "raw on") == 0 || strcmp(console_line, "raw off") == 0) {
        raw_enabled = strcmp(console_line, "raw on") == 0;
        telemetry_enabled = true;
        raw_started_ms = last_log_ms = gimbal_state.now_ms;
        if(raw_enabled) {
            log_info("RAW 10Hz 10s");
            const char* header =
                "imu,now_us,gyro_us,acc_us,seq,valid,calibrated,stationary,T_C,ax,ay,az,gx,gy,gz,gcx,gcy,gcz,roll,pitch,yaw,bx,by,bz\r\n"
                "motor,now_us,id,ready,velocity_valid,age_ms,encoder_rad,velocity_rad_s,target_rad,joint_rad,target_accepted\r\n";
            gimbal_write_line(header, (int)strlen(header), strlen(header) + 1u);
        }
        else {
            log_info("LOG summary=1Hz");
        }
    }
    else if(strcmp(console_line, "help") == 0) {
        log_info("COMMANDS status | log on/off | raw on/off | help");
        log_info("RUN auto; PA15 acknowledges fault");
    }
    else {
        log_warn("Unknown command: %s", console_line);
    }
}

static void gimbal_write_telemetry(uint32_t now_us) {
    if(raw_enabled && (uint32_t)(gimbal_state.now_ms - raw_started_ms) >= GIMBAL_RAW_DURATION_MS) {
        raw_enabled = false;
        log_info("RAW off; LOG 1Hz");
    }
    if(!telemetry_enabled) {
        return;
    }
    if(!raw_enabled && (uint32_t)(gimbal_state.now_ms - last_status_ms) >= gimbal_config.status_period_ms) {
        last_status_ms = gimbal_state.now_ms;
        gimbal_write_status();
    }
    if(!raw_enabled || (uint32_t)(gimbal_state.now_ms - last_log_ms) < gimbal_config.raw_log_period_ms) {
        return;
    }
    last_log_ms = gimbal_state.now_ms;
    gimbal_write_raw(now_us);
}

static void gimbal_write_raw(uint32_t now_us) {
    const AttitudeSnapshot* a = &gimbal_state.attitude_snapshot;
    gimbal_frame_begin("RAW");
    gimbal_frame_append(
        "imu,%lu,%lu,%lu,%lu,%u,%u,%u,%.3f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f\r\n",
        (unsigned long)now_us, (unsigned long)a->sample.gyro_timestamp_us,
        (unsigned long)a->sample.acc_timestamp_us, (unsigned long)a->sequence,
        (unsigned)a->valid, (unsigned)a->calibrated, (unsigned)a->stationary,
        (double)a->sample.temperature, (double)a->sample.acc.x, (double)a->sample.acc.y, (double)a->sample.acc.z,
        (double)a->sample.gyro.x, (double)a->sample.gyro.y, (double)a->sample.gyro.z,
        (double)a->gyro_corrected.x, (double)a->gyro_corrected.y, (double)a->gyro_corrected.z,
        (double)a->angle.roll, (double)a->angle.pitch, (double)a->angle.yaw,
        (double)a->gyro_bias_effective.x, (double)a->gyro_bias_effective.y, (double)a->gyro_bias_effective.z);
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalMotorFeedback* motor = &gimbal_state.motor[i];
        const GimbalAxisConfig* config = &gimbal_config.control.axis[i];
        float joint = gimbal_config.control.coordinates_confirmed ? config->direction * config->ratio * (motor->position - config->zero) : NAN;
        float target = gimbal_state.running ? gimbal_state.control.axis[i].command : (hold_captured ? hold_position[i] : NAN);
        bool accepted = gimbal_state.running ? (run_pending_mask & (1u << i)) == 0u : hold_captured && (gimbal_state.hold_pending_mask & (1u << i)) == 0u;
        gimbal_frame_append("motor,%lu,%u,%u,%u,%lu,%.6f,%.6f,%.6f,%.6f,%u\r\n",
                            (unsigned long)now_us, (unsigned)gimbal_config.motor_id[i], (unsigned)motor->ready,
                            (unsigned)motor->velocity_valid, (unsigned long)motor->age_ms,
                            (double)motor->position, (double)motor->velocity, (double)target, (double)joint,
                            (unsigned)accepted);
    }
    gimbal_frame_send();
}

static void gimbal_write_status(void) {
    const AttitudeSnapshot* snapshot = &gimbal_state.attitude_snapshot;
    const AttitudeState* imu_state = attitude.get_state();
    const char* state = gimbal_state.fault_latched ? "SAFE_STOP"
                      : gimbal_state.manual ? (gimbal_state.motors_released ? "MANUAL" : "MANUAL_WAIT")
                      : gimbal_state.rearming ? "REARMING"
                      : gimbal_state.running ? (gimbal_state.run_waiting ? "RUN_WAIT" : "RUN") : "STARTING";
    const char* recovery = !gimbal_state.fault ? "NONE" : (gimbal_state.fault_latched ? "PA15_CONFIRM" : "AUTO");
    gimbal_frame_begin("SUMMARY");
    gimbal_frame_append("[SYS] state=%s run_ready=%s | run_block=%s input=%s coords=%s | fault=%s stop_reason=%s recovery=%s\r\n",
                        state, gimbal_state.ready ? "YES" : "NO",
                        gimbal_state.fault_latched ? "WAIT_ACK" : gimbal_blocker_name(gimbal_state.blocker),
                        gimbal_blocker_name(gimbal_state.blocker),
                        gimbal_config.control.coordinates_confirmed ? "OK" : "PENDING",
                        gimbal_fault_name(gimbal_state.fault_reason), gimbal_state.fault_latched ? gimbal_stop_reason() : "NONE", recovery);
    char pose[96];
    if(snapshot->valid) {
        (void)snprintf(pose, sizeof(pose), "RPY_deg=%.2f/%.2f/%.2f",
                       (double)(snapshot->angle.roll * GIMBAL_RAD_TO_DEG),
                       (double)(snapshot->angle.pitch * GIMBAL_RAD_TO_DEG),
                       (double)(snapshot->angle.yaw * GIMBAL_RAD_TO_DEG));
    }
    else {
        (void)snprintf(pose, sizeof(pose), "RPY_deg=unavailable");
    }
    const char* imu_phase = snapshot->valid ? "READY" : (snapshot->calibrated ? "WAIT_DATA" : "CALIBRATING");
    gimbal_frame_append("[IMU] %s samples=%u/%u | still=%.0fms learn=%s | T=%.2fC %s\r\n",
                        imu_phase, (unsigned)imu_state->calibration_samples, (unsigned)imu_state->calibration_target,
                        (double)(imu_state->stationary_candidate_s * 1000.0f),
                        imu_state->bias_learning_allowed ? "ON" : "OFF", (double)snapshot->sample.temperature, pose);
    char motor[3][112];
    for(unsigned i = 0u; i < GIMBAL_AXIS_COUNT; ++i) {
        gimbal_format_motor(motor[i], sizeof(motor[i]), i);
    }
    gimbal_frame_append("[MOTOR] %s | %s | %s | hold=%s pending=%s\r\n",
                        motor[0], motor[1], motor[2], hold_captured ? "ON" : "OFF",
                        gimbal_pending_axes(gimbal_state.running ? run_pending_mask : gimbal_state.hold_pending_mask));
    gimbal_frame_append(
        "[DIAG] static_resets=%lu last_static_event=%s | retry_Y/R/P=%lu/%lu/%lu imu_err=%lu run_wait=%lu log_drop=%lu",
        (unsigned long)imu_state->static_reset_count, attitude.reset_reason_str(imu_state->last_reset_reason),
        (unsigned long)gimbal_state.command_retry_count[0],
        (unsigned long)gimbal_state.command_retry_count[1],
        (unsigned long)gimbal_state.command_retry_count[2],
        (unsigned long)gimbal_state.imu_error_count, (unsigned long)gimbal_state.run_wait_count,
        (unsigned long)gimbal_state.telemetry_dropped);
    if(gimbal_state.stop_diag.valid) {
        const GimbalStopDiagnostic* stopped = &gimbal_state.stop_diag;
        gimbal_frame_append(" | last_stop=%s@%lu axes=%s first_age=%lu/%lu/%lu stop_age=%lu/%lu/%lu",
                            gimbal_stop_reason(), (unsigned long)stopped->stopped.time_ms, gimbal_diagnostic_axes(stopped->axis_mask),
                            (unsigned long)stopped->first_bad.motor[0].age_ms,
                            (unsigned long)stopped->first_bad.motor[1].age_ms,
                            (unsigned long)stopped->first_bad.motor[2].age_ms,
                            (unsigned long)stopped->stopped.motor[0].age_ms,
                            (unsigned long)stopped->stopped.motor[1].age_ms,
                            (unsigned long)stopped->stopped.motor[2].age_ms);
    }
    char led[151] = "unavailable";
    if(gimbal_config.ops->format_led_diagnostics != NULL) {
        gimbal_config.ops->format_led_diagnostics(led, sizeof(led));
        led[sizeof(led) - 1u] = '\0';
    }
    gimbal_frame_append("\r\n[LED] %s\r\n", led);
    gimbal_frame_send();
    last_status_ms = gimbal_state.now_ms;
}

static void gimbal_format_motor(char* text, size_t capacity, unsigned axis) {
    static const char* const names[] = { "Y", "R", "P" };
    const GimbalMotorFeedback* motor = &gimbal_state.motor[axis];
    unsigned id = gimbal_config.motor_id[axis];
    if(motor->age_ms == UINT32_MAX) {
        (void)snprintf(text, capacity, "%s#%u=WAIT age=NA", names[axis], id);
    }
    else if(!motor->ready || !isfinite(motor->position)) {
        (void)snprintf(text, capacity, "%s#%u=WAIT age=%lums", names[axis], id, (unsigned long)motor->age_ms);
    }
    else {
        (void)snprintf(text, capacity, "%s#%u pos=%.5frad(%.2fdeg) age=%lums", names[axis], id,
                       (double)motor->position, (double)(motor->position * GIMBAL_RAD_TO_DEG),
                       (unsigned long)motor->age_ms);
    }
}

static const char* gimbal_pending_axes(uint8_t mask) {
    static const char* const names[] = { "NONE", "Y", "R", "Y/R", "P", "Y/P", "R/P", "Y/R/P" };
    return names[mask & 0x07u];
}

static void gimbal_frame_begin(const char* kind) {
    log_frame_length = 0u;
    log_frame_valid = true;
    ++log_frame_sequence;
    gimbal_frame_append("\r\n================ FRAME %06lu %s t=%lums ================\r\n",
                        (unsigned long)log_frame_sequence, kind, (unsigned long)gimbal_state.now_ms);
}

static void gimbal_frame_append(const char* format, ...) {
    if(!log_frame_valid) {
        return;
    }
    size_t remaining = sizeof(log_frame) - log_frame_length;
    va_list args;
    va_start(args, format);
    int count = vsnprintf(log_frame + log_frame_length, remaining, format, args);
    va_end(args);
    if(count < 0 || (size_t)count >= remaining) {
        log_frame_valid = false;
        return;
    }
    log_frame_length += (size_t)count;
}

static void gimbal_frame_send(void) {
    gimbal_frame_append("================ END FRAME %06lu ================\r\n",
                        (unsigned long)log_frame_sequence);
    /* 整包一次提交格式溢出或队列不足时只计一次丢弃，绝不发送半组状态 */
    if(!log_frame_valid || !gimbal_config.ops->write_console(log_frame, (uint32_t)log_frame_length)) {
        ++gimbal_state.telemetry_dropped;
    }
}

static void gimbal_write_line(const char* line, int length, size_t capacity) {
    if(length <= 0 || (size_t)length >= capacity || !gimbal_config.ops->write_console(line, (uint32_t)length)) {
        ++gimbal_state.telemetry_dropped;
    }
}
