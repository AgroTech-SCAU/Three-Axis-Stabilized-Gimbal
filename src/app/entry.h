#ifndef _app_entry_h_
#define _app_entry_h_

/**
 * @file entry.h
 * @brief CubeMX `main.c` 的应用入口替代层
 */

#include "assemble/assemble.h"
#include "delay.h"
#include "imu/imu.h"
#include "imu/bmi088.h"
#include "gimbal_control.h"
#include "stm32_hal_dwt.h"
#include "log.h"
#include "stm32_hal_can.h"
#include "vision_comms.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 系统初始化完成标记
 */
static bool init_ok = false;

#define ROBOT_FW_TAG "robot-can-v5-20260914"
#define ENTRY_VISION_ENABLED 1
#define ENTRY_VISION_LOG_PERIOD_MS 5000u
#define ENTRY_RAD_TO_DEG 57.29577951308232f
#define ENTRY_GIMBAL_ROLL_MOTOR_ID 8u
#define ENTRY_GIMBAL_PITCH_MOTOR_ID 2u
#define ENTRY_VISION_RESULT_TIMEOUT_MS 200u

static ms_t s_entry_vision_log_timer = 0u;
static ms_t s_entry_motor_bus_wait_log_timer = 0u;
static bool s_entry_motor_bus_ht_ready_logged = false;
static ms_t s_entry_can1_recovery_error_log_timer = 0u;
static ms_t s_entry_can1_recovery_progress_log_timer = 0u;
static bool s_entry_can1_recovery_active = false;
static Bmi088Snapshot s_entry_imu_snapshot;
static GimbalControl s_entry_gimbal_control;
static bool s_entry_log_imu_seen = false;
static uint32_t s_entry_log_imu_ms = 0u;
static bool s_entry_startup_holds_sent;
static uint32_t s_entry_startup_tx_ms;
#define ENTRY_STARTUP_HOLD_PERIOD_MS 10u
#define ENTRY_STARTUP_HOLD_TIMEOUT_MS 100u
#define ENTRY_STARTUP_HOLD_SPEED_RAD_S 0.3f

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static inline void entry_process_vision_level(void);
static inline bool entry_motor_bus_ht_ready(void);
static inline void entry_process_imu(void);
static inline void entry_process_yaw_control(void);
static inline void entry_process_tilt_control(void);
static inline bool entry_process_startup(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 系统初始化入口
 * @details 按照启动依赖顺序初始化 IMU、电机和可选视觉通信
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK)
        return;

    if(assemble_log() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT log ready");
    log_info("FW tag=%s vision=%s motor_boot=HT_DISCOVERY_THEN_DM can_api=0x%08lx",
             ROBOT_FW_TAG,
             ENTRY_VISION_ENABLED ? "on" : "off",
             (unsigned long)stm32_hal_can_api_version());

    if(assemble_imu() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT imu ready");
    delay_ms(100u);

    if(assemble_dm_motor() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT dm motor ready");
    delay_ms(100u);

    if(assemble_ht_motor() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT ht motor ready");
    delay_ms(100u);

#if ENTRY_VISION_ENABLED
    if(assemble_vision_comms() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT vision comms ready (external source optional)");
    delay_ms(100u);
#endif

    log_info("System initialized successfully");
    delay_ms(500u);

    init_ok = true;
}

/* Main-loop scheduling: IMU, CAN recovery, motor control, optional vision. */
static inline void entry_loop(void) {
    if(!init_ok) {
        return;
    }
    entry_process_imu();

    {
        bool can1_recovered = false;
        BspCanStatus can1_status = can_service_recovery(&hfdcan1, &can1_recovered);
        BspCanHealth health;
        bool have_health = can_get_health(&hfdcan1, &health) == STM32_HAL_CAN_OK;

        if(have_health && health.recovery_in_progress && !s_entry_can1_recovery_active) {
            s_entry_can1_recovery_active = true;
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

        if(have_health && health.recovery_in_progress && s_entry_can1_recovery_active &&
           delay_nb_ms(&s_entry_can1_recovery_progress_log_timer, 2000u)) {
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
            s_entry_can1_recovery_active = false;
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
                delay_nb_ms(&s_entry_can1_recovery_error_log_timer, 1000u)) {
            log_error("CAN1 recovery service: %s", can_error_code_to_str(can1_status));
        }
    }

    /*
     * Bring the shared motor bus up deterministically:
     * 1) HT sends only slow discovery probes until both motors answer.
     * 2) A real HT reply proves FDCAN1 + transceiver + physical bus are alive.
     * 3) Only then may the DM mode-switch handshake start.
     * This avoids a power-up race where several command producers hammer an
     * unready bus and accumulate TEC until Error Passive / Bus-Off.
     */
    assemble_ht_motor_process();
    if(entry_motor_bus_ht_ready()) {
        if(!s_entry_motor_bus_ht_ready_logged) {
            s_entry_motor_bus_ht_ready_logged = true;
            log_info("MOTOR BUS verified by HT feedback; DM handshake released");
        }
        assemble_dm_motor_process();
    }
    else if(delay_nb_ms(&s_entry_motor_bus_wait_log_timer, 1000u)) {
        log_info("MOTOR BUS discovery: waiting for HT id=2/8 feedback before DM handshake");
    }

    if(entry_process_startup()) {
        entry_process_yaw_control();
        entry_process_tilt_control();
    }

#if ENTRY_VISION_ENABLED
    vision_comms_process();
    entry_process_vision_level();
#endif

}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static inline void entry_process_imu(void) {
    /* Drain a bounded backlog; keep each sample's DRDY timestamp for fusion. */
    for(uint32_t i = 0U; i < 16U; ++i) {
        ImuStatus status = imu_update();
        if(status == IMU_STATUS_NOT_READY) break;
        if(status != IMU_STATUS_OK) {
            s_entry_log_imu_seen = false;
            gimbal_control_calibrate(&s_entry_gimbal_control, NULL, dwt_get_us());
            return;
        }
        if(bmi088_get_snapshot(&s_entry_imu_snapshot)) {
            uint32_t age_us = dwt_get_us() - s_entry_imu_snapshot.sample.gyro_timestamp_us;
            s_entry_log_imu_seen = age_us <= 10000U;
            if(s_entry_log_imu_seen) s_entry_log_imu_ms = HAL_GetTick();
            gimbal_control_calibrate(&s_entry_gimbal_control, &s_entry_imu_snapshot, dwt_get_us());
        }
    }
    if((uint32_t)(HAL_GetTick() - s_entry_log_imu_ms) > 10U) {
        s_entry_log_imu_seen = false;
        gimbal_control_calibrate(&s_entry_gimbal_control, NULL, dwt_get_us());
    }
}

static inline bool entry_process_startup(void) {
    if(s_entry_gimbal_control.startup.running) return true;
    BusMotorFeedback f={0};
    GimbalYawFeedback p={0}, r={0}, y={0};
    p.ready=assemble_ht_motor_get_feedback_snapshot_age(ENTRY_GIMBAL_PITCH_MOTOR_ID,&f,&p.age_ms);
    p.position=f.position; p.velocity=f.velocity;
    p.ready=p.ready && (f.valid & BUS_MOTOR_FEEDBACK_POSITION);
    p.velocity_valid=(f.valid & BUS_MOTOR_FEEDBACK_VELOCITY)!=0u;
    f=(BusMotorFeedback){0};
    r.ready=assemble_ht_motor_get_feedback_snapshot_age(ENTRY_GIMBAL_ROLL_MOTOR_ID,&f,&r.age_ms);
    r.position=f.position; r.velocity=f.velocity;
    r.ready=r.ready && (f.valid & BUS_MOTOR_FEEDBACK_POSITION);
    r.velocity_valid=(f.valid & BUS_MOTOR_FEEDBACK_VELOCITY)!=0u;
    f=(BusMotorFeedback){0};
    y.ready=assemble_dm_motor_get_feedback_snapshot_age(&f,&y.age_ms) && assemble_dm_motor_is_ready();
    y.position=f.position; y.velocity=f.velocity;
    y.ready=y.ready && (f.valid & BUS_MOTOR_FEEDBACK_POSITION);
    y.velocity_valid=(f.valid & BUS_MOTOR_FEEDBACK_VELOCITY)!=0u;
    if(gimbal_control_startup_update(&s_entry_gimbal_control,
        s_entry_log_imu_seen?&s_entry_imu_snapshot:NULL,&p,&r,&y,dwt_get_us(),s_entry_startup_holds_sent))
        return true;
    GimbalStartup *a=&s_entry_gimbal_control.startup;
    if(!a->captured) {
        s_entry_startup_holds_sent=false;
        assemble_ht_motor_cancel_control(ENTRY_GIMBAL_PITCH_MOTOR_ID);
        assemble_ht_motor_cancel_control(ENTRY_GIMBAL_ROLL_MOTOR_ID);
        assemble_dm_motor_clear_control_target();
        return false;
    }
    uint32_t now=HAL_GetTick();
    if((uint32_t)(now-s_entry_startup_tx_ms)>=ENTRY_STARTUP_HOLD_PERIOD_MS) {
        s_entry_startup_tx_ms=now;
        bool hp=assemble_ht_motor_set_control_target(ENTRY_GIMBAL_PITCH_MOTOR_ID,
            a->pitch_position,ENTRY_STARTUP_HOLD_SPEED_RAD_S,ENTRY_STARTUP_HOLD_TIMEOUT_MS);
        bool hr=assemble_ht_motor_set_control_target(ENTRY_GIMBAL_ROLL_MOTOR_ID,
            a->roll_position,ENTRY_STARTUP_HOLD_SPEED_RAD_S,ENTRY_STARTUP_HOLD_TIMEOUT_MS);
        bool hy=assemble_dm_motor_set_target_position_guarded(a->yaw_position,
            ENTRY_STARTUP_HOLD_SPEED_RAD_S,ENTRY_STARTUP_HOLD_TIMEOUT_MS);
        s_entry_startup_holds_sent=hp && hr && hy;
    }
    return false;
}

static inline void entry_process_tilt_control(void) {
    BusMotorFeedback f={0};
    GimbalYawFeedback p={0}, r={0}, y={0};
    GimbalYawCommand po,ro; bool pc=false,rc=false;
    p.ready=assemble_ht_motor_get_feedback_snapshot_age(ENTRY_GIMBAL_PITCH_MOTOR_ID,&f,&p.age_ms);
    p.position=f.position; p.ready=p.ready && (f.valid & BUS_MOTOR_FEEDBACK_POSITION);
    r.ready=assemble_ht_motor_get_feedback_snapshot_age(ENTRY_GIMBAL_ROLL_MOTOR_ID,&f,&r.age_ms);
    r.position=f.position; r.ready=r.ready && (f.valid & BUS_MOTOR_FEEDBACK_POSITION);
    y.ready=assemble_dm_motor_get_feedback_snapshot_age(&f,&y.age_ms) && assemble_dm_motor_is_ready();
    y.position=f.position; y.ready=y.ready && (f.valid & BUS_MOTOR_FEEDBACK_POSITION);
    gimbal_control_tilt_update(&s_entry_gimbal_control,
        s_entry_log_imu_seen?&s_entry_imu_snapshot:NULL,&p,&r,&y,dwt_get_us(),&po,&ro,&pc,&rc);
    if(pc && (!po.valid || !assemble_ht_motor_set_control_target(ENTRY_GIMBAL_PITCH_MOTOR_ID,po.position,po.speed,po.timeout_ms))) {
        assemble_ht_motor_cancel_control(ENTRY_GIMBAL_PITCH_MOTOR_ID);
        if(po.valid) gimbal_control_note_exit(&s_entry_gimbal_control.pitch,5u);
        s_entry_gimbal_control.pitch.active=false;
    }
    if(rc && (!ro.valid || !assemble_ht_motor_set_control_target(ENTRY_GIMBAL_ROLL_MOTOR_ID,ro.position,ro.speed,ro.timeout_ms))) {
        assemble_ht_motor_cancel_control(ENTRY_GIMBAL_ROLL_MOTOR_ID);
        if(ro.valid) gimbal_control_note_exit(&s_entry_gimbal_control.roll,5u);
        s_entry_gimbal_control.roll.active=false;
    }
}

static inline void entry_process_yaw_control(void) {
    BusMotorFeedback feedback = {0};
    uint32_t age_ms = UINT32_MAX;
    GimbalYawFeedback motor = {0};
    GimbalYawCommand command;
    motor.ready = entry_motor_bus_ht_ready() && assemble_dm_motor_is_ready() &&
        assemble_dm_motor_get_feedback_snapshot_age(&feedback, &age_ms) &&
        (feedback.valid & BUS_MOTOR_FEEDBACK_POSITION);
    motor.position = feedback.position;
    motor.velocity = feedback.velocity;
    motor.velocity_valid = (feedback.valid & BUS_MOTOR_FEEDBACK_VELOCITY) != 0u;
    motor.age_ms = age_ms;
    GimbalYawFeedback pitch={0}, roll={0};
    BusMotorFeedback pf={0}, rf={0};
    pitch.ready=assemble_ht_motor_get_feedback_snapshot_age(ENTRY_GIMBAL_PITCH_MOTOR_ID,&pf,&pitch.age_ms);
    pitch.position=pf.position; pitch.ready=pitch.ready && (pf.valid & BUS_MOTOR_FEEDBACK_POSITION);
    pitch.velocity=pf.velocity; pitch.velocity_valid=(pf.valid & BUS_MOTOR_FEEDBACK_VELOCITY)!=0u;
    roll.ready=assemble_ht_motor_get_feedback_snapshot_age(ENTRY_GIMBAL_ROLL_MOTOR_ID,&rf,&roll.age_ms);
    roll.position=rf.position; roll.ready=roll.ready && (rf.valid & BUS_MOTOR_FEEDBACK_POSITION);
    roll.velocity=rf.velocity; roll.velocity_valid=(rf.valid & BUS_MOTOR_FEEDBACK_VELOCITY)!=0u;
    const Bmi088Snapshot *sample = s_entry_log_imu_seen ? &s_entry_imu_snapshot : NULL;
    if(gimbal_control_yaw_update(&s_entry_gimbal_control, sample, &motor, &pitch, &roll, dwt_get_us(), &command)) {
        if(!command.valid ||
           !assemble_dm_motor_set_target_position_guarded(command.position, command.speed, command.timeout_ms))
            assemble_dm_motor_clear_control_target();
    }
}

static inline bool entry_motor_bus_ht_ready(void) {
    return assemble_ht_motor_has_feedback(ENTRY_GIMBAL_PITCH_MOTOR_ID) &&
           assemble_ht_motor_has_feedback(ENTRY_GIMBAL_ROLL_MOTOR_ID);
}

static inline void entry_process_vision_level(void) {
    VisionLevelResult result;

    if(!vision_comms_get_result(&result) ||
       !vision_comms_result_is_fresh(ENTRY_VISION_RESULT_TIMEOUT_MS)) {
        if(delay_nb_ms(&s_entry_vision_log_timer, ENTRY_VISION_LOG_PERIOD_MS)) {
            log_warn("VISION no fresh data (online=%d)",
                     (int)vision_comms_is_online());
        }
        return;
    }

    /* Vision telemetry is emitted by main_yaw_vofa_task() at 30 Hz. */

}

#endif

