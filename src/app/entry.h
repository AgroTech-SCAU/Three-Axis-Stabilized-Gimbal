#ifndef _app_entry_h_
#define _app_entry_h_

/**
 * @file entry.h
 * @brief CubeMX `main.c` 的应用入口替代层
 */

#include "app_runtime.h"
#include "app_status.h"
#include "arm.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "delay.h"
#include "imu/imu.h"
#include "log.h"
#include "odom.h"
#include "pc_comms.h"
#include "pi_comms.h"
#include "remote.h"
#include "vision_comms.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 系统初始化完成标记
 */
static bool init_ok = false;

#define ENTRY_GIMBAL_CONTROL_PERIOD_MS 20u
#define ENTRY_GIMBAL_HOLD_PERIOD_MS 50u
#define ENTRY_GIMBAL_LOG_PERIOD_MS 100u
#define ENTRY_RAD_TO_DEG 57.29577951308232f
#define ENTRY_GIMBAL_ROLL_MOTOR_ID 8u
#define ENTRY_GIMBAL_PITCH_MOTOR_ID 2u
#define ENTRY_GIMBAL_ROLL_ENABLE 1u
#define ENTRY_GIMBAL_PITCH_ENABLE 1u
#define ENTRY_GIMBAL_YAW_ENABLE 1u
#define ENTRY_GIMBAL_ROLL_SIGN (-1.0f)
#define ENTRY_GIMBAL_PITCH_SIGN (-1.0f)
#define ENTRY_GIMBAL_YAW_SIGN (-1.0f)
#define ENTRY_GIMBAL_ROLL_GAIN 1.5f
#define ENTRY_GIMBAL_PITCH_GAIN 1.15f
#define ENTRY_GIMBAL_MAX_OFFSET_RAD 1.8f
#define ENTRY_GIMBAL_LEVEL_DEADBAND_RAD 0.0349065850f
#define ENTRY_GIMBAL_ROLL_MAX_STEP_RAD 0.15f
#define ENTRY_GIMBAL_PITCH_MAX_STEP_RAD 0.08f
#define ENTRY_GIMBAL_YAW_CONTROL_PERIOD_MS 10u
#define ENTRY_GIMBAL_YAW_MAX_SPEED_RAD_S 1.0f
#define ENTRY_GIMBAL_YAW_MAX_STEP_RAD 0.05f
#define ENTRY_VISION_RESULT_TIMEOUT_MS 200u
#define ENTRY_GIMBAL_FAST_ANGLE_DELTA_RAD 0.2617993878f
#define ENTRY_GIMBAL_FAST_ANGLE_SLOW_MS 500u
#define ENTRY_GIMBAL_FAST_ROLL_MAX_STEP_RAD 0.02f
#define ENTRY_GIMBAL_FAST_PITCH_MAX_STEP_RAD 0.01f
#define ENTRY_GIMBAL_HOLD_ROLL_MAX_STEP_RAD 0.005f
#define ENTRY_GIMBAL_HOLD_PITCH_MAX_STEP_RAD 0.002f
#define ENTRY_GIMBAL_ERROR_FILTER_ALPHA 0.15f
#define ENTRY_GIMBAL_ROLL_INITIAL_POSITION_RAD 0.0f
#define ENTRY_GIMBAL_PITCH_INITIAL_POSITION_RAD 0.02f
#define ENTRY_GIMBAL_INITIAL_TOLERANCE_RAD 0.03f
#define ENTRY_HT_ID2_SPEED_TEST_ID 2u
#define ENTRY_HT_ID2_SPEED_TEST_RAD_S 2.0f
#define ENTRY_HT_ID2_SPEED_TEST_TX_PERIOD_MS 20u
#define ENTRY_HT_ID2_SPEED_TEST_LOG_PERIOD_MS 200u
#define ENTRY_HT_ID2_SPEED_TEST_LOOKAHEAD_S 0.20f
#define ENTRY_HT_ID2_SPEED_TEST_KP 1.0f
#define ENTRY_HT_ID2_SPEED_TEST_KD 0.10f

/**
 * @brief 250Hz 节拍计数器
 */
static uint8_t s_entry_250hz_tick = 0u;

/**
 * @brief 100Hz 节拍计数器
 */
static uint8_t s_entry_100hz_tick = 0u;

/**
 * @brief 50Hz 节拍计数器
 */
static uint8_t s_entry_50hz_tick = 0u;

/**
 * @brief 机械臂反馈刷新异常日志节流计时器
 */
static ms_t s_entry_arm_refresh_log_timer = 0u;

/**
 * @brief 机械臂反馈刷新连续失败计数
 */
static uint16_t s_entry_arm_refresh_fail_count = 0u;
static ms_t s_entry_gimbal_control_timer = 0u;
static ms_t s_entry_gimbal_hold_timer = 0u;
static ms_t s_entry_gimbal_log_timer = 0u;
static ms_t s_entry_vision_log_timer = 0u;
static ms_t s_entry_vision_vofa_timer = 0u;
#if ENTRY_GIMBAL_YAW_ENABLE
static ms_t s_entry_gimbal_yaw_control_timer = 0u;
static ms_t s_entry_gimbal_yaw_log_timer = 0u;
static bool s_entry_gimbal_yaw_ref_ready = false;
static float s_entry_gimbal_imu_ref_yaw = 0.0f;
static float s_entry_gimbal_zero_yaw_motor = 0.0f;
static float s_entry_gimbal_yaw_target = 0.0f;
#endif
static bool s_entry_gimbal_hold_target_ready = false;
static bool s_entry_gimbal_hold_logged = false;
static float s_entry_gimbal_hold_roll_target = 0.0f;
static float s_entry_gimbal_hold_pitch_target = 0.0f;
static ms_t s_entry_ht_id2_speed_tx_timer = 0u;
static ms_t s_entry_ht_id2_speed_log_timer = 0u;
static bool s_entry_ht_id2_speed_target_ready = false;
static float s_entry_ht_id2_speed_target_position = 0.0f;
static bool s_entry_gimbal_initial_target_ready = false;
static bool s_entry_gimbal_initial_position_ready = false;
static bool s_entry_gimbal_zero_ready = false;
static bool s_entry_gimbal_last_angle_ready = false;
static ms_t s_entry_gimbal_fast_angle_slow_until = 0u;
static float s_entry_gimbal_last_roll_angle = 0.0f;
static float s_entry_gimbal_last_pitch_angle = 0.0f;
#if ENTRY_GIMBAL_ROLL_ENABLE
static float s_entry_gimbal_zero_roll_angle = 0.0f;
static float s_entry_gimbal_zero_roll_motor = 0.0f;
static float s_entry_gimbal_initial_roll_target = 0.0f;
static float s_entry_gimbal_roll_target = 0.0f;
static float s_entry_gimbal_roll_error_filter = 0.0f;
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
static float s_entry_gimbal_zero_pitch_angle = 0.0f;
static float s_entry_gimbal_zero_pitch_motor = 0.0f;
static float s_entry_gimbal_initial_pitch_target = 0.0f;
static float s_entry_gimbal_pitch_target = 0.0f;
static float s_entry_gimbal_pitch_error_filter = 0.0f;
#endif

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 判断当前 500Hz 周期是否命中 250Hz 节拍
 * @return bool `true` 表示应执行 250Hz 任务
 */
static inline bool entry_tick_250hz(void);

/**
 * @brief 判断当前 500Hz 周期是否命中 100Hz 节拍
 * @return bool `true` 表示应执行 100Hz 任务
 */
static inline bool entry_tick_100hz(void);

/**
 * @brief 判断当前 500Hz 周期是否命中 50Hz 节拍
 * @return bool `true` 表示应执行 50Hz 任务
 */
static inline bool entry_tick_50hz(void);
static inline void entry_process_gimbal_level(void);
static inline void entry_process_gimbal_yaw(float yaw_angle);
static inline void entry_process_vision_level(void);
static inline void entry_process_gimbal_hold_position(void);
static inline void entry_process_ht_id2_speed_test(void);
static inline float entry_clampf(float value, float min_value, float max_value);
static inline float entry_step_towardsf(float current, float target, float max_step);
static inline float entry_absf(float value);
static inline float entry_apply_deadbandf(float value, float deadband);
static inline float entry_wrap_pi(float angle);
static inline float entry_wrap_one_turn(float position);
static inline float entry_nearest_absolute_target(float current_position, float encoder_target);
static inline void entry_update_gimbal_fast_angle_guard(float roll_angle, float pitch_angle);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 系统初始化入口
 * @details 按照启动依赖顺序完成底层装配, 应用层初始化, 最后启动 500Hz 定时调度
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK)
        return;

    if(assemble_log() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT log ready");

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

    if(assemble_vision_comms() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT vision comms ready");
    delay_ms(100u);

#if 0 /* Gimbal HT test: disable chassis, arm and application services. */

    if(assemble_rgb() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT rgb ready");
    delay_ms(100u);

    if(assemble_odom() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT odom ready");
    delay_ms(100u);

    if(assemble_chassis() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT chassis ready");
    delay_ms(100u);

    if(assemble_arm() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT arm ready");
    delay_ms(100u);

    if(assemble_suction() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT suction ready");
    delay_ms(100u);

    if(assemble_remote() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT remote ready");
    delay_ms(100u);

    {
        remote_init();

        if(assemble_pc_comms() != SYSTEM_STATUS_OK)
            return;

        if(assemble_pi_comms() != SYSTEM_STATUS_OK)
            return;

        app_runtime_init();
        app_status_init();
    }
    log_info("BOOT app ready");
    delay_ms(100u);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT tim6 500hz ready");
    delay_ms(100u);
#endif

    log_info("System initialized successfully");
    delay_ms(500u);

    init_ok = true;
}

/**
 * @brief 主循环调度入口
 * @details
 *
 * 500Hz base order:
 * 1. chassis.process()
 * 2. 250Hz slot: odom.process()
 * 3. 100Hz slot: remote_process() -> pc_comms_process() -> pi_comms_process()
 * 4. 50Hz slot: arm.refresh_current_state()
 * 5. app_runtime_process()
 *
 * background:
 * - app_status_process()
 */
static inline void entry_loop(void) {
    if(!init_ok) {
        return;
    }

    assemble_dm_motor_process();
    assemble_ht_motor_process();
    entry_process_gimbal_level();
    vision_comms_process();
    entry_process_vision_level();

#if 0 /* Gimbal HT test: no chassis/arm periodic traffic. */
    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        chassis.process();

        if(entry_tick_250hz()) {
            odom.process();
        }

        if(entry_tick_100hz()) {
            remote_process();
            pc_comms_process();
            pi_comms_process();
        }

        if(entry_tick_50hz()) {
            ArmStatus arm_status = arm.refresh_current_state();

            if(arm_status == ARM_OK) {
                s_entry_arm_refresh_fail_count = 0u;
            }
            else {
                s_entry_arm_refresh_fail_count++;
                if(s_entry_arm_refresh_fail_count >= 10u && delay_nb_ms(&s_entry_arm_refresh_log_timer, 1000u)) {
                    log_warn("ARM refresh unstable: %s, consecutive=%u",
                             arm.status_str(arm_status),
                             s_entry_arm_refresh_fail_count);
                }
            }
        }

        app_runtime_process();
    }

    app_status_process();
#endif
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static inline bool entry_tick_250hz(void) {
    s_entry_250hz_tick = (uint8_t)((s_entry_250hz_tick + 1u) % 2u);
    return s_entry_250hz_tick == 0u;
}

static inline bool entry_tick_100hz(void) {
    s_entry_100hz_tick = (uint8_t)((s_entry_100hz_tick + 1u) % 5u);
    return s_entry_100hz_tick == 0u;
}

static inline bool entry_tick_50hz(void) {
    s_entry_50hz_tick = (uint8_t)((s_entry_50hz_tick + 1u) % 10u);
    return s_entry_50hz_tick == 0u;
}

static inline void entry_process_ht_id2_speed_test(void) {
    float position;
    bool has_feedback;

    has_feedback = assemble_ht_motor_has_feedback(ENTRY_HT_ID2_SPEED_TEST_ID);

    if(has_feedback && assemble_ht_motor_get_raw_position(ENTRY_HT_ID2_SPEED_TEST_ID, &position)) {
        if(!s_entry_ht_id2_speed_target_ready) {
            s_entry_ht_id2_speed_target_position = position;
            s_entry_ht_id2_speed_target_ready = true;
            log_info("HT speed test start id=%u from_rad=%.3f cmd=%.3f rad_s",
                     ENTRY_HT_ID2_SPEED_TEST_ID,
                     (double)position,
                     (double)ENTRY_HT_ID2_SPEED_TEST_RAD_S);
        }

        if(delay_nb_ms(&s_entry_ht_id2_speed_tx_timer, ENTRY_HT_ID2_SPEED_TEST_TX_PERIOD_MS)) {
            s_entry_ht_id2_speed_target_position = position + ENTRY_HT_ID2_SPEED_TEST_RAD_S * ENTRY_HT_ID2_SPEED_TEST_LOOKAHEAD_S;

            (void)assemble_ht_motor_set_target_position_speed(
                ENTRY_HT_ID2_SPEED_TEST_ID,
                s_entry_ht_id2_speed_target_position,
                ENTRY_HT_ID2_SPEED_TEST_RAD_S);
        }

        if(delay_nb_ms(&s_entry_ht_id2_speed_log_timer, ENTRY_HT_ID2_SPEED_TEST_LOG_PERIOD_MS)) {
            log_info("HT speed test id=%u cmd=%.3f rad_s cur_rad=%.3f cur_mrad=%ld target_rad=%.3f",
                     ENTRY_HT_ID2_SPEED_TEST_ID,
                     (double)ENTRY_HT_ID2_SPEED_TEST_RAD_S,
                     (double)position,
                     (long)(entry_wrap_one_turn(position) * 1000.0f),
                     (double)s_entry_ht_id2_speed_target_position);
        }
    }
    else if(delay_nb_ms(&s_entry_ht_id2_speed_log_timer, ENTRY_HT_ID2_SPEED_TEST_LOG_PERIOD_MS)) {
        log_warn("HT speed test id=%u cmd=%.3f rad_s no feedback",
                 ENTRY_HT_ID2_SPEED_TEST_ID,
                 (double)ENTRY_HT_ID2_SPEED_TEST_RAD_S);
    }
}

static inline void entry_process_gimbal_hold_position(void) {
    bool roll_ready = assemble_ht_motor_has_feedback(ENTRY_GIMBAL_ROLL_MOTOR_ID);
    bool pitch_ready = assemble_ht_motor_has_feedback(ENTRY_GIMBAL_PITCH_MOTOR_ID);
    float roll_position = 0.0f;
    float pitch_position = 0.0f;

    if(!roll_ready || !pitch_ready) {
        return;
    }

    if(!assemble_ht_motor_get_raw_position(ENTRY_GIMBAL_ROLL_MOTOR_ID, &roll_position) || !assemble_ht_motor_get_raw_position(ENTRY_GIMBAL_PITCH_MOTOR_ID, &pitch_position)) {
        return;
    }

    if(!s_entry_gimbal_hold_target_ready) {
        s_entry_gimbal_hold_roll_target = roll_position;
        s_entry_gimbal_hold_pitch_target = pitch_position;
        s_entry_gimbal_hold_target_ready = true;
    }

    if(!s_entry_gimbal_hold_logged) {
        s_entry_gimbal_hold_logged = true;
        log_info("HT position test enabled roll_id=%u target=%.3f pitch_id=%u target=%.3f",
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)ENTRY_GIMBAL_ROLL_INITIAL_POSITION_RAD,
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)ENTRY_GIMBAL_PITCH_INITIAL_POSITION_RAD);
    }

    if(delay_nb_ms(&s_entry_gimbal_hold_timer, ENTRY_GIMBAL_HOLD_PERIOD_MS)) {
        s_entry_gimbal_hold_roll_target = entry_step_towardsf(
            s_entry_gimbal_hold_roll_target,
            ENTRY_GIMBAL_ROLL_INITIAL_POSITION_RAD,
            ENTRY_GIMBAL_HOLD_ROLL_MAX_STEP_RAD);
        s_entry_gimbal_hold_pitch_target = entry_step_towardsf(
            s_entry_gimbal_hold_pitch_target,
            ENTRY_GIMBAL_PITCH_INITIAL_POSITION_RAD,
            ENTRY_GIMBAL_HOLD_PITCH_MAX_STEP_RAD);

        (void)assemble_ht_motor_set_target_position(
            ENTRY_GIMBAL_ROLL_MOTOR_ID,
            s_entry_gimbal_hold_roll_target);
        (void)assemble_ht_motor_set_target_position(
            ENTRY_GIMBAL_PITCH_MOTOR_ID,
            s_entry_gimbal_hold_pitch_target);
    }

    if(delay_nb_ms(&s_entry_gimbal_log_timer, ENTRY_GIMBAL_LOG_PERIOD_MS)) {
        log_info("HT position test roll_id=%u cur=%.3f target=%.3f pitch_id=%u cur=%.3f target=%.3f",
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)roll_position,
                 (double)s_entry_gimbal_hold_roll_target,
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)pitch_position,
                 (double)s_entry_gimbal_hold_pitch_target);
    }
}

static inline void entry_process_gimbal_level(void) {
    ImuStatus status = imu_update();
    ImuAngle angle;
#if ENTRY_GIMBAL_ROLL_ENABLE
    float roll_error;
    float roll_min;
    float roll_max;
    float roll_target;
    float roll_position;
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
    float pitch_error;
    float pitch_min;
    float pitch_max;
    float pitch_target;
    float pitch_position;
#endif
    float roll_step = ENTRY_GIMBAL_ROLL_MAX_STEP_RAD;
    float pitch_step = ENTRY_GIMBAL_PITCH_MAX_STEP_RAD;

    if(status != IMU_STATUS_OK) {
        return;
    }

    /* IMU 每轮只更新一次，Yaw 独立于 HT Roll/Pitch readiness 运行。 */
    angle = imu_get_angle();
#if ENTRY_GIMBAL_YAW_ENABLE
    entry_process_gimbal_yaw(angle.yaw);
#endif

#if ENTRY_GIMBAL_ROLL_ENABLE
    if(!assemble_ht_motor_has_feedback(ENTRY_GIMBAL_ROLL_MOTOR_ID)) {
        return;
    }
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
    if(!assemble_ht_motor_has_feedback(ENTRY_GIMBAL_PITCH_MOTOR_ID)) {
        return;
    }
#endif

#if !ENTRY_GIMBAL_ROLL_ENABLE && !ENTRY_GIMBAL_PITCH_ENABLE
    if(delay_nb_ms(&s_entry_gimbal_log_timer, ENTRY_GIMBAL_LOG_PERIOD_MS)) {
        log_warn("GIMBAL disabled");
    }
    return;
#endif

    entry_update_gimbal_fast_angle_guard(angle.roll, angle.pitch);
    if((int32_t)(s_entry_gimbal_fast_angle_slow_until - HAL_GetTick()) > 0) {
        roll_step = ENTRY_GIMBAL_FAST_ROLL_MAX_STEP_RAD;
        pitch_step = ENTRY_GIMBAL_FAST_PITCH_MAX_STEP_RAD;
    }

#if ENTRY_GIMBAL_ROLL_ENABLE
    if(!assemble_ht_motor_get_raw_position(ENTRY_GIMBAL_ROLL_MOTOR_ID, &roll_position)) {
        return;
    }
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
    if(!assemble_ht_motor_get_raw_position(ENTRY_GIMBAL_PITCH_MOTOR_ID, &pitch_position)) {
        return;
    }
#endif

    if(!s_entry_gimbal_initial_target_ready) {
#if ENTRY_GIMBAL_ROLL_ENABLE
        s_entry_gimbal_initial_roll_target = entry_nearest_absolute_target(
            roll_position, ENTRY_GIMBAL_ROLL_INITIAL_POSITION_RAD);
        s_entry_gimbal_roll_target = roll_position;
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
        s_entry_gimbal_initial_pitch_target = entry_nearest_absolute_target(
            pitch_position, ENTRY_GIMBAL_PITCH_INITIAL_POSITION_RAD);
        s_entry_gimbal_pitch_target = pitch_position;
#endif
        s_entry_gimbal_initial_target_ready = true;

#if ENTRY_GIMBAL_ROLL_ENABLE && ENTRY_GIMBAL_PITCH_ENABLE
        log_info("GIMBAL init target roll_id=%u rad=%.3f mrad=%ld pitch_id=%u rad=%.3f mrad=%ld",
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)s_entry_gimbal_initial_roll_target,
                 (long)(entry_wrap_one_turn(s_entry_gimbal_initial_roll_target) * 1000.0f),
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)s_entry_gimbal_initial_pitch_target,
                 (long)(entry_wrap_one_turn(s_entry_gimbal_initial_pitch_target) * 1000.0f));
#elif ENTRY_GIMBAL_ROLL_ENABLE
        log_info("GIMBAL init target roll_id=%u pos=%.3f pitch=off",
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)s_entry_gimbal_initial_roll_target);
#elif ENTRY_GIMBAL_PITCH_ENABLE
        log_info("GIMBAL init target roll=off pitch_id=%u pos=%.3f",
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)s_entry_gimbal_initial_pitch_target);
#endif
    }

    if(!s_entry_gimbal_initial_position_ready) {
        if(delay_nb_ms(&s_entry_gimbal_control_timer, ENTRY_GIMBAL_CONTROL_PERIOD_MS)) {
#if ENTRY_GIMBAL_ROLL_ENABLE
            s_entry_gimbal_roll_target = entry_step_towardsf(
                s_entry_gimbal_roll_target,
                s_entry_gimbal_initial_roll_target,
                roll_step);
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
            s_entry_gimbal_pitch_target = entry_step_towardsf(
                s_entry_gimbal_pitch_target,
                s_entry_gimbal_initial_pitch_target,
                pitch_step);
#endif

#if ENTRY_GIMBAL_ROLL_ENABLE
            (void)assemble_ht_motor_set_target_position(ENTRY_GIMBAL_ROLL_MOTOR_ID, s_entry_gimbal_roll_target);
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
            (void)assemble_ht_motor_set_target_position(ENTRY_GIMBAL_PITCH_MOTOR_ID, s_entry_gimbal_pitch_target);
#endif
        }

#if ENTRY_GIMBAL_ROLL_ENABLE && ENTRY_GIMBAL_PITCH_ENABLE
        if(entry_absf(roll_position - s_entry_gimbal_initial_roll_target) <= ENTRY_GIMBAL_INITIAL_TOLERANCE_RAD && entry_absf(pitch_position - s_entry_gimbal_initial_pitch_target) <= ENTRY_GIMBAL_INITIAL_TOLERANCE_RAD) {
#elif ENTRY_GIMBAL_ROLL_ENABLE
        if(entry_absf(roll_position - s_entry_gimbal_initial_roll_target) <= ENTRY_GIMBAL_INITIAL_TOLERANCE_RAD) {
#elif ENTRY_GIMBAL_PITCH_ENABLE
        if(entry_absf(pitch_position - s_entry_gimbal_initial_pitch_target) <= ENTRY_GIMBAL_INITIAL_TOLERANCE_RAD) {
#endif
            s_entry_gimbal_initial_position_ready = true;
            log_info("GIMBAL init position ready");
        }

        if(delay_nb_ms(&s_entry_gimbal_log_timer, ENTRY_GIMBAL_LOG_PERIOD_MS)) {
#if ENTRY_GIMBAL_ROLL_ENABLE && ENTRY_GIMBAL_PITCH_ENABLE
            log_info("GIMBAL init imu roll=%.2f pitch=%.2f yaw=%.2f deg out roll_id=%u rad=%.3f mrad=%ld pitch_id=%u rad=%.3f mrad=%ld",
                     (double)(angle.roll * ENTRY_RAD_TO_DEG),
                     (double)(angle.pitch * ENTRY_RAD_TO_DEG),
                     (double)(angle.yaw * ENTRY_RAD_TO_DEG),
                     ENTRY_GIMBAL_ROLL_MOTOR_ID,
                     (double)s_entry_gimbal_roll_target,
                     (long)(entry_wrap_one_turn(s_entry_gimbal_roll_target) * 1000.0f),
                     ENTRY_GIMBAL_PITCH_MOTOR_ID,
                     (double)s_entry_gimbal_pitch_target,
                     (long)(entry_wrap_one_turn(s_entry_gimbal_pitch_target) * 1000.0f));
#elif ENTRY_GIMBAL_ROLL_ENABLE
            log_info("GIMBAL init imu roll=%.2f pitch=%.2f yaw=%.2f deg out roll_id=%u pos=%.3f pitch=off",
                     (double)(angle.roll * ENTRY_RAD_TO_DEG),
                     (double)(angle.pitch * ENTRY_RAD_TO_DEG),
                     (double)(angle.yaw * ENTRY_RAD_TO_DEG),
                     ENTRY_GIMBAL_ROLL_MOTOR_ID,
                     (double)s_entry_gimbal_roll_target);
#elif ENTRY_GIMBAL_PITCH_ENABLE
            log_info("GIMBAL init imu roll=%.2f pitch=%.2f yaw=%.2f deg out roll=off pitch_id=%u pos=%.3f",
                     (double)(angle.roll * ENTRY_RAD_TO_DEG),
                     (double)(angle.pitch * ENTRY_RAD_TO_DEG),
                     (double)(angle.yaw * ENTRY_RAD_TO_DEG),
                     ENTRY_GIMBAL_PITCH_MOTOR_ID,
                     (double)s_entry_gimbal_pitch_target);
#endif
        }
        return;
    }

    if(!s_entry_gimbal_zero_ready) {
#if ENTRY_GIMBAL_ROLL_ENABLE
        s_entry_gimbal_zero_roll_angle = angle.roll;
        s_entry_gimbal_zero_roll_motor = s_entry_gimbal_initial_roll_target;
        s_entry_gimbal_roll_target = s_entry_gimbal_initial_roll_target;
        s_entry_gimbal_roll_error_filter = 0.0f;
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
        s_entry_gimbal_zero_pitch_angle = angle.pitch;
        s_entry_gimbal_zero_pitch_motor = s_entry_gimbal_initial_pitch_target;
        s_entry_gimbal_pitch_target = s_entry_gimbal_initial_pitch_target;
        s_entry_gimbal_pitch_error_filter = 0.0f;
#endif
        s_entry_gimbal_zero_ready = true;

#if ENTRY_GIMBAL_ROLL_ENABLE && ENTRY_GIMBAL_PITCH_ENABLE
        log_info("GIMBAL zero imu_roll=%.2f imu_pitch=%.2f roll_id=%u rad=%.3f mrad=%ld pitch_id=%u rad=%.3f mrad=%ld",
                 (double)(s_entry_gimbal_zero_roll_angle * ENTRY_RAD_TO_DEG),
                 (double)(s_entry_gimbal_zero_pitch_angle * ENTRY_RAD_TO_DEG),
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)s_entry_gimbal_zero_roll_motor,
                 (long)(entry_wrap_one_turn(s_entry_gimbal_zero_roll_motor) * 1000.0f),
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)s_entry_gimbal_zero_pitch_motor,
                 (long)(entry_wrap_one_turn(s_entry_gimbal_zero_pitch_motor) * 1000.0f));
#elif ENTRY_GIMBAL_ROLL_ENABLE
        log_info("GIMBAL zero imu_roll=%.2f imu_pitch=%.2f roll_id=%u pos=%.3f pitch=off",
                 (double)(s_entry_gimbal_zero_roll_angle * ENTRY_RAD_TO_DEG),
                 (double)(angle.pitch * ENTRY_RAD_TO_DEG),
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)s_entry_gimbal_zero_roll_motor);
#elif ENTRY_GIMBAL_PITCH_ENABLE
        log_info("GIMBAL zero imu_roll=%.2f imu_pitch=%.2f roll=off pitch_id=%u pos=%.3f",
                 (double)(angle.roll * ENTRY_RAD_TO_DEG),
                 (double)(s_entry_gimbal_zero_pitch_angle * ENTRY_RAD_TO_DEG),
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)s_entry_gimbal_zero_pitch_motor);
#endif
    }

    if(delay_nb_ms(&s_entry_gimbal_control_timer, ENTRY_GIMBAL_CONTROL_PERIOD_MS)) {
#if ENTRY_GIMBAL_ROLL_ENABLE
        roll_error = entry_apply_deadbandf(angle.roll - s_entry_gimbal_zero_roll_angle, ENTRY_GIMBAL_LEVEL_DEADBAND_RAD);
        s_entry_gimbal_roll_error_filter += ENTRY_GIMBAL_ERROR_FILTER_ALPHA * (roll_error - s_entry_gimbal_roll_error_filter);

        roll_min = s_entry_gimbal_zero_roll_motor - ENTRY_GIMBAL_MAX_OFFSET_RAD;
        roll_max = s_entry_gimbal_zero_roll_motor + ENTRY_GIMBAL_MAX_OFFSET_RAD;
        roll_target = s_entry_gimbal_zero_roll_motor + (ENTRY_GIMBAL_ROLL_SIGN * ENTRY_GIMBAL_ROLL_GAIN * s_entry_gimbal_roll_error_filter);
        roll_target = entry_clampf(roll_target, roll_min, roll_max);
        s_entry_gimbal_roll_target = entry_step_towardsf(
            s_entry_gimbal_roll_target, roll_target, roll_step);
        (void)assemble_ht_motor_set_target_position(ENTRY_GIMBAL_ROLL_MOTOR_ID, s_entry_gimbal_roll_target);
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
        pitch_error = entry_apply_deadbandf(angle.pitch - s_entry_gimbal_zero_pitch_angle, ENTRY_GIMBAL_LEVEL_DEADBAND_RAD);
        s_entry_gimbal_pitch_error_filter += ENTRY_GIMBAL_ERROR_FILTER_ALPHA * (pitch_error - s_entry_gimbal_pitch_error_filter);

        pitch_min = s_entry_gimbal_zero_pitch_motor - ENTRY_GIMBAL_MAX_OFFSET_RAD;
        pitch_max = s_entry_gimbal_zero_pitch_motor + ENTRY_GIMBAL_MAX_OFFSET_RAD;
        pitch_target = s_entry_gimbal_zero_pitch_motor + (ENTRY_GIMBAL_PITCH_SIGN * ENTRY_GIMBAL_PITCH_GAIN * s_entry_gimbal_pitch_error_filter);
        pitch_target = entry_clampf(pitch_target, pitch_min, pitch_max);
        s_entry_gimbal_pitch_target = entry_step_towardsf(
            s_entry_gimbal_pitch_target, pitch_target, pitch_step);
        (void)assemble_ht_motor_set_target_position(ENTRY_GIMBAL_PITCH_MOTOR_ID, s_entry_gimbal_pitch_target);
#endif
    }

    if(delay_nb_ms(&s_entry_gimbal_log_timer, ENTRY_GIMBAL_LOG_PERIOD_MS)) {
#if ENTRY_GIMBAL_ROLL_ENABLE && ENTRY_GIMBAL_PITCH_ENABLE
        log_info("GIMBAL imu roll=%.2f pitch=%.2f yaw=%.2f deg out roll_id=%u rad=%.3f mrad=%ld pitch_id=%u rad=%.3f mrad=%ld",
                 (double)(angle.roll * ENTRY_RAD_TO_DEG),
                 (double)(angle.pitch * ENTRY_RAD_TO_DEG),
                 (double)(angle.yaw * ENTRY_RAD_TO_DEG),
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)s_entry_gimbal_roll_target,
                 (long)(entry_wrap_one_turn(s_entry_gimbal_roll_target) * 1000.0f),
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)s_entry_gimbal_pitch_target,
                 (long)(entry_wrap_one_turn(s_entry_gimbal_pitch_target) * 1000.0f));
#elif ENTRY_GIMBAL_ROLL_ENABLE
        log_info("GIMBAL imu roll=%.2f pitch=%.2f yaw=%.2f deg out roll_id=%u pos=%.3f pitch=off",
                 (double)(angle.roll * ENTRY_RAD_TO_DEG),
                 (double)(angle.pitch * ENTRY_RAD_TO_DEG),
                 (double)(angle.yaw * ENTRY_RAD_TO_DEG),
                 ENTRY_GIMBAL_ROLL_MOTOR_ID,
                 (double)s_entry_gimbal_roll_target);
#elif ENTRY_GIMBAL_PITCH_ENABLE
        log_info("GIMBAL imu roll=%.2f pitch=%.2f yaw=%.2f deg out roll=off pitch_id=%u pos=%.3f",
                 (double)(angle.roll * ENTRY_RAD_TO_DEG),
                 (double)(angle.pitch * ENTRY_RAD_TO_DEG),
                 (double)(angle.yaw * ENTRY_RAD_TO_DEG),
                 ENTRY_GIMBAL_PITCH_MOTOR_ID,
                 (double)s_entry_gimbal_pitch_target);
#endif
    }
}

static inline void entry_process_gimbal_yaw(float yaw_angle) {
#if ENTRY_GIMBAL_YAW_ENABLE
    float motor_position;
    float yaw_delta;
    float yaw_position_target;

    if(!assemble_dm_motor_is_ready() || !assemble_dm_motor_get_position(&motor_position)) {
        /* 反馈断流或 DM 故障后重新建立参考，避免恢复时突然跳转。 */
        s_entry_gimbal_yaw_ref_ready = false;
        return;
    }

    if(!s_entry_gimbal_yaw_ref_ready) {
        s_entry_gimbal_imu_ref_yaw = yaw_angle;
        s_entry_gimbal_zero_yaw_motor = motor_position;
        s_entry_gimbal_yaw_target = motor_position;
        s_entry_gimbal_yaw_control_timer = HAL_GetTick();
        s_entry_gimbal_yaw_ref_ready = true;
        log_info("GIMBAL yaw ref imu=%.2f deg motor=%.3f rad",
                 (double)(yaw_angle * ENTRY_RAD_TO_DEG),
                 (double)motor_position);
        return;
    }

    if(delay_nb_ms(&s_entry_gimbal_yaw_control_timer, ENTRY_GIMBAL_YAW_CONTROL_PERIOD_MS)) {
        /* IMU yaw 在 [-pi,pi) 跳变，必须先做最短角差再映射到电机目标。 */
        yaw_delta = entry_wrap_pi(yaw_angle - s_entry_gimbal_imu_ref_yaw);
        yaw_position_target = s_entry_gimbal_zero_yaw_motor + ENTRY_GIMBAL_YAW_SIGN * yaw_delta;
        s_entry_gimbal_yaw_target = entry_step_towardsf(
            s_entry_gimbal_yaw_target,
            yaw_position_target,
            ENTRY_GIMBAL_YAW_MAX_STEP_RAD);
        (void)assemble_dm_motor_set_target_position(
            s_entry_gimbal_yaw_target,
            ENTRY_GIMBAL_YAW_MAX_SPEED_RAD_S);
    }

    if(delay_nb_ms(&s_entry_gimbal_yaw_log_timer, 200u)) {
        log_info("GIMBAL yaw imu=%.2f deg target=%.3f rad dm=%.3f rad",
                 (double)(yaw_angle * ENTRY_RAD_TO_DEG),
                 (double)s_entry_gimbal_yaw_target,
                 (double)motor_position);
    }
#else
    (void)yaw_angle;
#endif
}

static inline void entry_process_vision_level(void) {
    VisionLevelResult result;

    if(!vision_comms_get_result(&result) ||
       !vision_comms_result_is_fresh(ENTRY_VISION_RESULT_TIMEOUT_MS)) {
        if(delay_nb_ms(&s_entry_vision_log_timer, ENTRY_GIMBAL_LOG_PERIOD_MS)) {
            log_warn("VISION no fresh data (online=%d)",
                     (int)vision_comms_is_online());
        }
        return;
    }

    /* VOFA+ 火力协议打印：通道名 state / corr_x / corr_y，50Hz */
    if(delay_nb_ms(&s_entry_vision_vofa_timer, 20u)) {
        int state = (int)result.state;
        float corr_x = result.corr_x_deg;
        float corr_y = result.corr_y_deg;
        log_vofa(state, corr_x, corr_y);
    }

    /* 用相机修正角驱动云台调平（示例，按需启用）：
     *   if(result.state == VISION_LEVEL_STATE_NOT_LEVEL) {
     *       把 corr_x_deg / corr_y_deg 映射到 roll/pitch 电机目标；
     *       参考 entry_process_gimbal_level() 中 IMU 误差的用法。
     *   }
     */
}

static inline void entry_update_gimbal_fast_angle_guard(float roll_angle, float pitch_angle) {
    ms_t now = HAL_GetTick();

    if(s_entry_gimbal_last_angle_ready) {
        float roll_delta = entry_absf(entry_wrap_pi(roll_angle - s_entry_gimbal_last_roll_angle));
        float pitch_delta = entry_absf(entry_wrap_pi(pitch_angle - s_entry_gimbal_last_pitch_angle));

        if((roll_delta > ENTRY_GIMBAL_FAST_ANGLE_DELTA_RAD) || (pitch_delta > ENTRY_GIMBAL_FAST_ANGLE_DELTA_RAD)) {
            s_entry_gimbal_fast_angle_slow_until = now + ENTRY_GIMBAL_FAST_ANGLE_SLOW_MS;
        }
    }

    s_entry_gimbal_last_roll_angle = roll_angle;
    s_entry_gimbal_last_pitch_angle = pitch_angle;
    s_entry_gimbal_last_angle_ready = true;
}
static inline float entry_clampf(float value, float min_value, float max_value) {
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}

static inline float entry_step_towardsf(float current, float target, float max_step) {
    float delta = target - current;

    if(delta > max_step) {
        return current + max_step;
    }
    if(delta < -max_step) {
        return current - max_step;
    }
    return target;
}

static inline float entry_absf(float value) {
    return value < 0.0f ? -value : value;
}

static inline float entry_apply_deadbandf(float value, float deadband) {
    if(entry_absf(value) <= deadband) {
        return 0.0f;
    }
    if(value > 0.0f) {
        return value - deadband;
    }
    return value + deadband;
}
static inline float entry_wrap_pi(float angle) {
    while(angle > 3.14159265358979323846f) {
        angle -= 6.28318530717958647692f;
    }
    while(angle < -3.14159265358979323846f) {
        angle += 6.28318530717958647692f;
    }
    return angle;
}

static inline float entry_wrap_one_turn(float position) {
    float wrapped = fmodf(position, 6.28318530717958647692f);

    if(wrapped < 0.0f) {
        wrapped += 6.28318530717958647692f;
    }
    return wrapped;
}

static inline float entry_nearest_absolute_target(float current_position, float encoder_target) {
    float current_wrapped = entry_wrap_one_turn(current_position);
    float target_wrapped = entry_wrap_one_turn(encoder_target);
    float delta = target_wrapped - current_wrapped;

    if(delta > 3.14159265358979323846f) {
        delta -= 6.28318530717958647692f;
    }
    else if(delta < -3.14159265358979323846f) {
        delta += 6.28318530717958647692f;
    }

    return current_position + delta;
}

#endif
