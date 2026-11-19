#include "gimbal_config.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

GimbalConfig gimbal_project_config(const GimbalPortOps* ops) {
    GimbalConfig config = {
        .ops = ops,
        .motor_id = { GIMBAL_YAW_MOTOR_ID, GIMBAL_ROLL_MOTOR_ID, GIMBAL_PITCH_MOTOR_ID },
        .hold_speed = 0.3f,
        .hold_period_ms = 10u,
        .hold_timeout_ms = 100u,
        .prepare_feedback_max_age_ms = 200u,
        .run_recovery_ms = 30u, /**< 保持 HT 60ms 有效门槛；等待时不算新目标，仍留足 Yaw 40ms 看门狗余量 */
        .status_period_ms = 1000u,
        .raw_log_period_ms = 100u,
        .control = {
            /* 用户确认：IMU 前X/左Y/上Z；三轴正向符合右手法则，中位反馈均为0
             * 沿用既有外Yaw(Z)-中Roll(X)-内Pitch(Y)机构模型及 +/-45度工作范围
             * 反馈已为关节输出角，ratio=1；不重复除减速比，不将启动位置改写为零位
             * 上电完成标定与准备保持后自动进入 RUN
             */
            .coordinates_confirmed = true,
            .coupled_tilt = false,
            .min_cos_pitch = 0.50f,
            .min_axis_response = 0.20f,
            .min_jacobian_det = 0.20f,
            .imu_max_age_us = 10000u,
            .stall_timeout_us = 100000u,
            .fusion_max_skew_us = 10000u,
            .yaw_ff_gain = 0.8f,
            .yaw_ff_lead_s = 0.10f,
            .yaw_ff_filter_s = 0.006f,
            .yaw_ff_deadband = 0.1f,
            .axis = {
                [GIMBAL_AXIS_YAW] = {
                    .direction = 1.0f,
                    .ratio = 1.0f,
                    .zero = 0.0f,
                    .min_angle = -0.785398163f,
                    .max_angle = 0.785398163f,
                    .protocol_limit = 12.5f,
                    .kp = 3.0f,
                    .kd = 0.03f,
                    .deadband = 0.000745329f,
                    .error_power = 1.0f,
                    .error_reference = 0.052359878f,
                    .max_correction = 10.0f,
                    .max_speed = 40.0f,
                    .command_speed = 30.0f,
                    .goal_filter_s = 0.0f,
                    .period_us = 3000u,
                    .max_step_us = 10000u,
                    .feedback_max_age_ms = 20u,
                    .command_timeout_ms = 40u,
                },
                [GIMBAL_AXIS_ROLL] = {
                    .direction = 1.0f,
                    .ratio = 1.0f,
                    .zero = 0.0f,
                    .min_angle = -0.785398163f,
                    .max_angle = 0.785398163f,
                    .protocol_limit = 20.58f,
                    .kp = 1.0f,
                    .kd = 0.001f,
                    .deadband = 0.0034906585f,
                    .error_power = 1.1f,
                    .error_reference = 0.052359878f,
                    .max_correction = 3.0f,
                    .max_speed = 1.5f,
                    .command_speed = 1.5f,
                    .goal_filter_s = 0.020f,
                    .period_us = 10000u,
                    .max_step_us = 30000u,
                    .feedback_max_age_ms = 60u,
                    .command_timeout_ms = 100u,
                },
                [GIMBAL_AXIS_PITCH] = {
                    .direction = 1.0f,
                    .ratio = 1.0f,
                    .zero = 0.0f,
                    .min_angle = -0.785398163f,
                    .max_angle = 0.785398163f,
                    .protocol_limit = 20.58f,
                    .kp = 1.0f,
                    .kd = 0.001f,
                    .deadband = 0.0034906585f,
                    .error_power = 1.0f,
                    .error_reference = 0.052359878f,
                    .max_correction = 30.0f,
                    .max_speed = 1.5f,
                    .command_speed = 1.5f,
                    .goal_filter_s = 0.020f,
                    .period_us = 10000u,
                    .max_step_us = 30000u,
                    .feedback_max_age_ms = 60u,
                    .command_timeout_ms = 100u,
                },
            },
        },
    };
    return config;
}
