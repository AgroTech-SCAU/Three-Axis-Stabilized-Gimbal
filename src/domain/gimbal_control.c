#include "gimbal_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static float gimbal_clamp(float value, float lower, float upper);
static float gimbal_wrap(float angle);
static bool gimbal_config_valid(const GimbalControlConfig* config);
static void gimbal_encoder_limits(const GimbalAxisConfig* config, float* lower, float* upper);
static float gimbal_error_correction(const GimbalAxisConfig* config, float error, float rate);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

GimbalControlStatus gimbal_control_check(const GimbalControlConfig* config,
                                         const GimbalMeasurement* measurement, const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT], uint32_t now_us) {
    if(!gimbal_config_valid(config) || measurement == NULL || motor == NULL) {
        return GIMBAL_CONTROL_INVALID_PARAM;
    }
    if(!config->coordinates_confirmed) {
        return GIMBAL_CONTROL_UNCONFIRMED;
    }
    /* 新鲜反馈越限立即退出，其他输入失效不延后限位保护 */
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalAxisConfig* axis = &config->axis[i];
        if(motor[i].ready && motor[i].age_ms <= axis->feedback_max_age_ms && isfinite(motor[i].position)) {
            float joint = axis->direction * axis->ratio * (motor[i].position - axis->zero);
            if(!isfinite(joint) || joint < axis->min_angle || joint > axis->max_angle ||
               fabsf(motor[i].position) > axis->protocol_limit) {
                return GIMBAL_CONTROL_LIMIT;
            }
        }
    }
    if(!measurement->valid || !isfinite(measurement->roll) || !isfinite(measurement->pitch) ||
       !isfinite(measurement->yaw) || (uint32_t)(now_us - measurement->timestamp_us) > config->imu_max_age_us ||
       (uint32_t)(now_us - measurement->acc_timestamp_us) > config->imu_max_age_us) {
        return GIMBAL_CONTROL_IMU_INVALID;
    }
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        if(!isfinite(measurement->gyro[i])) {
            return GIMBAL_CONTROL_IMU_INVALID;
        }
    }
    if(fabsf(cosf(measurement->pitch)) < config->min_cos_pitch) {
        return GIMBAL_CONTROL_SINGULAR;
    }
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalAxisConfig* axis = &config->axis[i];
        if(!motor[i].ready || motor[i].age_ms > axis->feedback_max_age_ms || !isfinite(motor[i].position)) {
            return GIMBAL_CONTROL_MOTOR_INVALID;
        }
    }
    GimbalKinematics model;
    if(!gimbal_control_kinematics(config, measurement, motor, now_us, &model)) {
        return GIMBAL_CONTROL_SINGULAR;
    }
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        if((i == GIMBAL_AXIS_YAW || !config->coupled_tilt) &&
           (!isfinite(model.angle_jacobian[i][i]) || fabsf(model.angle_jacobian[i][i]) < config->min_axis_response)) {
            return GIMBAL_CONTROL_SINGULAR;
        }
    }
    if(config->coupled_tilt) {
        float determinant = model.angle_jacobian[1][1] * model.angle_jacobian[2][2] -
                            model.angle_jacobian[1][2] * model.angle_jacobian[2][1];
        if(!isfinite(determinant) || fabsf(determinant) < config->min_jacobian_det) {
            return GIMBAL_CONTROL_SINGULAR;
        }
    }
    return GIMBAL_CONTROL_OK;
}

GimbalControlStatus gimbal_control_start(GimbalControl* state, const GimbalControlConfig* config,
                                         const GimbalMeasurement* measurement, const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT], uint32_t now_us) {
    if(state == NULL) {
        return GIMBAL_CONTROL_INVALID_PARAM;
    }
    GimbalControlStatus status = gimbal_control_check(config, measurement, motor, now_us);
    if(status != GIMBAL_CONTROL_OK) {
        return status;
    }
    memset(state, 0, sizeof(*state));
    const float target[GIMBAL_AXIS_COUNT] = { measurement->yaw, measurement->roll, measurement->pitch };
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        state->axis[i].target = target[i];
        state->axis[i].command = motor[i].position;
        state->axis[i].filtered_goal = motor[i].position;
        state->axis[i].last_us = now_us;
        state->axis[i].sequence = measurement->sequence;
    }
    state->running = true;
    return GIMBAL_CONTROL_OK;
}

GimbalControlStatus gimbal_control_update(GimbalControl* state, const GimbalControlConfig* config,
                                          const GimbalMeasurement* measurement, const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT],
                                          uint32_t now_us, GimbalMotorCommand command[GIMBAL_AXIS_COUNT]) {
    if(state == NULL || command == NULL || !state->running) {
        return GIMBAL_CONTROL_INVALID_PARAM;
    }
    memset(command, 0, sizeof(*command) * GIMBAL_AXIS_COUNT);
    GimbalControlStatus status = gimbal_control_check(config, measurement, motor, now_us);
    if(status != GIMBAL_CONTROL_OK) {
        state->running = false;
        return status;
    }
    gimbal_control_kinematics(config, measurement, motor, now_us, &state->kinematics);
    const float angle[GIMBAL_AXIS_COUNT] = { measurement->yaw, measurement->roll, measurement->pitch };
    float correction[GIMBAL_AXIS_COUNT];
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        GimbalAxisState* axis = &state->axis[i];
        axis->error = gimbal_wrap(axis->target - angle[i]);
        axis->rate = state->kinematics.angle_rate[i];
        correction[i] = gimbal_error_correction(&config->axis[i], axis->error, axis->rate);
    }
    float (*jacobian)[GIMBAL_AXIS_COUNT] = state->kinematics.angle_jacobian;
    if(config->coupled_tilt) {
        float determinant = jacobian[1][1] * jacobian[2][2] - jacobian[1][2] * jacobian[2][1];
        if(!isfinite(determinant) || fabsf(determinant) < config->min_jacobian_det) {
            state->running = false;
            return GIMBAL_CONTROL_SINGULAR;
        }
        float roll = correction[1];
        float pitch = correction[2];
        correction[1] = (jacobian[2][2] * roll - jacobian[1][2] * pitch) / determinant;
        correction[2] = (-jacobian[2][1] * roll + jacobian[1][1] * pitch) / determinant;
    }
    else {
        correction[1] /= jacobian[1][1];
        correction[2] /= jacobian[2][2];
    }
    correction[0] /= jacobian[0][0];
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        GimbalAxisState* axis = &state->axis[i];
        const GimbalAxisConfig* tuning = &config->axis[i];
        uint32_t elapsed = now_us - axis->last_us;
        if(elapsed > config->stall_timeout_us) {
            state->running = false;
            return GIMBAL_CONTROL_STALLED;
        }
        if(elapsed < tuning->period_us || axis->sequence == measurement->sequence) {
            continue;
        }
        if(i == GIMBAL_AXIS_YAW) {
            state->yaw_ff_position = 0.0f;
            if(state->kinematics.velocity_aligned && config->yaw_ff_gain > 0.0f) {
                float base_rate = axis->rate;
                for(unsigned j = 0; j < GIMBAL_AXIS_COUNT; ++j) {
                    base_rate -= jacobian[0][j] * motor[j].velocity;
                }
                float dt = elapsed * 1.0e-6f;
                float alpha = dt / (config->yaw_ff_filter_s + dt);
                state->yaw_base_rate += alpha * (base_rate - state->yaw_base_rate);
                if(fabsf(state->yaw_base_rate) >= config->yaw_ff_deadband) {
                    state->yaw_ff_position = -config->yaw_ff_gain * config->yaw_ff_lead_s *
                                             state->yaw_base_rate / jacobian[0][0];
                }
            }
            else {
                state->yaw_base_rate = 0.0f;
            }
            correction[i] += state->yaw_ff_position;
        }
        if(!isfinite(correction[i])) {
            state->running = false;
            return GIMBAL_CONTROL_SINGULAR;
        }
        float lower;
        float upper;
        gimbal_encoder_limits(tuning, &lower, &upper);
        float delta = gimbal_clamp(correction[i], -tuning->max_correction, tuning->max_correction);
        float goal = gimbal_clamp(motor[i].position + delta, lower, upper);
        float dt = (elapsed > tuning->max_step_us ? tuning->max_step_us : elapsed) * 1.0e-6f;
        float alpha = dt / (tuning->goal_filter_s + dt);
        axis->filtered_goal += alpha * (goal - axis->filtered_goal);
        float step = tuning->max_speed * dt;
        axis->command += gimbal_clamp(axis->filtered_goal - axis->command, -step, step);
        axis->command = gimbal_clamp(axis->command,
                                     fmaxf(lower, motor[i].position - tuning->max_correction),
                                     fminf(upper, motor[i].position + tuning->max_correction));
        axis->last_us = now_us;
        axis->sequence = measurement->sequence;
        command[i] = (GimbalMotorCommand){
            .position = axis->command,
            .speed = tuning->command_speed,
            .timeout_ms = tuning->command_timeout_ms,
            .updated = true,
        };
    }
    return GIMBAL_CONTROL_OK;
}

void gimbal_control_stop(GimbalControl* state) {
    if(state != NULL) {
        state->running = false;
    }
}

bool gimbal_control_kinematics(const GimbalControlConfig* config, const GimbalMeasurement* measurement,
                               const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT], uint32_t now_us, GimbalKinematics* result) {
    if(config == NULL || measurement == NULL || motor == NULL || result == NULL ||
       !isfinite(measurement->roll) || !isfinite(measurement->pitch) ||
       fabsf(cosf(measurement->pitch)) < config->min_cos_pitch) {
        return false;
    }
    memset(result, 0, sizeof(*result));
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalAxisConfig* axis = &config->axis[i];
        result->joint[i] = axis->direction * axis->ratio * (motor[i].position - axis->zero);
        if(!isfinite(result->joint[i])) {
            return false;
        }
    }
    float qr = result->joint[GIMBAL_AXIS_ROLL];
    float qp = result->joint[GIMBAL_AXIS_PITCH];
    /* R_BP = Rz(yaw) Rx(roll) Ry(pitch)；关节轴列向量表达于载荷前左上坐标 */
    const float joint_axis[3][GIMBAL_AXIS_COUNT] = {
        { -sinf(qp) * cosf(qr), cosf(qp), 0.0f },
        { sinf(qr), 0.0f, 1.0f },
        { cosf(qp) * cosf(qr), sinf(qp), 0.0f },
    };
    float sr = sinf(measurement->roll);
    float cr = cosf(measurement->roll);
    float cp = cosf(measurement->pitch);
    float tp = tanf(measurement->pitch);
    result->velocity_aligned = true;
    uint32_t imu_age = now_us - measurement->timestamp_us;
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        float scale = config->axis[i].direction * config->axis[i].ratio;
        for(unsigned row = 0; row < 3; ++row) {
            result->body_jacobian[row][i] = joint_axis[row][i] * scale;
        }
        float x = result->body_jacobian[0][i];
        float y = result->body_jacobian[1][i];
        float z = result->body_jacobian[2][i];
        result->angle_jacobian[0][i] = (sr * y + cr * z) / cp;
        result->angle_jacobian[1][i] = x + tp * (sr * y + cr * z);
        result->angle_jacobian[2][i] = cr * y - sr * z;
        uint64_t motor_age = (uint64_t)motor[i].age_ms * 1000u;
        uint64_t skew = motor_age > imu_age ? motor_age - imu_age : imu_age - motor_age;
        if(!motor[i].ready || !motor[i].velocity_valid || !isfinite(motor[i].velocity) ||
           motor[i].age_ms > config->axis[i].feedback_max_age_ms || skew > config->fusion_max_skew_us) {
            result->velocity_aligned = false;
        }
    }
    result->angle_rate[0] = (sr * measurement->gyro[1] + cr * measurement->gyro[2]) / cp;
    result->angle_rate[1] = measurement->gyro[0] + tp * (sr * measurement->gyro[1] + cr * measurement->gyro[2]);
    result->angle_rate[2] = cr * measurement->gyro[1] - sr * measurement->gyro[2];
    if(result->velocity_aligned) {
        for(unsigned row = 0; row < 3; ++row) {
            result->base_rate[row] = measurement->gyro[row];
            for(unsigned col = 0; col < GIMBAL_AXIS_COUNT; ++col) {
                result->base_rate[row] -= result->body_jacobian[row][col] * motor[col].velocity;
            }
        }
    }
    return true;
}

const char* gimbal_control_status_str(GimbalControlStatus status) {
    switch(status) {
        case GIMBAL_CONTROL_OK:
            return "Ready";
        case GIMBAL_CONTROL_INVALID_PARAM:
            return "Invalid config/input";
        case GIMBAL_CONTROL_UNCONFIRMED:
            return "Coordinates not confirmed";
        case GIMBAL_CONTROL_IMU_INVALID:
            return "IMU not ready/stale";
        case GIMBAL_CONTROL_MOTOR_INVALID:
            return "Motor feedback not ready/stale";
        case GIMBAL_CONTROL_LIMIT:
            return "Mechanical/protocol limit";
        case GIMBAL_CONTROL_SINGULAR:
            return "Geometry singular";
        case GIMBAL_CONTROL_STALLED:
            return "Control loop stalled";
        default:
            return "Unknown";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static float gimbal_clamp(float value, float lower, float upper) {
    return fminf(fmaxf(value, lower), upper);
}

static float gimbal_wrap(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

static bool gimbal_config_valid(const GimbalControlConfig* config) {
    if(config == NULL || !isfinite(config->min_cos_pitch) || config->min_cos_pitch <= 0.0f ||
       config->min_cos_pitch > 1.0f || !isfinite(config->min_axis_response) || config->min_axis_response <= 0.0f ||
       !isfinite(config->min_jacobian_det) || config->min_jacobian_det <= 0.0f || config->imu_max_age_us == 0u ||
       config->stall_timeout_us == 0u || !isfinite(config->yaw_ff_gain) || config->yaw_ff_gain < 0.0f ||
       !isfinite(config->yaw_ff_lead_s) || config->yaw_ff_lead_s < 0.0f ||
       !isfinite(config->yaw_ff_filter_s) || config->yaw_ff_filter_s < 0.0f ||
       !isfinite(config->yaw_ff_deadband) || config->yaw_ff_deadband < 0.0f) {
        return false;
    }
    for(unsigned i = 0; i < GIMBAL_AXIS_COUNT; ++i) {
        const GimbalAxisConfig* axis = &config->axis[i];
        if((axis->direction != 1.0f && axis->direction != -1.0f) || !isfinite(axis->ratio) || axis->ratio <= 0.0f ||
           !isfinite(axis->zero) || !isfinite(axis->min_angle) || !isfinite(axis->max_angle) || axis->min_angle >= axis->max_angle ||
           !isfinite(axis->protocol_limit) || axis->protocol_limit <= 0.0f || !isfinite(axis->kp) || axis->kp < 0.0f ||
           !isfinite(axis->kd) || axis->kd < 0.0f || !isfinite(axis->deadband) || axis->deadband < 0.0f ||
           !isfinite(axis->error_power) || axis->error_power < 1.0f || !isfinite(axis->error_reference) || axis->error_reference <= 0.0f ||
           !isfinite(axis->max_correction) || axis->max_correction <= 0.0f || !isfinite(axis->max_speed) || axis->max_speed <= 0.0f ||
           !isfinite(axis->command_speed) || axis->command_speed <= 0.0f || !isfinite(axis->goal_filter_s) || axis->goal_filter_s < 0.0f ||
           axis->period_us == 0u || axis->max_step_us < axis->period_us || axis->feedback_max_age_ms == 0u ||
           axis->command_timeout_ms == 0u || axis->period_us > config->stall_timeout_us) {
            return false;
        }
        float lower;
        float upper;
        gimbal_encoder_limits(axis, &lower, &upper);
        if(!isfinite(lower) || !isfinite(upper) || lower >= upper) {
            return false;
        }
    }
    return true;
}

static void gimbal_encoder_limits(const GimbalAxisConfig* config, float* lower, float* upper) {
    float scale = config->direction * config->ratio;
    float a = config->zero + config->min_angle / scale;
    float b = config->zero + config->max_angle / scale;
    *lower = fmaxf(fminf(a, b), -config->protocol_limit);
    *upper = fminf(fmaxf(a, b), config->protocol_limit);
}

static float gimbal_error_correction(const GimbalAxisConfig* config, float error, float rate) {
    float e = fabsf(error) <= config->deadband ? 0.0f : error - copysignf(config->deadband, error);
    float magnitude = config->error_reference * powf(fabsf(e) / config->error_reference, config->error_power);
    return config->kp * copysignf(magnitude, e) - config->kd * rate;
}
