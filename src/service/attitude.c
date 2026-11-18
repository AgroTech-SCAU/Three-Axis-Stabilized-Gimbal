#include "imu/bmi088.h"
#include "attitude.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 静止检测器；只统计真实的新 gyro/acc 样本
 */
typedef struct {
    ImuGyro gyro_mean;
    ImuAcc acc_mean;
    float candidate_s;
    float acc_elapsed_s;
    float acc_weight_s;
    float acc_variance;
    uint32_t acc_samples;
    uint32_t last_acc_us;
    bool gyro_mean_ready;
    bool acc_time_ready;
} AttitudeStaticWindow;

static AttitudeState s_state;
static AttitudeConfig s_config;
static ImuAttitude s_filter;
static AttitudeStaticWindow s_static;
static uint32_t s_last_sample_us;
static bool s_sample_time_ready;
static bool s_snapshot_ready;

#define X(name, str) .name = ATTITUDE_##name,
const struct AttitudeInterface attitude_interface = {
    { ATTITUDE_STATUS_TABLE },
    .default_config = attitude_default_config,
    .init = attitude_init,
    .set_bias_learning = attitude_set_bias_learning,
    .process = attitude_process,
    .get_snapshot = attitude_get_snapshot,
    .get_state = attitude_get_state,
    .status_str = attitude_status_str,
    .reset_reason_str = attitude_reset_reason_str,
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool attitude_config_valid(const AttitudeConfig* config);
static bool attitude_timestamp_fresh(uint32_t timestamp_us, uint32_t now_us);
static AttitudeResetReason attitude_sample_problem(const ImuSample* sample, uint32_t now_us);
static bool attitude_sample_fresh(const ImuSample* sample, uint32_t now_us);
static bool attitude_gyro_finite(ImuGyro gyro);
static float attitude_gyro_norm(ImuGyro gyro);
static void attitude_reset_static_window(bool clear_gyro_mean, AttitudeResetReason reason);
static bool attitude_update_static_window(const ImuSample* sample, float dt_s, AttitudeResetReason problem);
static ImuGyro attitude_sample_unbiased(const ImuSample* sample);
static void attitude_invalidate(bool discontinuity, AttitudeResetReason reason);
static void attitude_publish(const Bmi088Snapshot* raw, ImuAttitudeStatus status, bool fresh);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

AttitudeConfig attitude_default_config(void) {
    return (AttitudeConfig){
        .filter = imu_attitude_default_config(),
        .max_sample_age_us = 10000U,
        .max_sample_gap_us = 20000U,
        .max_samples_per_process = 16U,
        .stationary_time_s = 0.5f,
        .startup_gyro_threshold = 0.05f,
        .gyro_static_threshold = 0.020f,
        .gyro_mean_static_threshold = 0.003f,
        .gyro_mean_time_constant_s = 0.20f,
        .acc_norm_tolerance = 0.55f,
        .acc_rms_threshold = 0.35f,
    };
}

AttitudeStatus attitude_init(const AttitudeConfig* config) {
    AttitudeConfig selected = config != NULL ? *config : attitude_default_config();
    if(!attitude_config_valid(&selected)) {
        return ATTITUDE_INVALID_PARAM;
    }
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_static, 0, sizeof(s_static));
    s_config = selected;
    s_sample_time_ready = false;
    s_snapshot_ready = false;
    s_last_sample_us = 0U;
    if(imu_attitude_init(&s_filter, &s_config.filter) != IMU_ATTITUDE_STATUS_OK) {
        return ATTITUDE_INVALID_PARAM;
    }
    s_state.initialized = true;
    s_state.calibrated = s_filter.calibrated;
    s_state.calibration_target = selected.filter.gyro_calib_samples;
    return ATTITUDE_OK;
}

void attitude_set_bias_learning(bool allowed) {
    s_state.bias_learning_allowed = allowed;
    if(!allowed) {
        attitude_reset_static_window(true, ATTITUDE_RESET_LEARNING_DISABLED);
        imu_attitude_set_stationary(&s_filter, false);
        s_state.calibration_samples = s_filter.calib_count;
    }
}

AttitudeStatus attitude_process(uint32_t now_us) {
    bool consumed = false;
    if(!s_state.initialized) {
        return ATTITUDE_NOT_INITIALIZED;
    }
    for(uint32_t i = 0U; i < s_config.max_samples_per_process; ++i) {
        ImuStatus acquisition = imu_update();
        if(acquisition == IMU_STATUS_NOT_READY) {
            /* 队列空时保留未过期的静止窗口 */
            break;
        }
        if(acquisition != IMU_STATUS_OK) {
            attitude_invalidate(true, ATTITUDE_RESET_ACQUISITION_ERROR);
            return ATTITUDE_ERROR;
        }
        Bmi088Snapshot raw;
        if(!bmi088_get_snapshot(&raw)) {
            attitude_invalidate(true, ATTITUDE_RESET_ACQUISITION_ERROR);
            return ATTITUDE_ERROR;
        }
        consumed = true;
        ++s_state.processed_samples;
        bool gyro_valid = (raw.sample.flags & (IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_GYRO_NEW)) ==
                              (IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_GYRO_NEW) &&
                          attitude_gyro_finite(raw.sample.gyro);
        uint32_t elapsed_us = s_sample_time_ready ? raw.sample.gyro_timestamp_us - s_last_sample_us : 0U;
        bool continuous = !s_sample_time_ready ||
                          (elapsed_us > 0U && elapsed_us <= s_config.max_sample_gap_us);
        if(!gyro_valid || !continuous) {
            ++s_state.rejected_samples;
            attitude_invalidate(true, gyro_valid ? ATTITUDE_RESET_SAMPLE_GAP : ATTITUDE_RESET_INVALID_SAMPLE);
        }
        AttitudeResetReason problem = attitude_sample_problem(&raw.sample, now_us);
        bool fresh = problem == ATTITUDE_RESET_NONE;
        float dt_s = continuous ? (float)elapsed_us * 1.0e-6f : 0.0f;
        bool stationary = gyro_valid && continuous &&
                          attitude_update_static_window(&raw.sample, dt_s, problem);
        if(!stationary) {
            s_state.stationary = false;
        }
        imu_attitude_set_stationary(&s_filter, stationary);
        uint16_t previous_count = s_filter.calib_count;
        bool was_calibrated = s_filter.calibrated;
        ImuAttitudeStatus status = imu_attitude_update(&s_filter, &raw.sample);
        if(!was_calibrated && s_filter.calibrated && s_static.gyro_mean_ready) {
            /* 用本帧有效零偏平移 gyro 均值，保留静止时长与 acc 窗口
             * 避免校准前后基准不同导致伪复位 */
            ImuGyro corrected = attitude_sample_unbiased(&raw.sample);
            s_static.gyro_mean.x -= raw.sample.gyro.x - corrected.x;
            s_static.gyro_mean.y -= raw.sample.gyro.y - corrected.y;
            s_static.gyro_mean.z -= raw.sample.gyro.z - corrected.z;
        }
        if(stationary && !s_filter.calibrated && status == IMU_ATTITUDE_STATUS_CALIBRATING && s_filter.calib_count == 0U) {
            /* 温度、acc 时效或 gyro 方差不合格属于标定拒绝，不是采集故障 */
            if(previous_count > 0U) {
                ++s_state.static_reset_count;
            }
            s_state.last_reset_reason = ATTITUDE_RESET_CALIBRATION_REJECTED;
        }
        /* 按原时间戳处理积压样本，控制只读取最新有效结果 */
        attitude_publish(&raw, status, fresh && gyro_valid && continuous);
        s_last_sample_us = raw.sample.gyro_timestamp_us;
        s_sample_time_ready = gyro_valid;
    }
    if(s_snapshot_ready && !attitude_sample_fresh(&s_state.snapshot.sample, now_us)) {
        attitude_invalidate(s_sample_time_ready, attitude_sample_problem(&s_state.snapshot.sample, now_us));
    }
    return consumed ? ATTITUDE_OK : ATTITUDE_NOT_READY;
}

bool attitude_get_snapshot(AttitudeSnapshot* snapshot) {
    if(snapshot == NULL || !s_snapshot_ready) {
        return false;
    }
    *snapshot = s_state.snapshot;
    return true;
}

const AttitudeState* attitude_get_state(void) {
    return &s_state;
}

const char* attitude_status_str(AttitudeStatus status) {
    switch(status) {
#define X(name, str)      \
    case ATTITUDE_##name: \
        return str;
        ATTITUDE_STATUS_TABLE
#undef X
        default:
            return "UNKNOWN";
    }
}

const char* attitude_reset_reason_str(AttitudeResetReason reason) {
    switch(reason) {
#define X(name, str)            \
    case ATTITUDE_RESET_##name: \
        return str;
        ATTITUDE_RESET_REASON_TABLE
#undef X
        default:
            return "UNKNOWN";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool attitude_config_valid(const AttitudeConfig* config) {
    return config != NULL && config->filter.mode == IMU_ATTITUDE_MAHONY_6AXIS &&
           config->filter.mahony_ki == 0.0f && config->filter.mahony_ki_z == 0.0f &&
           config->max_sample_age_us > 0U && config->max_sample_age_us <= 100000U &&
           config->max_sample_gap_us > 0U &&
           config->max_sample_gap_us <= 100000U &&
           config->max_samples_per_process > 0U && config->max_samples_per_process <= 32U &&
           isfinite(config->filter.acc_norm) && config->filter.acc_norm > 0.0f &&
           isfinite(config->filter.acc_norm_tolerance) && config->filter.acc_norm_tolerance >= 0.0f &&
           isfinite(config->filter.gyro_calib_var_threshold) && config->filter.gyro_calib_var_threshold >= 0.0f &&
           isfinite(config->filter.mahony_kp) && config->filter.mahony_kp >= 0.0f &&
           isfinite(config->filter.gyro_x_temp_coeff) && isfinite(config->filter.gyro_y_temp_coeff) &&
           isfinite(config->filter.gyro_z_temp_coeff) &&
           isfinite(config->filter.zru_gyro_threshold) && config->filter.zru_gyro_threshold >= 0.0f &&
           isfinite(config->filter.zru_bias_gain) && config->filter.zru_bias_gain >= 0.0f &&
           isfinite(config->stationary_time_s) && config->stationary_time_s > 0.0f &&
           isfinite(config->startup_gyro_threshold) && config->startup_gyro_threshold > 0.0f &&
           isfinite(config->gyro_static_threshold) && config->gyro_static_threshold > 0.0f &&
           isfinite(config->gyro_mean_static_threshold) && config->gyro_mean_static_threshold > 0.0f &&
           isfinite(config->gyro_mean_time_constant_s) && config->gyro_mean_time_constant_s > 0.0f &&
           isfinite(config->acc_norm_tolerance) && config->acc_norm_tolerance > 0.0f &&
           isfinite(config->acc_rms_threshold) && config->acc_rms_threshold > 0.0f;
}

static bool attitude_gyro_finite(ImuGyro gyro) {
    return isfinite(gyro.x) && isfinite(gyro.y) && isfinite(gyro.z);
}

static float attitude_gyro_norm(ImuGyro gyro) {
    return sqrtf(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
}

static bool attitude_timestamp_fresh(uint32_t timestamp_us, uint32_t now_us) {
    /* DMA 样本可略晚于入口 now_us；双向无符号差处理有界领先与 32 位回绕 */
    return (uint32_t)(now_us - timestamp_us) <= s_config.max_sample_age_us ||
           (uint32_t)(timestamp_us - now_us) <= s_config.max_sample_age_us;
}

static AttitudeResetReason attitude_sample_problem(const ImuSample* sample, uint32_t now_us) {
    if(sample == NULL ||
       (sample->flags & (IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_ACC_VALID)) !=
           (IMU_SAMPLE_GYRO_VALID | IMU_SAMPLE_ACC_VALID) ||
       !attitude_gyro_finite(sample->gyro) ||
       !isfinite(sample->acc.x) || !isfinite(sample->acc.y) || !isfinite(sample->acc.z)) {
        return ATTITUDE_RESET_INVALID_SAMPLE;
    }
    if(!attitude_timestamp_fresh(sample->gyro_timestamp_us, now_us) ||
       !attitude_timestamp_fresh(sample->acc_timestamp_us, now_us)) {
        return ATTITUDE_RESET_SAMPLE_AGE;
    }
    return ATTITUDE_RESET_NONE;
}

static bool attitude_sample_fresh(const ImuSample* sample, uint32_t now_us) {
    return attitude_sample_problem(sample, now_us) == ATTITUDE_RESET_NONE;
}

static void attitude_reset_static_window(bool clear_gyro_mean, AttitudeResetReason reason) {
    if(s_static.candidate_s > 0.0f || s_static.acc_samples > 0U || s_state.stationary ||
       (!s_filter.calibrated && s_filter.calib_count > 0U)) {
        ++s_state.static_reset_count;
    }
    s_state.last_reset_reason = reason;
    s_state.stationary_candidate_s = 0.0f;
    ImuGyro mean = s_static.gyro_mean;
    bool mean_ready = s_static.gyro_mean_ready;
    memset(&s_static, 0, sizeof(s_static));
    if(!clear_gyro_mean) {
        s_static.gyro_mean = mean;
        s_static.gyro_mean_ready = mean_ready;
    }
    s_state.stationary = false;
    s_state.snapshot.stationary = false;
}

static ImuGyro attitude_sample_unbiased(const ImuSample* sample) {
    if(!s_filter.calibrated) {
        return sample->gyro;
    }
    ImuGyro temperature = s_filter.gyro_temp_comp;
    if(s_filter.gyro_temp_valid && (sample->flags & IMU_SAMPLE_TEMP_VALID) != 0U &&
       isfinite(sample->temperature)) {
        float delta_t = sample->temperature - s_filter.gyro_temp_ref;
        temperature = (ImuGyro){
            s_config.filter.gyro_x_temp_coeff * delta_t,
            s_config.filter.gyro_y_temp_coeff * delta_t,
            s_config.filter.gyro_z_temp_coeff * delta_t,
        };
    }
    return (ImuGyro){
        sample->gyro.x - s_filter.gyro_bias.x - temperature.x - s_filter.gyro_bias_residual.x,
        sample->gyro.y - s_filter.gyro_bias.y - temperature.y - s_filter.gyro_bias_residual.y,
        sample->gyro.z - s_filter.gyro_bias.z - temperature.z - s_filter.gyro_bias_residual.z,
    };
}

static bool attitude_update_static_window(const ImuSample* sample, float dt_s, AttitudeResetReason problem) {
    if(!s_state.bias_learning_allowed) {
        attitude_reset_static_window(true, ATTITUDE_RESET_LEARNING_DISABLED);
        return false;
    }
    if(problem != ATTITUDE_RESET_NONE) {
        attitude_reset_static_window(true, problem);
        return false;
    }
    ImuGyro gyro = attitude_sample_unbiased(sample);
    if(!s_static.gyro_mean_ready) {
        s_static.gyro_mean = gyro;
        s_static.gyro_mean_ready = true;
    }
    else {
        float alpha = 1.0f - expf(-dt_s / s_config.gyro_mean_time_constant_s);
        s_static.gyro_mean.x += alpha * (gyro.x - s_static.gyro_mean.x);
        s_static.gyro_mean.y += alpha * (gyro.y - s_static.gyro_mean.y);
        s_static.gyro_mean.z += alpha * (gyro.z - s_static.gyro_mean.z);
    }
    float instant_limit = s_filter.calibrated ? s_config.gyro_static_threshold : s_config.startup_gyro_threshold;
    float mean_limit = s_filter.calibrated ? s_config.gyro_mean_static_threshold : s_config.startup_gyro_threshold;
    float acc_norm = sqrtf(sample->acc.x * sample->acc.x + sample->acc.y * sample->acc.y + sample->acc.z * sample->acc.z);
    if(attitude_gyro_norm(gyro) > instant_limit) {
        attitude_reset_static_window(false, ATTITUDE_RESET_GYRO_MOTION);
        return false;
    }
    if(attitude_gyro_norm(s_static.gyro_mean) > mean_limit) {
        attitude_reset_static_window(false, ATTITUDE_RESET_GYRO_MEAN);
        return false;
    }
    if(fabsf(acc_norm - s_config.filter.acc_norm) > s_config.acc_norm_tolerance) {
        attitude_reset_static_window(false, ATTITUDE_RESET_ACC_NORM);
        return false;
    }
    s_static.candidate_s += dt_s;
    s_state.stationary_candidate_s = s_static.candidate_s;
    s_static.acc_elapsed_s += dt_s;
    bool new_acc = (sample->flags & IMU_SAMPLE_ACC_NEW) != 0U &&
                   (!s_static.acc_time_ready || sample->acc_timestamp_us != s_static.last_acc_us);
    if(new_acc) {
        bool first = s_static.acc_samples == 0U;
        float weight = s_static.acc_elapsed_s;
        if(weight <= 0.0f) {
            weight = 1.0e-6f;
        }
        s_static.acc_elapsed_s = 0.0f;
        s_static.last_acc_us = sample->acc_timestamp_us;
        s_static.acc_time_ready = true;
        s_static.acc_weight_s += weight;
        float alpha = s_state.stationary ? 1.0f - expf(-weight / s_config.stationary_time_s) : weight / s_static.acc_weight_s;
        if(first) {
            alpha = 1.0f;
        }
        float dx = sample->acc.x - s_static.acc_mean.x;
        float dy = sample->acc.y - s_static.acc_mean.y;
        float dz = sample->acc.z - s_static.acc_mean.z;
        s_static.acc_mean.x += alpha * dx;
        s_static.acc_mean.y += alpha * dy;
        s_static.acc_mean.z += alpha * dz;
        s_static.acc_variance = (1.0f - alpha) *
                                (s_static.acc_variance + alpha * (dx * dx + dy * dy + dz * dz));
        ++s_static.acc_samples;
        if(s_static.acc_variance > s_config.acc_rms_threshold * s_config.acc_rms_threshold) {
            attitude_reset_static_window(false, ATTITUDE_RESET_ACC_VARIANCE);
            return false;
        }
    }
    s_state.stationary = s_static.acc_samples >= 3U && s_static.candidate_s >= s_config.stationary_time_s;
    return s_state.stationary;
}

static void attitude_invalidate(bool discontinuity, AttitudeResetReason reason) {
    if(discontinuity) {
        ++s_state.discontinuity_count;
    }
    s_state.ready = false;
    s_state.snapshot.valid = false;
    attitude_reset_static_window(true, reason);
    imu_attitude_set_stationary(&s_filter, false);
    s_state.calibration_samples = s_filter.calib_count;
    s_sample_time_ready = false;
}

static void attitude_publish(const Bmi088Snapshot* raw, ImuAttitudeStatus status, bool fresh) {
    AttitudeSnapshot* snapshot = &s_state.snapshot;
    snapshot->sample = raw->sample;
    snapshot->sequence = raw->sequence;
    snapshot->angle = s_filter.angle;
    snapshot->quat = s_filter.quat;
    snapshot->gyro_corrected = s_filter.gyro_filtered;
    snapshot->gyro_bias_effective = s_filter.gyro_bias_effective;
    snapshot->gyro_bias_residual = s_filter.gyro_bias_residual;
    snapshot->calibrated = s_filter.calibrated;
    snapshot->stationary = s_state.stationary;
    snapshot->valid = fresh && s_filter.calibrated && status == IMU_ATTITUDE_STATUS_OK &&
                      isfinite(snapshot->angle.roll) && isfinite(snapshot->angle.pitch) && isfinite(snapshot->angle.yaw) &&
                      isfinite(snapshot->quat.w) && isfinite(snapshot->quat.x) &&
                      isfinite(snapshot->quat.y) && isfinite(snapshot->quat.z) &&
                      attitude_gyro_finite(snapshot->gyro_corrected) &&
                      attitude_gyro_finite(snapshot->gyro_bias_effective) && attitude_gyro_finite(snapshot->gyro_bias_residual);
    s_state.calibrated = snapshot->calibrated;
    s_state.ready = snapshot->valid;
    s_state.calibration_samples = s_filter.calib_count;
    s_snapshot_ready = true;
}
