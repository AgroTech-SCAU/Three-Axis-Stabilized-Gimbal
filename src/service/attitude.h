#ifndef _attitude_h_
#define _attitude_h_

#include <stdbool.h>
#include <stdint.h>

#include "imu/imu_attitude.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 姿态服务接口单例别名
 */
#define attitude attitude_interface

/**
 * @brief 姿态服务状态码表
 */
#define ATTITUDE_STATUS_TABLE                      \
    X(OK, "OK")                                    \
    X(INVALID_PARAM, "Invalid Parameter")          \
    X(NOT_INITIALIZED, "Attitude Not Initialized") \
    X(NOT_READY, "No New IMU Sample")              \
    X(ERROR, "IMU Acquisition Error")

#define X(name, str) ATTITUDE_##name,
/**
 * @brief 姿态服务状态码
 */
typedef enum {
    ATTITUDE_STATUS_TABLE
} AttitudeStatus;
#undef X

/**
 * @brief 静止候选清除或标定阻塞原因；用于低频汇总，不逐帧打印
 */
#define ATTITUDE_RESET_REASON_TABLE           \
    X(NONE, "NONE")                           \
    X(LEARNING_DISABLED, "LEARNING_DISABLED") \
    X(ACQUISITION_ERROR, "ACQUISITION_ERROR") \
    X(INVALID_SAMPLE, "INVALID_SAMPLE")       \
    X(SAMPLE_AGE, "SAMPLE_AGE")               \
    X(SAMPLE_GAP, "SAMPLE_GAP")               \
    X(GYRO_MOTION, "GYRO_MOTION")             \
    X(GYRO_MEAN, "GYRO_MEAN")                 \
    X(ACC_NORM, "ACC_NORM")                   \
    X(ACC_VARIANCE, "ACC_VARIANCE")           \
    X(CALIBRATION_REJECTED, "CALIBRATION_REJECTED")

#define X(name, str) ATTITUDE_RESET_##name,
typedef enum {
    ATTITUDE_RESET_REASON_TABLE
} AttitudeResetReason;
#undef X

/**
 * @brief 主循环消费与姿态算法配置，不改变 BMI088 采集频率
 */
typedef struct {
    ImuAttitudeConfig filter;         /**< 姿态算法配置；本版要求 mahony_ki/mahony_ki_z 均为零 */
    uint32_t max_sample_age_us;       /**< 控制快照允许的最大 gyro/acc 年龄，单位 us */
    uint32_t max_sample_gap_us;       /**< 静止统计允许的最大相邻 gyro 间隔，单位 us */
    uint32_t max_samples_per_process; /**< 每轮最多消费的原始 DMA 队列样本数 */
    float stationary_time_s;          /**< 静止持续统计窗口，单位 s */
    float startup_gyro_threshold;     /**< 未知零偏时使用的原始 gyro 宽门槛，单位 rad/s */
    float gyro_static_threshold;      /**< 校准后瞬时纯去偏 gyro 模长门槛，单位 rad/s */
    float gyro_mean_static_threshold; /**< 校准后 gyro 低通均值模长门槛，单位 rad/s */
    float gyro_mean_time_constant_s;  /**< gyro 均值低通时间常数，单位 s */
    float acc_norm_tolerance;         /**< 静止时加速度模长相对重力的容差，单位 m/s^2 */
    float acc_rms_threshold;          /**< 静止窗三轴加速度合成 RMS 门槛，单位 m/s^2 */
} AttitudeConfig;

/**
 * @brief 主循环拥有的姿态快照；机体系固定为 X 前、Y 左、Z 上
 * @details gyro_corrected 是纯去偏测量，不包含 Mahony 比例反馈；sample 保留原始标志和时间戳
 */
typedef struct {
    ImuSample sample;
    ImuAngle angle;
    ImuQuat quat;
    ImuGyro gyro_corrected;
    ImuGyro gyro_bias_effective;
    ImuGyro gyro_bias_residual;
    uint32_t sequence;
    bool calibrated;
    bool valid;
    bool stationary;
} AttitudeSnapshot;

/**
 * @brief 姿态服务只读状态
 */
typedef struct {
    AttitudeSnapshot snapshot;
    uint32_t processed_samples;
    uint32_t rejected_samples;
    uint32_t discontinuity_count;
    float stationary_candidate_s;          /**< 当前连续静止候选时长，单位 s；尚未达到静止门槛时也可观察 */
    uint32_t static_reset_count;           /**< 已有候选或未完成校准进度被丢弃的次数，不累计空窗口重复清除 */
    AttitudeResetReason last_reset_reason; /**< 最近清除/阻塞原因；恢复后保留，初始为 NONE */
    uint16_t calibration_samples;
    uint16_t calibration_target; /**< 当前配置实际要求的启动校准样本数 */
    bool initialized;
    bool calibrated;            /**< 启动零偏标定完成；不代表最近样本仍可用于控制 */
    bool ready;                 /**< 最近姿态已校准、成功更新且原始数据未超龄 */
    bool bias_learning_allowed; /**< 准备期学习许可；标定后还须有电机静止证据 */
    bool stationary;            /**< 内部持续 gyro/acc 统计通过；RUN 总为 false */
} AttitudeState;

/**
 * @brief 姿态服务接口表
 */
#define X(name, str) AttitudeStatus name;
extern const struct AttitudeInterface {
    struct {
        ATTITUDE_STATUS_TABLE
    };
    AttitudeConfig (*default_config)(void);
    AttitudeStatus (*init)(const AttitudeConfig* config);
    void (*set_bias_learning)(bool allowed);
    AttitudeStatus (*process)(uint32_t now_us);
    bool (*get_snapshot)(AttitudeSnapshot* snapshot);
    const AttitudeState* (*get_state)(void);
    const char* (*status_str)(AttitudeStatus status);
    const char* (*reset_reason_str)(AttitudeResetReason reason);
} attitude_interface;
#undef X

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 获取姿态服务默认配置
 * @return AttitudeConfig 默认配置
 */
AttitudeConfig attitude_default_config(void);

/**
 * @brief 初始化姿态服务，不重复初始化 BMI088 硬件
 * @param config 配置；NULL 使用默认值
 * @return 姿态服务状态码
 */
AttitudeStatus attitude_init(const AttitudeConfig* config);

/**
 * @brief 设置静止学习许可，撤销时冻结残余零偏
 * @param allowed 准备期允许启动标定；标定后还须有电机静止证据，RUN 为 false
 * @details 许可不等于静止，仍须通过 IMU 持续统计窗口
 */
void attitude_set_bias_learning(bool allowed);

/**
 * @brief 消费有限数量原始样本并更新唯一姿态估计
 * @param now_us 与 BMI088 样本同源的本轮入口微秒时间戳
 * @return OK 表示消费了新样本；NOT_READY 仅表示队列暂无新样本；ERROR 表示采集失败
 * @details 队列空保留标定累计，DMA 新帧允许有界领先入口时刻
 * 控制须重新读取时钟检查年龄，并检查 snapshot.valid 或 state.ready
 * 标定完成时用有效零偏平移 gyro 均值，保留连续静止统计
 */
AttitudeStatus attitude_process(uint32_t now_us);

/**
 * @brief 复制最近姿态快照，非消费式、限主循环调用
 * @param snapshot 输出快照
 * @return 已有原始样本时为 true；是否可控制由 snapshot.valid 决定
 */
bool attitude_get_snapshot(AttitudeSnapshot* snapshot);

/**
 * @brief 获取只读姿态状态，限主循环调用
 * @return 服务内部状态指针
 */
const AttitudeState* attitude_get_state(void);

/**
 * @brief 将状态码转换为静态字符串
 * @param status 服务状态码
 * @return 状态码说明
 */
const char* attitude_status_str(AttitudeStatus status);

/**
 * @brief 将最近静止清除/标定阻塞原因转换为静态字符串
 * @param reason 状态中的 last_reset_reason
 * @return 适合低频状态汇总的原因名称
 */
const char* attitude_reset_reason_str(AttitudeResetReason reason);

#endif
