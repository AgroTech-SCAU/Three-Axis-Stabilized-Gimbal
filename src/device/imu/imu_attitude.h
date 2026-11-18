#ifndef _imu_attitude_h_
#define _imu_attitude_h_

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 姿态融合算法状态码
 */
typedef enum {
    IMU_ATTITUDE_STATUS_OK = 0,        /**< 操作成功 */
    IMU_ATTITUDE_STATUS_ERROR,         /**< 通用错误 */
    IMU_ATTITUDE_STATUS_INVALID_PARAM, /**< 参数无效 */
    IMU_ATTITUDE_STATUS_CALIBRATING,   /**< 正在进行陀螺零偏校准 */
    IMU_ATTITUDE_STATUS_NOT_READY,     /**< 尚未得到可用姿态 */
} ImuAttitudeStatus;

/**
 * @brief 姿态四元数
 */
typedef struct {
    float w; /**< 实部 */
    float x; /**< x 虚部 */
    float y; /**< y 虚部 */
    float z; /**< z 虚部 */
} ImuQuat;

/**
 * @brief 姿态融合状态，由调用方持有、算法维护
 */
typedef struct {
    ImuAttitudeConfig config; /**< 算法配置 */
    ImuQuat quat;             /**< 当前姿态四元数 */
    ImuAngle angle;           /**< 当前姿态角 */

    ImuGyro gyro_bias;           /**< 启动参考温度下的实测零偏 */
    ImuGyro gyro_bias_residual;  /**< 静止在线残余；不与启动零偏重复扣除 */
    ImuGyro gyro_bias_effective; /**< 当前实际扣除的启动零偏+温差补偿+残余 */
    ImuGyro gyro_temp_comp;      /**< 当前相对启动温度的三轴补偿 */
    ImuGyro gyro_bias_sum;       /**< 启动校准阶段的零偏累加值 */
    ImuGyro gyro_sq_sum;         /**< 启动校准阶段的角速度平方和，用于方差估计 */
    ImuGyro gyro_filtered;       /**< 最近一次纯去偏角速度；不包含姿态反馈 */

    ImuAcc acc_filtered;      /**< 最近一次用于融合的加速度 */
    float last_acc_norm;      /**< 最近一次参与可信度判断的加速度模长 */
    uint32_t last_acc_age_us; /**< 最近一次参与融合的加速度相对 gyro 的陈旧时间 */
    uint32_t last_update_us;  /**< 上一次更新的时间戳，单位 us */

    uint16_t calib_count; /**< 已累计的陀螺校准样本数 */
    bool calibrated;      /**< true 表示陀螺零偏校准完成 */
    bool has_angle;       /**< true 表示当前已有可读取姿态 */
    bool has_update_time; /**< 时间戳已初始化；零值也是有效的回绕时刻 */
    bool acc_trusted;     /**< true 表示最近一次融合时加速度被认为可信 */

    float gyro_temp_ref;         /**< 启动标定时的参考温度，单位 ℃ */
    float gyro_temp_sum;         /**< 启动校准阶段的温度累加值 */
    uint16_t gyro_temp_count;    /**< 启动校准阶段累计到的有效温度样本数 */
    bool gyro_temp_valid;        /**< true 表示 gyro_temp_ref 有效 */
    float gyro_z_temp_intercept; /**< z 轴 bias 线性模型截距 */
    float gyro_z_bias_effective; /**< 当前实际用于扣除的 z 轴 bias */

    uint32_t zru_static_time_us; /**< 连续满足静止条件的累计时长，单位 us */
    bool zru_enabled;            /**< true 表示允许执行静止 ZRU */
    bool zru_active;             /**< true 表示本次更新已触发静止 ZRU */
    bool stationary;             /**< 上层已确认的静止窗口；不由目标姿态是否改变推断 */
} ImuAttitude;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化姿态融合算法实例
 * @param attitude 算法状态对象
 * @param config 算法配置
 * @return 姿态融合状态码
 */
ImuAttitudeStatus imu_attitude_init(ImuAttitude* attitude, const ImuAttitudeConfig* config);

/**
 * @brief 获取六轴 Mahony 默认配置
 * @return 默认配置；温漂斜率仍需按实际传感器升温日志复核
 */
ImuAttitudeConfig imu_attitude_default_config(void);

/**
 * @brief 使用一帧 IMU 采样更新姿态
 * @param attitude 算法状态对象
 * @param sample IMU 采样数据；角速度单位 rad/s，加速度单位 m/s^2，时间戳单位 us
 * @return 姿态融合状态码；校准期间返回 `IMU_ATTITUDE_STATUS_CALIBRATING`
 */
ImuAttitudeStatus imu_attitude_update(ImuAttitude* attitude, const ImuSample* sample);

/**
 * @brief 设置外部静止许可；撤销时冻结已校准的残余零偏并清除未完成的启动标定
 * @param attitude 算法实例
 * @param stationary 已通过持续统计窗口的静止状态
 */
void imu_attitude_set_stationary(ImuAttitude* attitude, bool stationary);

/**
 * @brief 获取当前姿态角（带 bias 补偿）
 * @param attitude 算法状态对象
 * @param angle 输出姿态角
 * @return 姿态融合状态码
 */
ImuAttitudeStatus imu_attitude_get_angle(const ImuAttitude* attitude, ImuAngle* angle);

/**
 * @brief 获取当前姿态四元数
 * @param attitude 算法状态对象
 * @param quat 输出四元数
 * @return 姿态融合状态码
 */
ImuAttitudeStatus imu_attitude_get_quat(const ImuAttitude* attitude, ImuQuat* quat);

/**
 * @brief 获取姿态融合估计的陀螺零偏
 * @param attitude 算法状态对象
 * @param gyro_bias 输出陀螺零偏，单位 rad/s
 * @return 姿态融合状态码
 */
ImuAttitudeStatus imu_attitude_get_gyro_bias(const ImuAttitude* attitude, ImuGyro* gyro_bias);

/**
 * @brief 获取最近一次扣除零偏和温漂后的纯测量角速度
 * @param attitude 算法状态对象
 * @param gyro_corrected 输出角速度，单位 rad/s
 * @return 姿态融合状态码
 */
ImuAttitudeStatus imu_attitude_get_gyro_corrected(const ImuAttitude* attitude, ImuGyro* gyro_corrected);

/**
 * @brief 重置当前 yaw 角
 * @param attitude 算法状态对象
 * @param yaw 新的 yaw 角，单位 rad
 * @return 姿态融合状态码
 */
ImuAttitudeStatus imu_attitude_reset_yaw(ImuAttitude* attitude, float yaw);

#endif
