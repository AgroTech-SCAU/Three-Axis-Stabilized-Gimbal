#ifndef _gimbal_control_h_
#define _gimbal_control_h_

/**
 * @file gimbal_control.h
 * @brief Z-X-Y 云台运动学与位置外环；不依赖设备协议或 HAL
 * @details IMU 提供载荷姿态，编码器用于关节映射、限位、Jacobian 和速度前馈
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/** @brief 机械串联顺序：外偏航、中横滚、内俯仰 */
typedef enum {
    GIMBAL_AXIS_YAW = 0,
    GIMBAL_AXIS_ROLL,
    GIMBAL_AXIS_PITCH,
    GIMBAL_AXIS_COUNT,
} GimbalAxis;

typedef enum {
    GIMBAL_CONTROL_OK = 0,
    GIMBAL_CONTROL_INVALID_PARAM,
    GIMBAL_CONTROL_UNCONFIRMED,
    GIMBAL_CONTROL_IMU_INVALID,
    GIMBAL_CONTROL_MOTOR_INVALID,
    GIMBAL_CONTROL_LIMIT,
    GIMBAL_CONTROL_SINGULAR,
    GIMBAL_CONTROL_STALLED,
} GimbalControlStatus;

/** @brief 前左上机体系下的姿态；角度 rad，物理角速度 rad/s */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro[3];
    uint32_t timestamp_us;
    uint32_t acc_timestamp_us;
    uint32_t sequence;
    bool valid;
} GimbalMeasurement;

/** @brief 原驱动反馈，不在驱动中改变方向、零位或单位 */
typedef struct {
    float position;
    float velocity;
    uint32_t age_ms;
    bool ready;
    bool velocity_valid;
} GimbalMotorFeedback;

/** @brief 输出编码器单位的位置目标，通过既有驱动接口提交 */
typedef struct {
    float position;
    float speed;
    uint32_t timeout_ms;
    bool updated;
} GimbalMotorCommand;

typedef struct {
    float direction; /**< +1 或 -1；正关节角按前左上右手定则 */
    float ratio;     /**< 关节 rad / 驱动反馈 rad；反馈已是输出轴时为 1 */
    float zero;      /**< 可重复机械中位对应的原始编码器读数 */
    float min_angle;
    float max_angle;
    float protocol_limit;
    float kp;
    float kd;
    float deadband;
    float error_power;
    float error_reference;
    float max_correction;
    float max_speed;
    float command_speed;
    float goal_filter_s;
    uint32_t period_us;
    uint32_t max_step_us;
    uint32_t feedback_max_age_ms;
    uint32_t command_timeout_ms;
} GimbalAxisConfig;

typedef struct {
    GimbalAxisConfig axis[GIMBAL_AXIS_COUNT];
    bool coordinates_confirmed;
    bool coupled_tilt;
    float min_cos_pitch;
    float min_axis_response;
    float min_jacobian_det;
    uint32_t imu_max_age_us;
    uint32_t stall_timeout_us;
    uint32_t fusion_max_skew_us;
    float yaw_ff_gain;
    float yaw_ff_lead_s;
    float yaw_ff_filter_s;
    float yaw_ff_deadband;
} GimbalControlConfig;

typedef struct {
    float joint[GIMBAL_AXIS_COUNT];
    float body_jacobian[3][GIMBAL_AXIS_COUNT]; /**< 行 x/y/z，列 yaw/roll/pitch */
    float angle_jacobian[GIMBAL_AXIS_COUNT][GIMBAL_AXIS_COUNT];
    float angle_rate[GIMBAL_AXIS_COUNT];
    float base_rate[3]; /**< 扣除关节运动后，表达于载荷机体系的底座角速度 */
    bool velocity_aligned;
} GimbalKinematics;

typedef struct {
    float target;
    float command;
    float filtered_goal;
    float error;
    float rate;
    uint32_t last_us;
    uint32_t sequence;
} GimbalAxisState;

typedef struct {
    GimbalAxisState axis[GIMBAL_AXIS_COUNT];
    GimbalKinematics kinematics;
    float yaw_base_rate;
    float yaw_ff_position;
    bool running;
} GimbalControl;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/** @brief 检查配置、数据时效、机械限位和当前姿态可控性 */
GimbalControlStatus gimbal_control_check(const GimbalControlConfig* config,
                                         const GimbalMeasurement* measurement, const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT], uint32_t now_us);

/** @brief 仅捕获本次保持目标，不改变机械零位或 IMU 标定 */
GimbalControlStatus gimbal_control_start(GimbalControl* state, const GimbalControlConfig* config,
                                         const GimbalMeasurement* measurement, const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT], uint32_t now_us);

/** @brief 每轮调用，故障检查不受控制周期限制；updated 表示需要提交目标 */
GimbalControlStatus gimbal_control_update(GimbalControl* state, const GimbalControlConfig* config,
                                          const GimbalMeasurement* measurement, const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT],
                                          uint32_t now_us, GimbalMotorCommand command[GIMBAL_AXIS_COUNT]);

void gimbal_control_stop(GimbalControl* state);
bool gimbal_control_kinematics(const GimbalControlConfig* config, const GimbalMeasurement* measurement,
                               const GimbalMotorFeedback motor[GIMBAL_AXIS_COUNT], uint32_t now_us, GimbalKinematics* result);
const char* gimbal_control_status_str(GimbalControlStatus status);

#endif
