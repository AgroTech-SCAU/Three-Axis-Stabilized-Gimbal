#ifndef _ht_motor_h_
#define _ht_motor_h_

#include "bus_motor.h"

/**
 * @file ht_motor.h
 * @brief 高擎 Classic CAN v2.x 总线电机驱动接口
 *
 * 统一对外单位：位置 rad，速度 rad/s，力矩 N*m。
 * 总线侧严格使用 Classical CAN，单帧数据长度不超过 8 字节。
 */

#define HT_MOTOR_MAX_ID 30u
#define HT_MOTOR_MAX_FRAME_LEN 8u

/** 高擎电机运行状态/模式编号（与寄存器模式编号保持兼容）。 */
typedef enum {
    HT_MOTOR_MODE_STOP = 0u,
    HT_MOTOR_MODE_ERROR = 1u,
    HT_MOTOR_MODE_READY_2 = 2u,
    HT_MOTOR_MODE_READY_3 = 3u,
    HT_MOTOR_MODE_READY_4 = 4u,
    HT_MOTOR_MODE_PWM = 5u,
    HT_MOTOR_MODE_VOLTAGE = 6u,
    HT_MOTOR_MODE_FOC_VOLTAGE = 7u,
    HT_MOTOR_MODE_DQ_VOLTAGE = 8u,
    HT_MOTOR_MODE_DQ_CURRENT = 9u,
    HT_MOTOR_MODE_POSITION = 10u,
    HT_MOTOR_MODE_TIMEOUT = 11u,
    HT_MOTOR_MODE_ZERO_SPEED = 12u,
    HT_MOTOR_MODE_RANGE = 13u,
    HT_MOTOR_MODE_MEASURE_L = 14u,
    HT_MOTOR_MODE_BRAKE = 15u,
} HtMotorMode;

/** 当前云台实际使用的高擎型号。 */
typedef enum {
    HT_MOTOR_MODEL_UNKNOWN = 0u,
    HT_MOTOR_MODEL_4438_30,
    HT_MOTOR_MODEL_5047_36,
} HtMotorModel;

/** 单台电机型号配置，用于 Classic CAN int16 力矩修正。 */
typedef struct {
    uint16_t id;
    HtMotorModel model;
} HtMotorDeviceProfile;

/** 通过 BusMotorConfig.driver_config 传入。 */
typedef struct {
    const HtMotorDeviceProfile* profiles;
    uint8_t count;
} HtMotorDriverConfig;

/**
 * 兼容旧 FDCAN 接口保留的原始组控结构。
 * Classic CAN v2 当前云台路径不使用组控；相应 API 返回 UNSUPPORTED。
 */
typedef struct {
    int16_t position;
    int16_t speed;
    int16_t torque;
} HtMotorGroupPvtRaw;

typedef struct {
    int16_t position;
    int16_t speed;
    int16_t torque;
    int16_t kp;
    int16_t kd;
} HtMotorGroupMitRaw;

extern const LegacyBusMotorInterface ht_motor_instance;

/**
 * @brief 解析 Classic CAN 状态回复。
 *
 * 当前运行链查询 0x01~0x03 三个 int16 寄存器，对应位置、速度、力矩。
 * @param frame_id 回复 CAN ID（ID 1~7 通常为标准帧，ID >=8 为扩展帧）
 * @param data 数据段
 * @param len 数据长度，Classic CAN 最大 8
 * @param feedback 可选输出
 */
BusMotorStatus ht_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t* data,
    uint8_t len,
    BusMotorFeedback* feedback);

/** @brief 主动查询位置、速度、力矩。 */
BusMotorStatus ht_motor_request_feedback(uint16_t id);

/**
 * @brief 兼容旧接口。Classic CAN v2 的 MIT 控制需要独立协议封装，
 * 当前三轴云台未使用该 API，因此返回 MOTOR_STATUS_UNSUPPORTED。
 */
BusMotorStatus ht_motor_set_motion(uint16_t id,
    float position,
    float speed,
    float torque,
    float kp,
    float kd);

/** Classic CAN 当前不使用旧 FDCAN 一拖多帧格式。 */
BusMotorStatus ht_motor_group_set_pvt_raw(uint16_t first_id,
    const HtMotorGroupPvtRaw* commands,
    uint8_t count,
    uint8_t query_id);

/** Classic CAN 当前不使用旧 FDCAN 一拖多帧格式。 */
BusMotorStatus ht_motor_group_set_motion_raw(uint16_t first_id,
    const HtMotorGroupMitRaw* commands,
    uint8_t count,
    uint8_t query_id);

#endif
