#ifndef _ht_motor_h_
#define _ht_motor_h_

#include "bus_motor.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @file ht_motor.h
 * @brief 高擎 FDCAN 总线电机驱动接口
 */

/**
 * @brief 高擎电机支持的协议 ID 范围
 */
#define HT_MOTOR_MAX_ID 30u

/**
 * @brief 高擎 FDCAN 最大数据长度
 */
#define HT_MOTOR_MAX_FRAME_LEN 64u

/**
 * @brief 高擎电机运行模式
 */
typedef enum {
    HT_MOTOR_MODE_STOP = 0u,          /**< 停止并清除错误 */
    HT_MOTOR_MODE_ERROR = 1u,         /**< 错误状态 */
    HT_MOTOR_MODE_READY_2 = 2u,       /**< 准备运行 */
    HT_MOTOR_MODE_READY_3 = 3u,       /**< 准备运行 */
    HT_MOTOR_MODE_READY_4 = 4u,       /**< 准备运行 */
    HT_MOTOR_MODE_PWM = 5u,           /**< PWM 模式 */
    HT_MOTOR_MODE_VOLTAGE = 6u,       /**< 电压模式 */
    HT_MOTOR_MODE_FOC_VOLTAGE = 7u,   /**< FOC 电压模式 */
    HT_MOTOR_MODE_DQ_VOLTAGE = 8u,    /**< DQ 电压模式 */
    HT_MOTOR_MODE_DQ_CURRENT = 9u,    /**< DQ 电流模式 */
    HT_MOTOR_MODE_POSITION = 10u,     /**< 位置/运控模式 */
    HT_MOTOR_MODE_TIMEOUT = 11u,      /**< 超时模式 */
    HT_MOTOR_MODE_ZERO_SPEED = 12u,   /**< 零速模式 */
    HT_MOTOR_MODE_RANGE = 13u,        /**< 范围模式 */
    HT_MOTOR_MODE_MEASURE_L = 14u,    /**< 电感测量模式 */
    HT_MOTOR_MODE_BRAKE = 15u,        /**< 刹车模式 */
} HtMotorMode;

/**
 * @brief 一拖多模式中的原始 int16 控制量
 *
 * 高擎一拖多协议固定使用 int16。由于不同固件/电机型号的量程映射可能不同，
 * 此结构直接保存协议原始值，避免驱动层错误假定缩放系数。
 */
typedef struct {
    int16_t position;
    int16_t speed;
    int16_t torque;
} HtMotorGroupPvtRaw;

/**
 * @brief 一拖多位置、速度、力矩、Kp、Kd 原始控制量
 */
typedef struct {
    int16_t position;
    int16_t speed;
    int16_t torque;
    int16_t kp;
    int16_t kd;
} HtMotorGroupMitRaw;

/**
 * @brief 高擎电机统一接口实例
 */
extern const LegacyBusMotorInterface ht_motor_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 解析高擎电机 FDCAN 回复帧并刷新本地缓存
 * @param frame_id FDCAN 扩展 ID
 * @param data 回复数据
 * @param len 回复数据长度
 * @param feedback 可选输出反馈，允许为 NULL
 * @return 电机状态码
 */
BusMotorStatus ht_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t* data,
    uint8_t len,
    BusMotorFeedback* feedback);

BusMotorStatus ht_motor_request_feedback(uint16_t id);

/**
 * @brief 发送完整运控指令：位置、速度、前馈力矩、Kp、Kd
 *
 * 位置和速度输入单位与统一接口一致，分别为 rad、rad/s。
 * @param id 电机 ID
 * @param position 目标位置，rad
 * @param speed 目标速度，rad/s
 * @param torque 前馈力矩，N*m
 * @param kp 位置比例系数
 * @param kd 速度比例系数
 * @return 电机状态码
 */
BusMotorStatus ht_motor_set_motion(uint16_t id,
    float position,
    float speed,
    float torque,
    float kp,
    float kd);

/**
 * @brief 发送一拖多位置、速度、力矩原始指令
 * @param first_id 本组首个电机 ID，仅支持 1、11、21
 * @param commands 控制量数组
 * @param count 本次写入数量，不得超过对应分组容量
 * @param query_id 需要查询状态的电机 ID；0 表示不查询
 * @return 电机状态码
 */
BusMotorStatus ht_motor_group_set_pvt_raw(uint16_t first_id,
    const HtMotorGroupPvtRaw* commands,
    uint8_t count,
    uint8_t query_id);

/**
 * @brief 发送一拖多位置、速度、力矩、Kp、Kd原始指令
 * @param first_id 本组首个电机 ID，仅支持 1、7、13、19、25
 * @param commands 控制量数组
 * @param count 本次写入数量，不得超过对应分组容量
 * @param query_id 需要查询状态的电机 ID；0 表示不查询
 * @return 电机状态码
 */
BusMotorStatus ht_motor_group_set_motion_raw(uint16_t first_id,
    const HtMotorGroupMitRaw* commands,
    uint8_t count,
    uint8_t query_id);

#endif
