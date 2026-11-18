#ifndef _gimbal_config_h_
#define _gimbal_config_h_

#include "gimbal.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/** @brief 与冻结电机装配保持一致；本版不改变总线 ID */
#define GIMBAL_YAW_MOTOR_ID 1u
#define GIMBAL_ROLL_MOTOR_ID 8u
#define GIMBAL_PITCH_MOTOR_ID 2u

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/** @brief 云台坐标、控制参数与日志周期 */
GimbalConfig gimbal_project_config(const GimbalPortOps* ops);

#endif
