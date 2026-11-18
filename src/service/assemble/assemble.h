#ifndef _assemble_h_
#define _assemble_h_

#include <stdbool.h>
#include <stdint.h>
#include "bus_motor/bus_motor.h"
extern volatile float g_yaw_velocity;
// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    SYSTEM_STATUS_OK = 0,
    SYSTEM_STATUS_ERROR,
} SystemStatus;


// ! ========================= 接 口 函 数 声 明 ========================= ! //

SystemStatus assemble_gimbal(void);
SystemStatus assemble_user_io(void);
SystemStatus assemble_delay(void);
SystemStatus assemble_log(void);
SystemStatus assemble_rgb(void);
SystemStatus assemble_imu(void);
SystemStatus assemble_ht_motor(void);
void assemble_ht_motor_process(void);
bool assemble_ht_motor_has_feedback(uint16_t id);
bool assemble_ht_motor_get_raw_position(uint16_t id, float* position);
bool assemble_ht_motor_set_target_position(uint16_t id, float position);
bool assemble_ht_motor_set_target_position_speed(uint16_t id, float position, float speed);
bool assemble_ht_motor_set_target_speed(uint16_t id, float speed);
bool assemble_ht_motor_get_feedback_snapshot_age(uint16_t id, BusMotorFeedback* feedback, uint32_t* age_ms);
bool assemble_ht_motor_set_control_target(uint16_t id, float position, float speed, uint32_t timeout_ms);
void assemble_ht_motor_cancel_control(uint16_t id);
SystemStatus assemble_dm_motor(void);
void assemble_dm_motor_process(void);
bool assemble_dm_motor_get_last_sent_target(float* position);
bool assemble_dm_motor_get_feedback_snapshot(BusMotorFeedback* feedback);
bool assemble_dm_motor_get_feedback_snapshot_age(BusMotorFeedback* feedback, uint32_t* age_ms);
bool assemble_dm_motor_set_target_position_guarded(float position, float speed, uint32_t timeout_ms);
void assemble_dm_motor_clear_control_target(void);
bool assemble_ht_motor_get_feedback_snapshot(uint16_t id, BusMotorFeedback* feedback);
bool assemble_dm_motor_has_feedback(void);
bool assemble_dm_motor_is_ready(void);
bool assemble_dm_motor_get_position(float* position);
bool assemble_dm_motor_set_target_position(float position, float speed);

#endif
