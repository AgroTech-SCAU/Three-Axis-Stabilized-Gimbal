#ifndef _assemble_h_
#define _assemble_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    SYSTEM_STATUS_OK = 0,
    SYSTEM_STATUS_ERROR,
} SystemStatus;

extern volatile bool tim6_500hz_flag;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

SystemStatus assemble_delay(void);
SystemStatus assemble_log(void);
SystemStatus assemble_rgb(void);
SystemStatus assemble_imu(void);
SystemStatus assemble_chassis(void);
SystemStatus assemble_ht_motor(void);
void assemble_ht_motor_process(void);
bool assemble_ht_motor_has_feedback(uint16_t id);
bool assemble_ht_motor_get_raw_position(uint16_t id, float* position);
bool assemble_ht_motor_set_target_position(uint16_t id, float position);
bool assemble_ht_motor_set_target_position_speed(uint16_t id, float position, float speed);
bool assemble_ht_motor_set_target_speed(uint16_t id, float speed);
SystemStatus assemble_dm_motor(void);
void assemble_dm_motor_process(void);
bool assemble_dm_motor_has_feedback(void);
bool assemble_dm_motor_is_ready(void);
bool assemble_dm_motor_get_position(float* position);
bool assemble_dm_motor_set_target_position(float position, float speed);
SystemStatus assemble_suction(void);
SystemStatus assemble_odom(void);
SystemStatus assemble_remote(void);
SystemStatus assemble_tim6_500hz(void);
SystemStatus assemble_arm(void);
SystemStatus assemble_pc_comms(void);
SystemStatus assemble_pi_comms(void);
SystemStatus assemble_vision_comms(void);

#endif
