#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include "imu/bmi088.h"

typedef struct {
    bool collecting, ready, have_sequence;
    uint16_t count;
    uint32_t start_us, last_sequence;
    ImuAngle anchor, previous, target;
    double roll_delta, pitch_delta, yaw_delta;
    double roll_sum, pitch_sum, yaw_sum;
} GimbalReference;

typedef enum { GIMBAL_YAW_OFF, GIMBAL_YAW_WAIT, GIMBAL_YAW_TRACK,
               GIMBAL_YAW_LIMIT } GimbalYawStatus;
typedef struct {
    bool active, origin_ready;
    uint32_t last_us, sequence;
    float origin, command, error, rate;
    float filtered_goal;
    uint32_t exit_count, last_exit_reason; /* latched until reset; active->inactive only */
    float axis_response; /* latest local attitude-angle / motor-angle derivative */
    GimbalYawStatus status;
} GimbalTiltAxis;

typedef struct {
    bool captured, holds_seen, stable, reference_allowed, running;
    uint32_t hold_us, stable_us;
    float pitch_position, roll_position, yaw_position;
} GimbalStartup;

typedef struct {
    GimbalStartup startup;
    GimbalTiltAxis pitch, roll;
    GimbalReference reference;
    bool yaw_active, motor_origin_ready;
    uint32_t last_control_us, last_sequence;
    float motor_origin, command_position;
    float yaw_error, yaw_rate;
    float yaw_base_rate, yaw_ff_position; /* rad/s, motor radians */
    GimbalYawStatus yaw_status;
} GimbalControl;
typedef struct {
    bool ready;
    float position;
    uint32_t age_ms;
    float velocity; /* encoder-positive rad/s, from the same CAN snapshot */
    bool velocity_valid;
} GimbalYawFeedback;
typedef struct {
    bool valid;
    float position, speed;
    uint32_t timeout_ms;
} GimbalYawCommand;

/* Call before attitude control. holds_sent means the latest hold submissions succeeded,
 * not that the motor acknowledged them; fresh stationary feedback is checked separately. */
bool gimbal_control_startup_update(GimbalControl *s, const Bmi088Snapshot *sample,
    const GimbalYawFeedback *pitch, const GimbalYawFeedback *roll,
    const GimbalYawFeedback *yaw, uint32_t now_us, bool holds_sent);
/* Diagnostic reasons: 1 disabled, 2 input/geometry invalid, 3 position limit,
 * 4 long control stall, 5 transmit rejected. Does not change motor behavior. */
void gimbal_control_note_exit(GimbalTiltAxis *axis, uint32_t reason);
void gimbal_control_reset(GimbalControl *s);
void gimbal_control_calibrate(GimbalControl *s, const Bmi088Snapshot *sample, uint32_t now_us);
bool gimbal_control_get_reference(const GimbalControl *s, ImuAngle *target);
/* Return true when output (including cancellation) must be applied.
 * Poll every main iteration, even without a new IMU sample, for fault gating.
 * This module never accesses CAN or modifies the captured reference on recovery.
 */
bool gimbal_control_yaw_update(GimbalControl *s, const Bmi088Snapshot *sample,
                              const GimbalYawFeedback *motor,
                              const GimbalYawFeedback *pitch, const GimbalYawFeedback *roll,
                              uint32_t now_us,
                              GimbalYawCommand *command);
/* Yaw -> Roll -> Pitch mechanism; native IMU X rear, Y right, Z up.
 * Motor angles are absolute radians; mechanical zeros must match the hardware.
 * Each return flag means apply that output, including invalid/cancel outputs. */
void gimbal_control_tilt_update(GimbalControl *s, const Bmi088Snapshot *sample,
    const GimbalYawFeedback *pitch, const GimbalYawFeedback *roll,
    const GimbalYawFeedback *yaw, uint32_t now_us,
    GimbalYawCommand *pitch_out, GimbalYawCommand *roll_out,
    bool *pitch_changed, bool *roll_changed);
#endif
