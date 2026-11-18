#include "gimbal_control.h"
#include <math.h>
#include <stddef.h>

/* ==================== ALL USER TUNING PARAMETERS ==================== */
/* Startup reference: 50 samples at 20 ms -> approximately one second. */
#define GIMBAL_STARTUP_STABLE_US 300000u
#define GIMBAL_STARTUP_MAX_POSITION_ERROR_RAD 0.02f
#define GIMBAL_STARTUP_MAX_MOTOR_RATE_RAD_S 0.05f
#define GIMBAL_STARTUP_MAX_IMU_RATE_RAD_S 0.10f
#define GIMBAL_REFERENCE_SAMPLES 50u
#define GIMBAL_REFERENCE_INTERVAL_US 20000u
#define GIMBAL_REFERENCE_SLOT_TOLERANCE_US 5000u

#define GIMBAL_YAW_ENABLE 1u
/* +1: increasing motor position increases IMU yaw. Verify on hardware. */
#define GIMBAL_YAW_MOTOR_SIGN 1.0f
#define GIMBAL_YAW_POSITION_KP 4.0f  /* motor rad / yaw error rad */
#define GIMBAL_YAW_POSITION_KD 1.0f  /* seconds: yaw-rate damping -> motor radians */
/* Camera IMU + encoder velocity disturbance feedforward (outer Z yaw axis).
 * Position correction is primary; velocity feedforward adds a bounded lead. */
#define GIMBAL_YAW_FF_ENABLE 1u
#define GIMBAL_YAW_FF_GAIN 0.8f
#define GIMBAL_YAW_FF_LEAD_S 0.10f /* seconds: velocity -> position lead */
#define GIMBAL_YAW_FF_FILTER_S 0.006f
#define GIMBAL_YAW_FF_RATE_DEADBAND_RAD_S 0.1f
#define GIMBAL_YAW_DEADBAND_RAD 0.000745329f
#define GIMBAL_YAW_MAX_SPEED_RAD_S 40.0f
#define GIMBAL_YAW_MOTOR_VEL_RAD_S 30.0f
#define GIMBAL_YAW_MAX_CORRECTION_RAD 10.0f /* max target lead over encoder */
#define GIMBAL_YAW_MAX_TRAVEL_RAD 20.0f /* +/- from first control motor position */
#define GIMBAL_YAW_CONTROL_PERIOD_US 3000u /* 3 ms minimum interval; approximately 333 Hz */
#define GIMBAL_YAW_COMMAND_TIMEOUT_MS 40u
#define GIMBAL_YAW_FEEDBACK_MAX_AGE_MS 20u
#define GIMBAL_IMU_MAX_AGE_US 10000u
#define GIMBAL_ACCEL_MAX_AGE_US 10000u
#define GIMBAL_YAW_MAX_STEP_DT_US 10000u /* cap target slew step after a delayed loop */
#define GIMBAL_YAW_MIN_COS_PITCH 0.50f /* suspend near Euler singularity */
#define GIMBAL_DM_POSITION_LIMIT_RAD 12.5f /* DMG6220 protocol range, not mechanical range */

/* PITCH / ROLL PARAMETERS: all controller tuning stays in this file. */
#define GIMBAL_PITCH_ENABLE 1u
#define GIMBAL_ROLL_ENABLE 1u
/* Set only after confirming outer Z(yaw), middle X(roll), inner Y(pitch),
 * and that the attitude estimator body frame matches the camera frame. */
#define GIMBAL_TILT_GEOMETRY_CONFIRMED 1u
#define GIMBAL_TILT_COUPLED_ENABLE 0u /* 0: separate axis loops; 1: coupled inverse + shared slew */
#define GIMBAL_PITCH_MOTOR_SIGN 1.0f
#define GIMBAL_ROLL_MOTOR_SIGN 1.0f
/* Encoder positions with the camera square to the outer frame (NOT boot targets).
 * Zero defaults are provisional; confirm both on the mechanism.
 * Keep native IMU axes: X rear, Y right, Z up. Motor signs refer to these axes. */
#define GIMBAL_ROLL_JOINT_ZERO_RAD 0.0f
#define GIMBAL_PITCH_JOINT_ZERO_RAD 0.0f /* provisional: measure mechanical zero */
#define GIMBAL_PITCH_KP 0.90f
#define GIMBAL_PITCH_KD 0.0f
#define GIMBAL_PITCH_DEADBAND_RAD 0.0034906585f
#define GIMBAL_PITCH_MAX_SPEED_RAD_S 1.5f
#define GIMBAL_PITCH_MAX_CORRECTION_RAD 30.0f
#define GIMBAL_PITCH_MAX_TRAVEL_RAD 15.0f

#define GIMBAL_ROLL_KP 0.2f
#define GIMBAL_ROLL_ERROR_POWER 1.1f /* >= 1; 1 restores linear P */
#define GIMBAL_ROLL_ERROR_REF_RAD 0.0523598776f /* 3 deg after deadband */
#define GIMBAL_ROLL_KD 0.0f
#define GIMBAL_ROLL_DEADBAND_RAD 0.0034906585f
#define GIMBAL_ROLL_MAX_SPEED_RAD_S 1.5f
#define GIMBAL_ROLL_MAX_CORRECTION_RAD 3.0f
#define GIMBAL_ROLL_MAX_TRAVEL_RAD 1.5f
#define GIMBAL_TILT_CONTROL_PERIOD_US 10000u
#define GIMBAL_TILT_MAX_DT_US 30000u /* cap slew dt, not a cancellation threshold */
#define GIMBAL_TILT_STALL_TIMEOUT_US 100000u
#define GIMBAL_TILT_GOAL_FILTER_S 0.020f /* 20 ms target smoothing; 0 disables */
#define GIMBAL_TILT_FEEDBACK_MAX_AGE_MS 60u
#define GIMBAL_TILT_COMMAND_TIMEOUT_MS 100u
#define GIMBAL_TILT_MIN_COS_PITCH 0.50f
#define GIMBAL_TILT_MIN_AXIS_RESPONSE 0.20f /* minimum single-axis angle response */
#define GIMBAL_TILT_MIN_JACOBIAN_DET 0.20f
#define GIMBAL_HT_POSITION_LIMIT_RAD 20.58f /* int16 position protocol range */
/* =================================================================== */


static float wrap_pi(float a) {
    a = fmodf(a, 6.28318530718f);
    if(a > 3.14159265359f) a -= 6.28318530718f;
    if(a < -3.14159265359f) a += 6.28318530718f;
    return a;
}
static void gimbal_reference_reset(GimbalReference *s) {
    if(s != NULL) *s = (GimbalReference){0};
}
static bool eligible(const Bmi088Snapshot *p, uint32_t now) {
    if(p == NULL || !p->calibrated ||
       !(p->sample.flags & IMU_SAMPLE_GYRO_VALID) ||
       !(p->sample.flags & IMU_SAMPLE_ACC_VALID) ||
       (uint32_t)(now - p->sample.gyro_timestamp_us) > GIMBAL_IMU_MAX_AGE_US ||
       (uint32_t)(now - p->sample.acc_timestamp_us) > GIMBAL_ACCEL_MAX_AGE_US) return false;
    return isfinite(p->angle.roll) && isfinite(p->angle.pitch) && isfinite(p->angle.yaw);
}
static void begin(GimbalReference *s, const Bmi088Snapshot *p) {
    gimbal_reference_reset(s);
    s->collecting = true;
    s->have_sequence = true;
    s->last_sequence = p->sequence;
    s->start_us = p->sample.gyro_timestamp_us;
    s->anchor = p->angle;
    s->previous = p->angle;
}
static void gimbal_reference_update(GimbalReference *s, const Bmi088Snapshot *p, uint32_t now) {
    if(s == NULL || s->ready) return;
    if(!eligible(p, now)) { gimbal_reference_reset(s); return; }
    if(!s->collecting) { begin(s, p); return; }
    if(s->have_sequence && p->sequence == s->last_sequence) return;
    s->last_sequence = p->sequence;
    /* Unwrap consecutive samples; movement does not reset the window. */
    s->roll_delta += wrap_pi(p->angle.roll - s->previous.roll);
    s->pitch_delta += wrap_pi(p->angle.pitch - s->previous.pitch);
    s->yaw_delta += wrap_pi(p->angle.yaw - s->previous.yaw);
    s->previous = p->angle;
    uint32_t elapsed = p->sample.gyro_timestamp_us - s->start_us;
    uint32_t due = (s->count + 1u) * GIMBAL_REFERENCE_INTERVAL_US;
    if(elapsed < due) return;
    if(elapsed - due > GIMBAL_REFERENCE_SLOT_TOLERANCE_US) {
        begin(s, p); return;
    }
    /* Average the 50 sampled orientations, accounting for angular wrap. */
    s->roll_sum += s->roll_delta; s->pitch_sum += s->pitch_delta; s->yaw_sum += s->yaw_delta;
    if(++s->count == GIMBAL_REFERENCE_SAMPLES) {
        s->target.roll = wrap_pi(s->anchor.roll + (float)(s->roll_sum / GIMBAL_REFERENCE_SAMPLES));
        s->target.pitch = wrap_pi(s->anchor.pitch + (float)(s->pitch_sum / GIMBAL_REFERENCE_SAMPLES));
        s->target.yaw = wrap_pi(s->anchor.yaw + (float)(s->yaw_sum / GIMBAL_REFERENCE_SAMPLES));
        s->ready = true;
        s->collecting = false;
    }
}

void gimbal_control_reset(GimbalControl *s) {
    if(s != NULL) *s = (GimbalControl){0};
}
void gimbal_control_calibrate(GimbalControl *s, const Bmi088Snapshot *sample, uint32_t now_us) {
    if(s != NULL && s->startup.reference_allowed)
        gimbal_reference_update(&s->reference, sample, now_us);
}
static bool startup_feedback_valid(const GimbalYawFeedback *m, uint32_t max_age,
                                   float position_limit) {
    return m && m->ready && m->age_ms<=max_age && m->velocity_valid &&
        isfinite(m->position) && fabsf(m->position)<=position_limit && isfinite(m->velocity);
}
bool gimbal_control_startup_update(GimbalControl *s, const Bmi088Snapshot *p,
    const GimbalYawFeedback *pitch, const GimbalYawFeedback *roll,
    const GimbalYawFeedback *yaw, uint32_t now, bool holds_sent) {
    if(!s) return false;
    GimbalStartup *a=&s->startup;
    if(a->running) return true; /* Runtime reconnection must not redefine the reference. */
    bool ready=startup_feedback_valid(pitch,GIMBAL_TILT_FEEDBACK_MAX_AGE_MS,GIMBAL_HT_POSITION_LIMIT_RAD) &&
        startup_feedback_valid(roll,GIMBAL_TILT_FEEDBACK_MAX_AGE_MS,GIMBAL_HT_POSITION_LIMIT_RAD) &&
        startup_feedback_valid(yaw,GIMBAL_YAW_FEEDBACK_MAX_AGE_MS,GIMBAL_DM_POSITION_LIMIT_RAD);
    if(!ready) {
        *a=(GimbalStartup){0}; gimbal_reference_reset(&s->reference); return false;
    }
    if(!a->captured) {
        a->pitch_position=pitch->position; a->roll_position=roll->position;
        a->yaw_position=yaw->position; a->captured=true;
        return false;
    }
    if(!holds_sent) {
        a->holds_seen=a->stable=a->reference_allowed=false;
        gimbal_reference_reset(&s->reference); return false;
    }
    if(!a->holds_seen) { a->holds_seen=true; a->hold_us=now; return false; }
    if(!eligible(p,now) || !isfinite(p->gyro_corrected.x) ||
       !isfinite(p->gyro_corrected.y) || !isfinite(p->gyro_corrected.z)) {
        a->stable=a->reference_allowed=false;
        gimbal_reference_reset(&s->reference); return false;
    }
    if(a->reference_allowed) {
        /* Keep the requested fixed 50-sample window; motion does not restart it. */
        a->running=s->reference.ready;
        return a->running;
    }
    bool settled=(int32_t)(now-pitch->age_ms*1000u-a->hold_us)>0 &&
        (int32_t)(now-roll->age_ms*1000u-a->hold_us)>0 &&
        (int32_t)(now-yaw->age_ms*1000u-a->hold_us)>0 &&
        fabsf(pitch->position-a->pitch_position)<=GIMBAL_STARTUP_MAX_POSITION_ERROR_RAD &&
        fabsf(roll->position-a->roll_position)<=GIMBAL_STARTUP_MAX_POSITION_ERROR_RAD &&
        fabsf(yaw->position-a->yaw_position)<=GIMBAL_STARTUP_MAX_POSITION_ERROR_RAD &&
        fabsf(pitch->velocity)<=GIMBAL_STARTUP_MAX_MOTOR_RATE_RAD_S &&
        fabsf(roll->velocity)<=GIMBAL_STARTUP_MAX_MOTOR_RATE_RAD_S &&
        fabsf(yaw->velocity)<=GIMBAL_STARTUP_MAX_MOTOR_RATE_RAD_S &&
        fabsf(p->gyro_corrected.x)<=GIMBAL_STARTUP_MAX_IMU_RATE_RAD_S &&
        fabsf(p->gyro_corrected.y)<=GIMBAL_STARTUP_MAX_IMU_RATE_RAD_S &&
        fabsf(p->gyro_corrected.z)<=GIMBAL_STARTUP_MAX_IMU_RATE_RAD_S;
    if(!settled) { a->stable=false; return false; }
    if(!a->stable) { a->stable=true; a->stable_us=now; }
    if((uint32_t)(now-a->stable_us)>=GIMBAL_STARTUP_STABLE_US)
        a->reference_allowed=true;
    return false;
}
bool gimbal_control_get_reference(const GimbalControl *s, ImuAngle *target) {
    if(s == NULL || target == NULL || !s->reference.ready) return false;
    *target = s->reference.target;
    return true;
}
static bool tilt_feedback_valid(const GimbalYawFeedback *m);

static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ==================== YAW ATTITUDE HOLD ALGORITHM ====================
 * e = wrap(target_yaw - measured_yaw)
 * q_goal = encoder + (POSITION_KP*deadband(e) - POSITION_KD*yaw_rate)/J
 *          - FF_GAIN*FF_LEAD_S*filtered_base_rate/J
 * Slew the combined position target at MAX_SPEED; no accumulated feedforward.
 * Camera-mounted IMU closes the outer loop; DM POS_VEL closes its own
 * position loop. This is NOT the DJI current-output dual PID controller.
 */
bool gimbal_control_yaw_update(GimbalControl *s, const Bmi088Snapshot *p,
                              const GimbalYawFeedback *m,
                              const GimbalYawFeedback *pitch, const GimbalYawFeedback *roll,
                              uint32_t now, GimbalYawCommand *out) {
    if(s == NULL || out == NULL) return false;
    *out = (GimbalYawCommand){0};
    bool valid = GIMBAL_YAW_ENABLE && s->reference.ready && eligible(p, now) &&
        m != NULL && m->ready && m->age_ms <= GIMBAL_YAW_FEEDBACK_MAX_AGE_MS &&
        tilt_feedback_valid(pitch) && tilt_feedback_valid(roll) &&
        isfinite(m->position) && fabsf(m->position) <= GIMBAL_DM_POSITION_LIMIT_RAD &&
        isfinite(p->gyro_corrected.y) && isfinite(p->gyro_corrected.z) &&
        fabsf(cosf(p->angle.pitch)) >= GIMBAL_YAW_MIN_COS_PITCH;
    if(!valid) {
        s->yaw_active = false;
        s->yaw_status = GIMBAL_YAW_ENABLE ? GIMBAL_YAW_WAIT : GIMBAL_YAW_OFF;
        return true; /* Explicitly cancel the driver's previous corrective target. */
    }
    out->speed = GIMBAL_YAW_MOTOR_VEL_RAD_S;

    out->timeout_ms = GIMBAL_YAW_COMMAND_TIMEOUT_MS;
    if(s->motor_origin_ready &&
       fabsf(m->position - s->motor_origin) > GIMBAL_YAW_MAX_TRAVEL_RAD) {
        s->yaw_active = false;
        s->yaw_status = GIMBAL_YAW_LIMIT;
        return true;
    }
    if(!s->yaw_active) {
        if(!s->motor_origin_ready) {
            s->motor_origin = m->position;
            s->motor_origin_ready = true;
        }
        s->yaw_base_rate = 0.0f;
        s->yaw_ff_position = 0.0f;
        s->command_position = m->position; /* Bumpless start/reconnect. */
        s->last_control_us = now;
        s->last_sequence = p->sequence;
        s->yaw_active = true;
        s->yaw_status = GIMBAL_YAW_WAIT;
        out->valid = true; out->position = m->position;
        return true;
    }
    uint32_t elapsed = now - s->last_control_us;
    if(elapsed < GIMBAL_YAW_CONTROL_PERIOD_US || p->sequence == s->last_sequence)
        return false;
    s->last_control_us = now;
    s->last_sequence = p->sequence;
    float dt = (elapsed > GIMBAL_YAW_MAX_STEP_DT_US ? GIMBAL_YAW_MAX_STEP_DT_US : elapsed) * 0.000001f;
    s->yaw_error = wrap_pi(s->reference.target.yaw - p->angle.yaw);
    /* Standard ZYX attitude: body gyro Z alone is not yaw rate when tilted. */
    s->yaw_rate = (sinf(p->angle.roll) * p->gyro_corrected.y +
                   cosf(p->angle.roll) * p->gyro_corrected.z) / cosf(p->angle.pitch);
    float error = s->yaw_error;
    if(fabsf(error) <= GIMBAL_YAW_DEADBAND_RAD) error = 0.0f;
    else error += error > 0.0f ? -GIMBAL_YAW_DEADBAND_RAD : GIMBAL_YAW_DEADBAND_RAD;
    /* Z-X-Y joints, expressed in native IMU body axes.
     * Outer yaw body axis=(-sin(qp)cos(qr), sin(qr), cos(qp)cos(qr)).
     * Outer yaw encoder angle does not rotate the inner axes. */
    float qp=GIMBAL_PITCH_MOTOR_SIGN*(pitch->position-GIMBAL_PITCH_JOINT_ZERO_RAD);
    float qr=GIMBAL_ROLL_MOTOR_SIGN*(roll->position-GIMBAL_ROLL_JOINT_ZERO_RAD);
    float sr=sinf(p->angle.roll), cr=cosf(p->angle.roll), cp=cosf(p->angle.pitch);
    float joint_to_yaw=GIMBAL_YAW_MOTOR_SIGN*(sr*sinf(qr)+cr*cosf(qp)*cosf(qr))/cp;
    if(!isfinite(joint_to_yaw) || fabsf(joint_to_yaw)<GIMBAL_TILT_MIN_AXIS_RESPONSE) {
        s->yaw_active=false; s->yaw_status=GIMBAL_YAW_WAIT; return true;
    }
    float correction=(GIMBAL_YAW_POSITION_KP*error-GIMBAL_YAW_POSITION_KD*s->yaw_rate)/joint_to_yaw;
    s->yaw_ff_position = 0.0f;
    if(GIMBAL_YAW_FF_ENABLE && m->velocity_valid && isfinite(m->velocity) &&
       pitch->velocity_valid && isfinite(pitch->velocity) &&
       roll->velocity_valid && isfinite(roll->velocity) &&
       fabsf(joint_to_yaw) > 0.1f) {
        /* Remove all three joint contributions, not only the yaw motor. */
        float base_rate = s->yaw_rate - joint_to_yaw*m->velocity
            - GIMBAL_PITCH_MOTOR_SIGN*sr/cp*pitch->velocity
            - GIMBAL_ROLL_MOTOR_SIGN*cr*sinf(qp)/cp*roll->velocity;
        float sample_dt = elapsed * 0.000001f;
        float alpha = sample_dt / (GIMBAL_YAW_FF_FILTER_S + sample_dt);
        s->yaw_base_rate += alpha * (base_rate - s->yaw_base_rate);
        float filtered_base_rate = s->yaw_base_rate;
        if(fabsf(filtered_base_rate) < GIMBAL_YAW_FF_RATE_DEADBAND_RAD_S)
            filtered_base_rate = 0.0f;
        s->yaw_ff_position = -GIMBAL_YAW_FF_GAIN * GIMBAL_YAW_FF_LEAD_S *
            filtered_base_rate / joint_to_yaw;
        correction += s->yaw_ff_position;
    } else {
        s->yaw_base_rate = 0.0f;
    }
    correction = clampf(correction, -GIMBAL_YAW_MAX_CORRECTION_RAD, GIMBAL_YAW_MAX_CORRECTION_RAD);
    float lower = fmaxf(-GIMBAL_DM_POSITION_LIMIT_RAD, s->motor_origin - GIMBAL_YAW_MAX_TRAVEL_RAD);
    float upper = fminf(GIMBAL_DM_POSITION_LIMIT_RAD, s->motor_origin + GIMBAL_YAW_MAX_TRAVEL_RAD);
    float goal = clampf(m->position + correction, lower, upper);
    float step = GIMBAL_YAW_MAX_SPEED_RAD_S * dt;
    float next = s->command_position + clampf(goal - s->command_position, -step, step);
    lower = fmaxf(lower, m->position - GIMBAL_YAW_MAX_CORRECTION_RAD);
    upper = fminf(upper, m->position + GIMBAL_YAW_MAX_CORRECTION_RAD);
    s->command_position = clampf(next, lower, upper);
    s->yaw_status = (fabsf(s->command_position - s->motor_origin) >= GIMBAL_YAW_MAX_TRAVEL_RAD) ? GIMBAL_YAW_LIMIT : GIMBAL_YAW_TRACK;
    out->valid = true;
    out->position = s->command_position;
    return true;
}

/* PITCH / ROLL PD: attitude error -> bounded relative motor position.
 * Euler rates come from calibrated body gyro. The 2x2 Jacobian compensates
 * Z-X-Y mechanism, with outer yaw held fixed in this local solve.
 * This remains separate from the yaw loop, not a full three-axis inverse.
 */
static float tilt_deadband(float e, float band) {
    return fabsf(e) <= band ? 0.0f : e - copysignf(band, e);
}
/* Signed normalized power: preserve direction and radians.
 * P equals linear P at |error after deadband| == ERROR_REF_RAD. */
static float roll_position_correction(float error) {
    float e = tilt_deadband(error, GIMBAL_ROLL_DEADBAND_RAD);
    float magnitude = GIMBAL_ROLL_ERROR_REF_RAD *
        powf(fabsf(e) / GIMBAL_ROLL_ERROR_REF_RAD, GIMBAL_ROLL_ERROR_POWER);
    return GIMBAL_ROLL_KP * copysignf(magnitude, e);
}
static bool tilt_feedback_valid(const GimbalYawFeedback *m) {
    return m && m->ready && m->age_ms <= GIMBAL_TILT_FEEDBACK_MAX_AGE_MS && isfinite(m->position);
}
/* Bound position goals before applying a shared two-axis slew fraction. */
static float tilt_goal(const GimbalTiltAxis *a, float position, float delta,
                       float lead, float travel) {
    float lo=fmaxf(a->origin-travel,-GIMBAL_HT_POSITION_LIMIT_RAD);
    float hi=fminf(a->origin+travel,GIMBAL_HT_POSITION_LIMIT_RAD);
    return clampf(position+clampf(delta,-lead,lead),lo,hi);
}
static float tilt_slew_fraction(float dp, float dr, float dt) {
    float fraction=1.0f;
    if(fabsf(dp)>GIMBAL_PITCH_MAX_SPEED_RAD_S*dt)
        fraction=GIMBAL_PITCH_MAX_SPEED_RAD_S*dt/fabsf(dp);
    if(fabsf(dr)>GIMBAL_ROLL_MAX_SPEED_RAD_S*dt)
        fraction=fminf(fraction,GIMBAL_ROLL_MAX_SPEED_RAD_S*dt/fabsf(dr));
    return fraction;
}
void gimbal_control_note_exit(GimbalTiltAxis *a, uint32_t reason) {
    if(a && a->active) { ++a->exit_count; a->last_exit_reason=reason; }
}
static bool tilt_axis_update(GimbalTiltAxis *a, bool enabled, bool valid,
    const GimbalYawFeedback *m, uint32_t now, uint32_t seq, float delta,
    float speed, float lead, float travel, float shared_step, GimbalYawCommand *out) {
    *out = (GimbalYawCommand){0};
    if(!enabled || !valid || !isfinite(delta)) {
        gimbal_control_note_exit(a,enabled?2u:1u);
        a->active = false; a->status = enabled ? GIMBAL_YAW_WAIT : GIMBAL_YAW_OFF;
        return true;
    }
    if(fabsf(m->position)>GIMBAL_HT_POSITION_LIMIT_RAD ||
       (a->origin_ready && fabsf(m->position-a->origin) > travel)) {
        gimbal_control_note_exit(a,3u);
        a->active = false; a->status = GIMBAL_YAW_LIMIT; return true;
    }
    out->speed = speed; out->timeout_ms = GIMBAL_TILT_COMMAND_TIMEOUT_MS;
    if(!a->active) {
        if(!a->origin_ready) { a->origin=m->position; a->origin_ready=true; }
        a->command=m->position; a->filtered_goal=m->position; a->last_us=now; a->sequence=seq; a->active=true;
        a->status=GIMBAL_YAW_WAIT; out->valid=true; out->position=a->command; return true;
    }
    uint32_t elapsed=now-a->last_us;
    if(elapsed>GIMBAL_TILT_STALL_TIMEOUT_US) {
        gimbal_control_note_exit(a,4u);
        a->active=false; a->status=GIMBAL_YAW_WAIT; return true;
    }
    if(elapsed<GIMBAL_TILT_CONTROL_PERIOD_US || seq==a->sequence) return false;
    a->last_us=now; a->sequence=seq;
    float lo=fmaxf(a->origin-travel,-GIMBAL_HT_POSITION_LIMIT_RAD);
    float hi=fminf(a->origin+travel,GIMBAL_HT_POSITION_LIMIT_RAD);
    float goal=tilt_goal(a,m->position,delta,lead,travel);
    float dt=fminf((float)elapsed,(float)GIMBAL_TILT_MAX_DT_US)*0.000001f;
    /* Smooth encoder-step changes as well as attitude correction. No accumulation.
     * Coupled mode retains its existing shared slew geometry. */
    if(!GIMBAL_TILT_COUPLED_ENABLE && GIMBAL_TILT_GOAL_FILTER_S>0.0f) {
        float alpha=dt/(GIMBAL_TILT_GOAL_FILTER_S+dt);
        a->filtered_goal+=alpha*(goal-a->filtered_goal);
        goal=a->filtered_goal;
    } else a->filtered_goal=goal;
    float step=shared_step>=0.0f?shared_step:speed*dt;
    float next=a->command+clampf(goal-a->command,-step,step);
    /* Saturate instead of cancelling/reinitializing on a lead-limit boundary. */
    lo=fmaxf(lo,m->position-lead); hi=fminf(hi,m->position+lead);
    a->command=clampf(next,lo,hi);
    a->filtered_goal=clampf(a->filtered_goal,lo,hi);
    a->status=(goal==lo || goal==hi)?GIMBAL_YAW_LIMIT:GIMBAL_YAW_TRACK;
    out->valid=true; out->position=a->command; return true;
}

void gimbal_control_tilt_update(GimbalControl *s, const Bmi088Snapshot *p,
    const GimbalYawFeedback *pitch, const GimbalYawFeedback *roll,
    const GimbalYawFeedback *yaw, uint32_t now,
    GimbalYawCommand *po, GimbalYawCommand *ro, bool *pc, bool *rc) {
    if(!s || !po || !ro || !pc || !rc) return;
    (void)yaw; /* Outer yaw encoder angle is irrelevant to the inner-axis Jacobian. */
    float dp=0.0f, dr=0.0f;
    bool valid=GIMBAL_TILT_GEOMETRY_CONFIRMED && s->reference.ready && eligible(p,now) &&
        tilt_feedback_valid(pitch) && tilt_feedback_valid(roll) &&
        isfinite(p->gyro_corrected.x) && isfinite(p->gyro_corrected.y) && isfinite(p->gyro_corrected.z) &&
        fabsf(cosf(p->angle.pitch))>=GIMBAL_TILT_MIN_COS_PITCH;
    bool pitch_valid=valid, roll_valid=valid;
    if(valid) {
        float sr=sinf(p->angle.roll), cr=cosf(p->angle.roll), tp=tanf(p->angle.pitch);
        s->pitch.error=wrap_pi(s->reference.target.pitch-p->angle.pitch);
        s->roll.error=wrap_pi(s->reference.target.roll-p->angle.roll);
        s->pitch.rate=cr*p->gyro_corrected.y-sr*p->gyro_corrected.z;
        s->roll.rate=p->gyro_corrected.x+tp*(sr*p->gyro_corrected.y+cr*p->gyro_corrected.z);
        float up = 0.0f, ur = 0.0f;
        if(GIMBAL_PITCH_ENABLE)
            up = GIMBAL_PITCH_KP * tilt_deadband(s->pitch.error, GIMBAL_PITCH_DEADBAND_RAD)
                 - GIMBAL_PITCH_KD * s->pitch.rate;
        if(GIMBAL_ROLL_ENABLE)
            ur = roll_position_correction(s->roll.error)
                 - GIMBAL_ROLL_KD * s->roll.rate;
        float qp=GIMBAL_PITCH_MOTOR_SIGN*(pitch->position-GIMBAL_PITCH_JOINT_ZERO_RAD);
        float sp=sinf(qp), cp=cosf(qp);
        /* Inner pitch body axis=(0,1,0); middle roll=(cos(qp),0,sin(qp)).
         * Convert body axes to ZYX attitude rates; no yaw-angle rotation. */
        float jpp=GIMBAL_PITCH_MOTOR_SIGN*cr;
        float jpr=-GIMBAL_ROLL_MOTOR_SIGN*sr*sp;
        float jrp=GIMBAL_PITCH_MOTOR_SIGN*tp*sr;
        float jrr=GIMBAL_ROLL_MOTOR_SIGN*(cp+tp*cr*sp);
        s->pitch.axis_response=jpp;
        s->roll.axis_response=jrr;
        if(GIMBAL_TILT_COUPLED_ENABLE && GIMBAL_PITCH_ENABLE && GIMBAL_ROLL_ENABLE) {
            float det=jpp*jrr-jpr*jrp;
            valid=isfinite(det) && fabsf(det)>=GIMBAL_TILT_MIN_JACOBIAN_DET;
            pitch_valid=roll_valid=valid;
            if(valid) { dp=(jrr*up-jpr*ur)/det; dr=(-jrp*up+jpp*ur)/det; }
        } else {
            /* Each motor uses its own attitude error; retain local axis geometry.
             * Physical cross-axis motion still exists and is not cancelled here. */
            pitch_valid=isfinite(jpp) && fabsf(jpp)>=GIMBAL_TILT_MIN_AXIS_RESPONSE;
            roll_valid=isfinite(jrr) && fabsf(jrr)>=GIMBAL_TILT_MIN_AXIS_RESPONSE;
            if(GIMBAL_PITCH_ENABLE && pitch_valid) dp=up/jpp;
            if(GIMBAL_ROLL_ENABLE && roll_valid) dr=ur/jrr;
        }
    }
    uint32_t seq=p?p->sequence:0u;
    float pitch_step=-1.0f, roll_step=-1.0f;
    /* Single-axis/reconnect behavior stays unchanged. Coordinate normal paired updates. */
    uint32_t elapsed=now-s->pitch.last_us;
    if(GIMBAL_TILT_COUPLED_ENABLE && GIMBAL_PITCH_ENABLE && GIMBAL_ROLL_ENABLE && valid &&
       isfinite(dp) && isfinite(dr) && s->pitch.active && s->roll.active &&
       s->pitch.last_us==s->roll.last_us && seq!=s->pitch.sequence && seq!=s->roll.sequence &&
       elapsed>=GIMBAL_TILT_CONTROL_PERIOD_US && elapsed<=GIMBAL_TILT_MAX_DT_US) {
        float pitch_delta=tilt_goal(&s->pitch,pitch->position,dp,
            GIMBAL_PITCH_MAX_CORRECTION_RAD,GIMBAL_PITCH_MAX_TRAVEL_RAD)-s->pitch.command;
        float roll_delta=tilt_goal(&s->roll,roll->position,dr,
            GIMBAL_ROLL_MAX_CORRECTION_RAD,GIMBAL_ROLL_MAX_TRAVEL_RAD)-s->roll.command;
        float fraction=tilt_slew_fraction(pitch_delta,roll_delta,elapsed*0.000001f);
        pitch_step=fabsf(pitch_delta)*fraction;
        roll_step=fabsf(roll_delta)*fraction;
    }
    *pc=tilt_axis_update(&s->pitch,GIMBAL_PITCH_ENABLE,pitch_valid,pitch,now,seq,dp,
        GIMBAL_PITCH_MAX_SPEED_RAD_S,GIMBAL_PITCH_MAX_CORRECTION_RAD,GIMBAL_PITCH_MAX_TRAVEL_RAD,pitch_step,po);
    *rc=tilt_axis_update(&s->roll,GIMBAL_ROLL_ENABLE,roll_valid,roll,now,seq,dr,
        GIMBAL_ROLL_MAX_SPEED_RAD_S,GIMBAL_ROLL_MAX_CORRECTION_RAD,GIMBAL_ROLL_MAX_TRAVEL_RAD,roll_step,ro);
}
