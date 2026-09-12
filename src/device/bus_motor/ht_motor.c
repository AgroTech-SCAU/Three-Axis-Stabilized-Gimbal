#include "ht_motor.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* HighTorque Classic CAN v2.x protocol constants. */
#define HT_MOTOR_DEFAULT_TIMEOUT_MS       20u
#define HT_MOTOR_REPLY_REQUEST_FLAG       0x8000u
#define HT_MOTOR_TWO_PI                   6.28318530717958647692f
#define HT_MOTOR_POS_RAW_PER_TURN         10000.0f
#define HT_MOTOR_VEL_RAW_PER_TURN_PER_SEC 4000.0f
#define HT_MOTOR_UNLIMITED_RAW             ((int16_t)INT16_MIN)

#define HT_CAN_CMD_NORMAL_0               0x07u
#define HT_CAN_CMD_NORMAL_1               0x07u
#define HT_CAN_CMD_POS_VEL_MAX_TORQUE_1   0x35u
#define HT_CAN_CMD_TORQUE_0               0x05u
#define HT_CAN_CMD_TORQUE_1               0x13u
#define HT_CAN_CMD_READ_I16_3             0x17u
#define HT_CAN_REPLY_I16_3                0x27u
#define HT_CAN_REG_POSITION               0x01u

/* Official HighTorque model torque correction coefficients. */
#define HT_4438_30_TORQUE_K  0.005256f
#define HT_4438_30_TORQUE_D -0.050000f
#define HT_5047_36_TORQUE_K  0.004938f
#define HT_5047_36_TORQUE_D -0.033130f

typedef struct {
    uint16_t id;
    HtMotorModel model;
    uint8_t mode;
    bool has_feedback;
    BusMotorFeedback feedback;
} HtMotorSlot;

static const BusMotorPortOps* s_ops = NULL;
static bool s_is_initialized = false;
static uint32_t s_timeout_ms = HT_MOTOR_DEFAULT_TIMEOUT_MS;
static uint8_t s_retry_count = 0u;
static HtMotorSlot s_slots[HT_MOTOR_MAX_ID];

static BusMotorStatus ht_motor_init(const BusMotorConfig* config);
static const char* ht_motor_status_str(BusMotorStatus status);
static const char* ht_motor_mode_str(BusMotorMode mode);
static BusMotorStatus ht_motor_enable(uint16_t id);
static BusMotorStatus ht_motor_disable(uint16_t id);
static BusMotorStatus ht_motor_switch_mode(uint16_t id, BusMotorMode mode);
static BusMotorStatus ht_motor_set_pos(uint16_t id, float position);
static BusMotorStatus ht_motor_set_spd(uint16_t id, float speed);
static BusMotorStatus ht_motor_set_pos_vel(uint16_t id, float position, float speed);
static BusMotorStatus ht_motor_set_tor(uint16_t id, float torque);
static BusMotorStatus ht_motor_set_pd(uint16_t id, float kp, float kd);
static BusMotorStatus ht_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback);
static float ht_motor_get_pos(uint16_t id);
static float ht_motor_get_spd(uint16_t id);
static float ht_motor_get_tor(uint16_t id);
static BusMotorStatus ht_motor_stop(uint16_t id);
static BusMotorStatus ht_motor_brake(uint16_t id);

static HtMotorSlot* ht_motor_get_slot(uint16_t id);
static const HtMotorSlot* ht_motor_get_slot_const(uint16_t id);
static void ht_motor_reset_slot(HtMotorSlot* slot, uint16_t id);
static uint32_t ht_motor_make_command_id(uint16_t id);
static uint16_t ht_motor_get_reply_source_id(uint32_t frame_id);
static BusMotorStatus ht_motor_send(uint16_t id, const uint8_t* data, uint8_t len);
static BusMotorStatus ht_motor_request_state(uint16_t id);
static BusMotorStatus ht_motor_wait_feedback(uint16_t id);
static int16_t ht_motor_read_i16_le(const uint8_t* data);
static void ht_motor_write_i16_le(uint8_t* data, int16_t value);
static int16_t ht_motor_saturate_i16(float value);
static int16_t ht_motor_position_to_raw(float position_rad);
static int16_t ht_motor_speed_to_raw(float speed_rad_s);
static float ht_motor_position_from_raw(int16_t raw);
static float ht_motor_speed_from_raw(int16_t raw);
static bool ht_motor_torque_coeff(HtMotorModel model, float* k, float* d);
static bool ht_motor_torque_to_raw(const HtMotorSlot* slot, float torque_nm, int16_t* raw);
static float ht_motor_torque_from_raw(const HtMotorSlot* slot, int16_t raw);

const LegacyBusMotorInterface ht_motor_instance = {
    .init = ht_motor_init,
    .status_str = ht_motor_status_str,
    .mode_str = ht_motor_mode_str,
    .enable = ht_motor_enable,
    .disable = ht_motor_disable,
    .switch_mode = ht_motor_switch_mode,
    .set_pos = ht_motor_set_pos,
    .set_spd = ht_motor_set_spd,
    .set_pos_vel = ht_motor_set_pos_vel,
    .set_tor = ht_motor_set_tor,
    .set_pd = ht_motor_set_pd,
    .update_feedback = ht_motor_update_feedback,
    .get_pos = ht_motor_get_pos,
    .get_spd = ht_motor_get_spd,
    .get_tor = ht_motor_get_tor,
    .stop = ht_motor_stop,
    .brake = ht_motor_brake,
};

static BusMotorStatus ht_motor_init(const BusMotorConfig* config) {
    uint16_t id;

    if(config == NULL || config->ops == NULL || config->ops->send == NULL) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    s_ops = config->ops;
    s_timeout_ms = config->timeout_ms == 0u ? HT_MOTOR_DEFAULT_TIMEOUT_MS : config->timeout_ms;
    s_retry_count = config->retry_count;

    for(id = 1u; id <= HT_MOTOR_MAX_ID; ++id) {
        ht_motor_reset_slot(&s_slots[id - 1u], id);
    }

    if(config->driver_config != NULL) {
        const HtMotorDriverConfig* driver_cfg = (const HtMotorDriverConfig*)config->driver_config;
        uint8_t i;

        if(driver_cfg->count > 0u && driver_cfg->profiles == NULL) {
            return MOTOR_STATUS_INVALID_PARAM;
        }

        for(i = 0u; i < driver_cfg->count; ++i) {
            HtMotorSlot* slot = ht_motor_get_slot(driver_cfg->profiles[i].id);
            if(slot == NULL) {
                return MOTOR_STATUS_INVALID_PARAM;
            }
            slot->model = driver_cfg->profiles[i].model;
        }
    }

    s_is_initialized = true;
    return MOTOR_STATUS_OK;
}

static const char* ht_motor_status_str(BusMotorStatus status) {
    switch(status) {
#define X(name, value) case MOTOR_STATUS_##name: return #name;
        MOTOR_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}

static const char* ht_motor_mode_str(BusMotorMode mode) {
    switch((HtMotorMode)mode) {
        case HT_MOTOR_MODE_STOP: return "STOP";
        case HT_MOTOR_MODE_POSITION: return "POSITION";
        case HT_MOTOR_MODE_BRAKE: return "BRAKE";
        default: return "UNSUPPORTED";
    }
}

/* Classic CAN control commands directly enter control; no extra enable frame is needed. */
static BusMotorStatus ht_motor_enable(uint16_t id) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    if(slot == NULL) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(!s_is_initialized) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    slot->mode = (uint8_t)HT_MOTOR_MODE_POSITION;
    return MOTOR_STATUS_OK;
}

static BusMotorStatus ht_motor_disable(uint16_t id) {
    return ht_motor_stop(id);
}

static BusMotorStatus ht_motor_switch_mode(uint16_t id, BusMotorMode mode) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    BusMotorStatus status;

    if(slot == NULL) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    switch((HtMotorMode)mode) {
        case HT_MOTOR_MODE_POSITION:
            return ht_motor_enable(id);
        case HT_MOTOR_MODE_STOP:
            status = ht_motor_stop(id);
            break;
        case HT_MOTOR_MODE_BRAKE:
            status = ht_motor_brake(id);
            break;
        default:
            return MOTOR_STATUS_UNSUPPORTED;
    }

    return status;
}

static BusMotorStatus ht_motor_set_pos(uint16_t id, float position) {
    uint8_t data[8] = { HT_CAN_CMD_NORMAL_0, HT_CAN_CMD_NORMAL_1, 0u, 0u, 0u, 0u, 0u, 0u };

    if(ht_motor_get_slot(id) == NULL || !isfinite(position)) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    ht_motor_write_i16_le(&data[2], ht_motor_position_to_raw(position));
    ht_motor_write_i16_le(&data[4], HT_MOTOR_UNLIMITED_RAW);
    ht_motor_write_i16_le(&data[6], HT_MOTOR_UNLIMITED_RAW);
    return ht_motor_send(id, data, sizeof(data));
}

static BusMotorStatus ht_motor_set_spd(uint16_t id, float speed) {
    uint8_t data[8] = { HT_CAN_CMD_NORMAL_0, HT_CAN_CMD_NORMAL_1, 0u, 0u, 0u, 0u, 0u, 0u };

    if(ht_motor_get_slot(id) == NULL || !isfinite(speed)) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    ht_motor_write_i16_le(&data[2], HT_MOTOR_UNLIMITED_RAW);
    ht_motor_write_i16_le(&data[4], ht_motor_speed_to_raw(speed));
    ht_motor_write_i16_le(&data[6], HT_MOTOR_UNLIMITED_RAW);
    return ht_motor_send(id, data, sizeof(data));
}

static BusMotorStatus ht_motor_set_pos_vel(uint16_t id, float position, float speed) {
    uint8_t data[8] = { HT_CAN_CMD_NORMAL_0, HT_CAN_CMD_POS_VEL_MAX_TORQUE_1, 0u, 0u, 0u, 0u, 0u, 0u };

    if(ht_motor_get_slot(id) == NULL || !isfinite(position) || !isfinite(speed)) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    ht_motor_write_i16_le(&data[2], ht_motor_speed_to_raw(speed));
    ht_motor_write_i16_le(&data[4], HT_MOTOR_UNLIMITED_RAW);
    ht_motor_write_i16_le(&data[6], ht_motor_position_to_raw(position));
    return ht_motor_send(id, data, sizeof(data));
}

static BusMotorStatus ht_motor_set_tor(uint16_t id, float torque) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    uint8_t data[4] = { HT_CAN_CMD_TORQUE_0, HT_CAN_CMD_TORQUE_1, 0u, 0u };
    int16_t raw;

    if(slot == NULL || !isfinite(torque)) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(!ht_motor_torque_to_raw(slot, torque, &raw)) {
        return MOTOR_STATUS_PROFILE_MISMATCH;
    }

    ht_motor_write_i16_le(&data[2], raw);
    return ht_motor_send(id, data, sizeof(data));
}

static BusMotorStatus ht_motor_set_pd(uint16_t id, float kp, float kd) {
    if(ht_motor_get_slot(id) == NULL || !isfinite(kp) || !isfinite(kd)) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    return MOTOR_STATUS_UNSUPPORTED;
}

static BusMotorStatus ht_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    BusMotorStatus status;
    uint8_t attempt;

    if(slot == NULL) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(!s_is_initialized) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    if(s_ops->read == NULL || s_ops->now_ms == NULL) {
        if(!slot->has_feedback) {
            return MOTOR_STATUS_NO_FEEDBACK;
        }
        if(feedback != NULL) {
            *feedback = slot->feedback;
        }
        return MOTOR_STATUS_OK;
    }

    for(attempt = 0u; attempt <= s_retry_count; ++attempt) {
        if(s_ops->flush_rx != NULL) {
            s_ops->flush_rx();
        }
        status = ht_motor_request_state(id);
        if(status != MOTOR_STATUS_OK) {
            continue;
        }
        status = ht_motor_wait_feedback(id);
        if(status == MOTOR_STATUS_OK) {
            if(feedback != NULL) {
                *feedback = slot->feedback;
            }
            return MOTOR_STATUS_OK;
        }
    }

    return MOTOR_STATUS_TIMEOUT;
}

static float ht_motor_get_pos(uint16_t id) {
    const HtMotorSlot* slot = ht_motor_get_slot_const(id);
    return (slot != NULL && slot->has_feedback) ? slot->feedback.position : 0.0f;
}

static float ht_motor_get_spd(uint16_t id) {
    const HtMotorSlot* slot = ht_motor_get_slot_const(id);
    return (slot != NULL && slot->has_feedback) ? slot->feedback.speed : 0.0f;
}

static float ht_motor_get_tor(uint16_t id) {
    const HtMotorSlot* slot = ht_motor_get_slot_const(id);
    return (slot != NULL && slot->has_feedback) ? slot->feedback.torque : 0.0f;
}

static BusMotorStatus ht_motor_stop(uint16_t id) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    const uint8_t data[3] = { 0x01u, 0x00u, 0x00u };
    BusMotorStatus status;

    if(slot == NULL) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    status = ht_motor_send(id, data, sizeof(data));
    if(status == MOTOR_STATUS_OK) {
        slot->mode = (uint8_t)HT_MOTOR_MODE_STOP;
    }
    return status;
}

static BusMotorStatus ht_motor_brake(uint16_t id) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    const uint8_t data[3] = { 0x01u, 0x00u, 0x0Fu };
    BusMotorStatus status;

    if(slot == NULL) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    status = ht_motor_send(id, data, sizeof(data));
    if(status == MOTOR_STATUS_OK) {
        slot->mode = (uint8_t)HT_MOTOR_MODE_BRAKE;
    }
    return status;
}

BusMotorStatus ht_motor_set_motion(uint16_t id,
    float position,
    float speed,
    float torque,
    float kp,
    float kd) {
    if(ht_motor_get_slot(id) == NULL || !isfinite(position) || !isfinite(speed) ||
       !isfinite(torque) || !isfinite(kp) || !isfinite(kd)) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    return MOTOR_STATUS_UNSUPPORTED;
}

BusMotorStatus ht_motor_group_set_pvt_raw(uint16_t first_id,
    const HtMotorGroupPvtRaw* commands,
    uint8_t count,
    uint8_t query_id) {
    (void)first_id;
    (void)query_id;
    if(commands == NULL || count == 0u) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    return MOTOR_STATUS_UNSUPPORTED;
}

BusMotorStatus ht_motor_group_set_motion_raw(uint16_t first_id,
    const HtMotorGroupMitRaw* commands,
    uint8_t count,
    uint8_t query_id) {
    (void)first_id;
    (void)query_id;
    if(commands == NULL || count == 0u) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    return MOTOR_STATUS_UNSUPPORTED;
}

BusMotorStatus ht_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t* data,
    uint8_t len,
    BusMotorFeedback* feedback) {
    uint16_t id;
    HtMotorSlot* slot;
    int16_t raw_position;
    int16_t raw_speed;
    int16_t raw_torque;

    if(data == NULL || len > HT_MOTOR_MAX_FRAME_LEN) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(len < 8u || data[0] != HT_CAN_REPLY_I16_3 || data[1] != HT_CAN_REG_POSITION) {
        return MOTOR_STATUS_NO_FEEDBACK;
    }

    id = ht_motor_get_reply_source_id(frame_id);
    slot = ht_motor_get_slot(id);
    if(slot == NULL) {
        return MOTOR_STATUS_ID_MISMATCH;
    }

    raw_position = ht_motor_read_i16_le(&data[2]);
    raw_speed = ht_motor_read_i16_le(&data[4]);
    raw_torque = ht_motor_read_i16_le(&data[6]);

    slot->feedback.id = id;
    slot->feedback.valid = BUS_MOTOR_FEEDBACK_POSITION |
                           BUS_MOTOR_FEEDBACK_VELOCITY |
                           BUS_MOTOR_FEEDBACK_TORQUE;
    slot->feedback.position = ht_motor_position_from_raw(raw_position);
    slot->feedback.velocity = ht_motor_speed_from_raw(raw_speed);
    slot->feedback.torque = ht_motor_torque_from_raw(slot, raw_torque);
    slot->feedback.error_code = 0u;
    slot->has_feedback = true;

    if(feedback != NULL) {
        *feedback = slot->feedback;
    }
    return MOTOR_STATUS_OK;
}

BusMotorStatus ht_motor_request_feedback(uint16_t id) {
    if(ht_motor_get_slot(id) == NULL) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    return ht_motor_request_state(id);
}

static HtMotorSlot* ht_motor_get_slot(uint16_t id) {
    if(id == 0u || id > HT_MOTOR_MAX_ID) {
        return NULL;
    }
    return &s_slots[id - 1u];
}

static const HtMotorSlot* ht_motor_get_slot_const(uint16_t id) {
    if(id == 0u || id > HT_MOTOR_MAX_ID) {
        return NULL;
    }
    return &s_slots[id - 1u];
}

static void ht_motor_reset_slot(HtMotorSlot* slot, uint16_t id) {
    if(slot == NULL) {
        return;
    }
    memset(slot, 0, sizeof(*slot));
    slot->id = id;
    slot->model = HT_MOTOR_MODEL_UNKNOWN;
    slot->feedback.id = id;
}

/* HighTorque examples use 0x8000 | id so control/query frames use extended ID. */
static uint32_t ht_motor_make_command_id(uint16_t id) {
    return HT_MOTOR_REPLY_REQUEST_FLAG | (uint32_t)(id & 0x7Fu);
}

static uint16_t ht_motor_get_reply_source_id(uint32_t frame_id) {
    uint16_t source_id = (uint16_t)((frame_id >> 8) & 0x7Fu);
    if(source_id != 0u) {
        return source_id;
    }
    return (uint16_t)(frame_id & 0x7Fu);
}

static BusMotorStatus ht_motor_send(uint16_t id, const uint8_t* data, uint8_t len) {
    if(!s_is_initialized) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(s_ops == NULL || s_ops->send == NULL) {
        return MOTOR_STATUS_PORT_ERROR;
    }
    if(id == 0u || id > HT_MOTOR_MAX_ID || data == NULL || len == 0u || len > HT_MOTOR_MAX_FRAME_LEN) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return s_ops->send(ht_motor_make_command_id(id), data, len)
        ? MOTOR_STATUS_OK
        : MOTOR_STATUS_PORT_ERROR;
}

static BusMotorStatus ht_motor_request_state(uint16_t id) {
    const uint8_t data[8] = { HT_CAN_CMD_READ_I16_3, HT_CAN_REG_POSITION, 0u, 0u, 0u, 0u, 0u, 0u };
    return ht_motor_send(id, data, sizeof(data));
}

static BusMotorStatus ht_motor_wait_feedback(uint16_t id) {
    uint32_t start_ms;
    uint32_t frame_id;
    uint8_t data[HT_MOTOR_MAX_FRAME_LEN];
    uint8_t len;

    if(s_ops == NULL || s_ops->read == NULL || s_ops->now_ms == NULL) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    start_ms = s_ops->now_ms();
    while((uint32_t)(s_ops->now_ms() - start_ms) < s_timeout_ms) {
        len = sizeof(data);
        if(s_ops->read(&frame_id, data, &len)) {
            BusMotorStatus status = ht_motor_parse_feedback_frame(frame_id, data, len, NULL);
            if(status == MOTOR_STATUS_OK && ht_motor_get_reply_source_id(frame_id) == id) {
                return MOTOR_STATUS_OK;
            }
        }
    }
    return MOTOR_STATUS_TIMEOUT;
}

static int16_t ht_motor_read_i16_le(const uint8_t* data) {
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void ht_motor_write_i16_le(uint8_t* data, int16_t value) {
    uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)(raw & 0xFFu);
    data[1] = (uint8_t)((raw >> 8) & 0xFFu);
}

static int16_t ht_motor_saturate_i16(float value) {
    if(value >= 32767.0f) {
        return INT16_MAX;
    }
    if(value <= -32767.0f) {
        return (int16_t)-32767;
    }
    return (int16_t)value;
}

static int16_t ht_motor_position_to_raw(float position_rad) {
    return ht_motor_saturate_i16(position_rad / HT_MOTOR_TWO_PI * HT_MOTOR_POS_RAW_PER_TURN);
}

static int16_t ht_motor_speed_to_raw(float speed_rad_s) {
    return ht_motor_saturate_i16(speed_rad_s / HT_MOTOR_TWO_PI * HT_MOTOR_VEL_RAW_PER_TURN_PER_SEC);
}

static float ht_motor_position_from_raw(int16_t raw) {
    return ((float)raw / HT_MOTOR_POS_RAW_PER_TURN) * HT_MOTOR_TWO_PI;
}

static float ht_motor_speed_from_raw(int16_t raw) {
    return ((float)raw / HT_MOTOR_VEL_RAW_PER_TURN_PER_SEC) * HT_MOTOR_TWO_PI;
}

static bool ht_motor_torque_coeff(HtMotorModel model, float* k, float* d) {
    if(k == NULL || d == NULL) {
        return false;
    }

    switch(model) {
        case HT_MOTOR_MODEL_4438_30:
            *k = HT_4438_30_TORQUE_K;
            *d = HT_4438_30_TORQUE_D;
            return true;
        case HT_MOTOR_MODEL_5047_36:
            *k = HT_5047_36_TORQUE_K;
            *d = HT_5047_36_TORQUE_D;
            return true;
        default:
            return false;
    }
}

static bool ht_motor_torque_to_raw(const HtMotorSlot* slot, float torque_nm, int16_t* raw) {
    float k;
    float d;

    if(slot == NULL || raw == NULL || !ht_motor_torque_coeff(slot->model, &k, &d)) {
        return false;
    }
    *raw = ht_motor_saturate_i16((torque_nm - d) / k);
    return true;
}

static float ht_motor_torque_from_raw(const HtMotorSlot* slot, int16_t raw) {
    float k;
    float d;

    if(slot != NULL && ht_motor_torque_coeff(slot->model, &k, &d)) {
        return (float)raw * k + d;
    }

    /* Unknown model: preserve a neutral generic scale rather than inventing a correction offset. */
    return (float)raw * 0.005f;
}
