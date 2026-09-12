#include "ht_motor.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define HT_MOTOR_DEFAULT_TIMEOUT_MS 20u
#define HT_MOTOR_REPLY_REQUEST_FLAG 0x8000u
#define HT_MOTOR_BROADCAST_ID       0x7Fu
#define HT_MOTOR_NOP                0x50u
#define HT_MOTOR_TWO_PI             6.28318530717958647692f

#define HT_MOTOR_REG_MODE           0x00u
#define HT_MOTOR_REG_POSITION       0x01u
#define HT_MOTOR_REG_SPEED          0x02u
#define HT_MOTOR_REG_TORQUE         0x03u
#define HT_MOTOR_REG_ERROR          0x0Fu
#define HT_MOTOR_REG_TARGET_POS     0x20u
#define HT_MOTOR_REG_TARGET_SPEED   0x21u
#define HT_MOTOR_REG_FEEDFORWARD    0x22u
#define HT_MOTOR_REG_KP             0x23u
#define HT_MOTOR_REG_KD             0x24u

#define HT_MOTOR_CMD_WRITE_I8_1     0x01u
#define HT_MOTOR_CMD_WRITE_F32_1    0x0Du
#define HT_MOTOR_CMD_WRITE_F32_2    0x0Eu
#define HT_MOTOR_CMD_WRITE_F32_3    0x0Fu
#define HT_MOTOR_CMD_READ_I8_1      0x11u
#define HT_MOTOR_CMD_READ_F32_3     0x1Fu

#define HT_MOTOR_GROUP_PVT_1_10_ID  0x8090u
#define HT_MOTOR_GROUP_PVT_11_20_ID 0x8091u
#define HT_MOTOR_GROUP_PVT_21_30_ID 0x8092u
#define HT_MOTOR_GROUP_MIT_1_6_ID   0x8093u

typedef struct {
    uint16_t id;
    uint8_t mode;
    bool has_feedback;
    BusMotorFeedback feedback;
} HtMotorSlot;

static const BusMotorPortOps* s_ops = 0;
static bool s_is_initialized = false;
static uint32_t s_timeout_ms = HT_MOTOR_DEFAULT_TIMEOUT_MS;
static uint8_t s_retry_count = 0u;
static HtMotorSlot s_slots[HT_MOTOR_MAX_ID];

// ! ========================= 私 有 函 数 声 明 ========================= ! //

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
static bool ht_motor_is_valid_mode(BusMotorMode mode);
static uint32_t ht_motor_make_request_id(uint16_t id, bool need_reply);
static uint16_t ht_motor_get_reply_source_id(uint32_t frame_id);
static BusMotorStatus ht_motor_send(uint32_t frame_id, const uint8_t* data, uint8_t len);
static BusMotorStatus ht_motor_write_i8(uint16_t id, uint8_t reg, uint8_t value);
static BusMotorStatus ht_motor_write_float(uint16_t id, uint8_t reg, float value);
static BusMotorStatus ht_motor_write_float2(uint16_t id, uint8_t reg, float value0, float value1);
static BusMotorStatus ht_motor_request_state(uint16_t id);
static BusMotorStatus ht_motor_wait_feedback(uint16_t id);
static uint8_t ht_motor_type_size(uint8_t type);
static float ht_motor_read_float_le(const uint8_t* data);
static int32_t ht_motor_read_i32_le(const uint8_t* data);
static int16_t ht_motor_read_i16_le(const uint8_t* data);
static void ht_motor_write_i16_le(uint8_t* data, int16_t value);
static uint8_t ht_motor_fd_length(uint8_t payload_len);
static uint32_t ht_motor_group_pvt_frame_id(uint16_t first_id);
static uint32_t ht_motor_group_motion_frame_id(uint16_t first_id);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

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

/**
 * @brief 初始化高擎电机实例
 */
static BusMotorStatus ht_motor_init(const BusMotorConfig* config) {
    uint16_t id;

    if(config == 0 || config->ops == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(config->ops->send == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    s_ops = config->ops;
    s_timeout_ms = config->timeout_ms == 0u
        ? HT_MOTOR_DEFAULT_TIMEOUT_MS
        : config->timeout_ms;
    s_retry_count = config->retry_count;

    for(id = 1u; id <= HT_MOTOR_MAX_ID; ++id) {
        ht_motor_reset_slot(&s_slots[id - 1u], id);
    }

    s_is_initialized = true;
    return MOTOR_STATUS_OK;
}

/**
 * @brief 将状态码转换为常量字符串
 */
static const char* ht_motor_status_str(BusMotorStatus status) {
    switch(status) {
#define X(name, value) case MOTOR_STATUS_##name: return #name;
        MOTOR_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}

/**
 * @brief 将模式值转换为常量字符串
 */
static const char* ht_motor_mode_str(BusMotorMode mode) {
    switch((HtMotorMode)mode) {
        case HT_MOTOR_MODE_STOP: return "STOP";
        case HT_MOTOR_MODE_ERROR: return "ERROR";
        case HT_MOTOR_MODE_READY_2: return "READY_2";
        case HT_MOTOR_MODE_READY_3: return "READY_3";
        case HT_MOTOR_MODE_READY_4: return "READY_4";
        case HT_MOTOR_MODE_PWM: return "PWM";
        case HT_MOTOR_MODE_VOLTAGE: return "VOLTAGE";
        case HT_MOTOR_MODE_FOC_VOLTAGE: return "FOC_VOLTAGE";
        case HT_MOTOR_MODE_DQ_VOLTAGE: return "DQ_VOLTAGE";
        case HT_MOTOR_MODE_DQ_CURRENT: return "DQ_CURRENT";
        case HT_MOTOR_MODE_POSITION: return "POSITION";
        case HT_MOTOR_MODE_TIMEOUT: return "TIMEOUT";
        case HT_MOTOR_MODE_ZERO_SPEED: return "ZERO_SPEED";
        case HT_MOTOR_MODE_RANGE: return "RANGE";
        case HT_MOTOR_MODE_MEASURE_L: return "MEASURE_L";
        case HT_MOTOR_MODE_BRAKE: return "BRAKE";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 使能高擎电机，进入位置/运控模式
 */
static BusMotorStatus ht_motor_enable(uint16_t id) {
    return ht_motor_switch_mode(id, HT_MOTOR_MODE_POSITION);
}

/**
 * @brief 失能高擎电机
 */
static BusMotorStatus ht_motor_disable(uint16_t id) {
    return ht_motor_switch_mode(id, HT_MOTOR_MODE_STOP);
}

/**
 * @brief 切换高擎电机模式
 */
static BusMotorStatus ht_motor_switch_mode(uint16_t id, BusMotorMode mode) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    BusMotorStatus status;

    if(slot == 0 || ht_motor_is_valid_mode(mode) == false) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    status = ht_motor_write_i8(id, HT_MOTOR_REG_MODE, (uint8_t)mode);
    if(status == MOTOR_STATUS_OK) {
        slot->mode = (uint8_t)mode;
    }

    return status;
}

/**
 * @brief 设置目标位置，统一接口单位 rad，协议发送单位转
 */
static BusMotorStatus ht_motor_set_pos(uint16_t id, float position) {
    if(ht_motor_get_slot(id) == 0 || isfinite(position) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return ht_motor_write_float(id,
        HT_MOTOR_REG_TARGET_POS,
        position / HT_MOTOR_TWO_PI);
}

/**
 * @brief 设置目标速度，统一接口单位 rad/s，协议发送单位转/秒
 */
static BusMotorStatus ht_motor_set_spd(uint16_t id, float speed) {
    if(ht_motor_get_slot(id) == 0 || isfinite(speed) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return ht_motor_write_float(id,
        HT_MOTOR_REG_TARGET_SPEED,
        speed / HT_MOTOR_TWO_PI);
}

/**
 * @brief 同一子帧设置目标位置和速度
 */
static BusMotorStatus ht_motor_set_pos_vel(uint16_t id, float position, float speed) {
    if(ht_motor_get_slot(id) == 0 || isfinite(position) == 0 || isfinite(speed) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return ht_motor_write_float2(id,
        HT_MOTOR_REG_TARGET_POS,
        position / HT_MOTOR_TWO_PI,
        speed / HT_MOTOR_TWO_PI);
}

/**
 * @brief 设置前馈力矩
 */
static BusMotorStatus ht_motor_set_tor(uint16_t id, float torque) {
    if(ht_motor_get_slot(id) == 0 || isfinite(torque) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return ht_motor_write_float(id, HT_MOTOR_REG_FEEDFORWARD, torque);
}

/**
 * @brief 同一子帧设置位置环 Kp 和 Kd
 */
static BusMotorStatus ht_motor_set_pd(uint16_t id, float kp, float kd) {
    if(ht_motor_get_slot(id) == 0 || isfinite(kp) == 0 || isfinite(kd) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return ht_motor_write_float2(id, HT_MOTOR_REG_KP, kp, kd);
}

/**
 * @brief 主动查询电机位置、速度、力矩和错误码
 */
static BusMotorStatus ht_motor_update_feedback(uint16_t id, BusMotorFeedback* feedback) {
    HtMotorSlot* slot = ht_motor_get_slot(id);
    BusMotorStatus status;
    uint8_t attempt;

    if(slot == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(s_is_initialized == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }

    if(s_ops->read == 0 || s_ops->now_ms == 0) {
        if(slot->has_feedback == false) {
            return MOTOR_STATUS_NO_FEEDBACK;
        }
        if(feedback != 0) {
            *feedback = slot->feedback;
        }
        return MOTOR_STATUS_OK;
    }

    for(attempt = 0u; attempt <= s_retry_count; ++attempt) {
        if(s_ops->flush_rx != 0) {
            s_ops->flush_rx();
        }

        status = ht_motor_request_state(id);
        if(status != MOTOR_STATUS_OK) {
            continue;
        }

        status = ht_motor_wait_feedback(id);
        if(status == MOTOR_STATUS_OK) {
            if(feedback != 0) {
                *feedback = slot->feedback;
            }
            return MOTOR_STATUS_OK;
        }
    }

    return MOTOR_STATUS_TIMEOUT;
}

/**
 * @brief 获取最近位置反馈
 */
static float ht_motor_get_pos(uint16_t id) {
    const HtMotorSlot* slot = ht_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.position;
}

/**
 * @brief 获取最近速度反馈
 */
static float ht_motor_get_spd(uint16_t id) {
    const HtMotorSlot* slot = ht_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.speed;
}

/**
 * @brief 获取最近力矩反馈
 */
static float ht_motor_get_tor(uint16_t id) {
    const HtMotorSlot* slot = ht_motor_get_slot_const(id);

    if(slot == 0 || slot->has_feedback == false) {
        return 0.0f;
    }

    return slot->feedback.torque;
}

/**
 * @brief 停止电机
 */
static BusMotorStatus ht_motor_stop(uint16_t id) {
    return ht_motor_switch_mode(id, HT_MOTOR_MODE_STOP);
}

/**
 * @brief 刹车
 */
static BusMotorStatus ht_motor_brake(uint16_t id) {
    return ht_motor_switch_mode(id, HT_MOTOR_MODE_BRAKE);
}

/**
 * @brief 完整运控指令
 */
BusMotorStatus ht_motor_set_motion(uint16_t id,
    float position,
    float speed,
    float torque,
    float kp,
    float kd) {
    uint8_t data[24];

    if(ht_motor_get_slot(id) == 0
        || isfinite(position) == 0
        || isfinite(speed) == 0
        || isfinite(torque) == 0
        || isfinite(kp) == 0
        || isfinite(kd) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    data[0] = HT_MOTOR_CMD_WRITE_F32_3;
    data[1] = HT_MOTOR_REG_TARGET_POS;
    position /= HT_MOTOR_TWO_PI;
    speed /= HT_MOTOR_TWO_PI;
    memcpy(&data[2], &position, sizeof(position));
    memcpy(&data[6], &speed, sizeof(speed));
    memcpy(&data[10], &torque, sizeof(torque));

    data[14] = HT_MOTOR_CMD_WRITE_F32_2;
    data[15] = HT_MOTOR_REG_KP;
    memcpy(&data[16], &kp, sizeof(kp));
    memcpy(&data[20], &kd, sizeof(kd));

    return ht_motor_send(ht_motor_make_request_id(id, false), data, sizeof(data));
}

/**
 * @brief 一拖多位置、速度、力矩原始控制
 */
BusMotorStatus ht_motor_group_set_pvt_raw(uint16_t first_id,
    const HtMotorGroupPvtRaw* commands,
    uint8_t count,
    uint8_t query_id) {
    uint8_t data[64];
    uint8_t i;
    uint8_t payload_len;
    uint8_t frame_len;
    uint32_t frame_id = ht_motor_group_pvt_frame_id(first_id);

    if(s_is_initialized == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(frame_id == 0u || commands == 0 || count == 0u || count > 10u) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(query_id > HT_MOTOR_MAX_ID) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    payload_len = (uint8_t)(count * 6u);
    for(i = 0u; i < count; ++i) {
        ht_motor_write_i16_le(&data[i * 6u], commands[i].position);
        ht_motor_write_i16_le(&data[i * 6u + 2u], commands[i].speed);
        ht_motor_write_i16_le(&data[i * 6u + 4u], commands[i].torque);
    }

    data[payload_len++] = 0x17u;
    data[payload_len++] = query_id;
    frame_len = ht_motor_fd_length(payload_len);
    while(payload_len < frame_len) {
        data[payload_len++] = HT_MOTOR_NOP;
    }

    return ht_motor_send(frame_id, data, frame_len);
}

/**
 * @brief 一拖多完整运控原始控制
 */
BusMotorStatus ht_motor_group_set_motion_raw(uint16_t first_id,
    const HtMotorGroupMitRaw* commands,
    uint8_t count,
    uint8_t query_id) {
    uint8_t data[64];
    uint8_t i;
    uint8_t payload_len;
    uint8_t frame_len;
    uint32_t frame_id = ht_motor_group_motion_frame_id(first_id);

    if(s_is_initialized == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(frame_id == 0u || commands == 0 || count == 0u || count > 6u) {
        return MOTOR_STATUS_INVALID_PARAM;
    }
    if(query_id > HT_MOTOR_MAX_ID) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    payload_len = (uint8_t)(count * 10u);
    for(i = 0u; i < count; ++i) {
        ht_motor_write_i16_le(&data[i * 10u], commands[i].position);
        ht_motor_write_i16_le(&data[i * 10u + 2u], commands[i].speed);
        ht_motor_write_i16_le(&data[i * 10u + 4u], commands[i].torque);
        ht_motor_write_i16_le(&data[i * 10u + 6u], commands[i].kp);
        ht_motor_write_i16_le(&data[i * 10u + 8u], commands[i].kd);
    }

    data[payload_len++] = 0x17u;
    data[payload_len++] = query_id;
    frame_len = ht_motor_fd_length(payload_len);
    while(payload_len < frame_len) {
        data[payload_len++] = HT_MOTOR_NOP;
    }

    return ht_motor_send(frame_id, data, frame_len);
}

/**
 * @brief 解析高擎回复帧
 */
BusMotorStatus ht_motor_parse_feedback_frame(uint32_t frame_id,
    const uint8_t* data,
    uint8_t len,
    BusMotorFeedback* feedback) {
    HtMotorSlot* slot;
    uint16_t id;
    uint8_t offset = 0u;
    bool updated = false;

    if(data == 0 || len == 0u || len > HT_MOTOR_MAX_FRAME_LEN) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    id = ht_motor_get_reply_source_id(frame_id);
    slot = ht_motor_get_slot(id);
    if(slot == 0) {
        return MOTOR_STATUS_ID_MISMATCH;
    }

    while(offset < len) {
        uint8_t cmd = data[offset++];
        uint8_t operation;
        uint8_t type;
        uint8_t count;
        uint8_t size;
        uint8_t reg;
        uint8_t i;

        if(cmd == HT_MOTOR_NOP) {
            continue;
        }

        operation = (uint8_t)(cmd >> 4);
        type = (uint8_t)((cmd >> 2) & 0x03u);
        count = (uint8_t)(cmd & 0x03u);
        if(operation != 0x02u) {
            return MOTOR_STATUS_ERROR;
        }

        if(count == 0u) {
            if(offset + 2u > len) {
                return MOTOR_STATUS_ERROR;
            }
            count = data[offset++];
        }
        if(offset >= len || count == 0u) {
            return MOTOR_STATUS_ERROR;
        }

        reg = data[offset++];
        size = ht_motor_type_size(type);
        if(size == 0u || (uint16_t)offset + (uint16_t)count * size > len) {
            return MOTOR_STATUS_ERROR;
        }

        for(i = 0u; i < count; ++i) {
            const uint8_t* value = &data[offset + i * size];
            uint8_t current_reg = (uint8_t)(reg + i);
            float decoded = 0.0f;

            if(type == 0u) {
                decoded = (float)(int8_t)value[0];
            }
            else if(type == 1u) {
                decoded = (float)ht_motor_read_i16_le(value);
            }
            else if(type == 2u) {
                decoded = (float)ht_motor_read_i32_le(value);
            }
            else {
                decoded = ht_motor_read_float_le(value);
            }

            switch(current_reg) {
                case HT_MOTOR_REG_MODE:
                    slot->mode = (uint8_t)decoded;
                    break;

                case HT_MOTOR_REG_POSITION:
                    slot->feedback.position = decoded * HT_MOTOR_TWO_PI;
                    updated = true;
                    break;

                case HT_MOTOR_REG_SPEED:
                    slot->feedback.speed = decoded * HT_MOTOR_TWO_PI;
                    updated = true;
                    break;

                case HT_MOTOR_REG_TORQUE:
                    slot->feedback.torque = decoded;
                    updated = true;
                    break;

                case HT_MOTOR_REG_ERROR:
                    slot->feedback.error_code = (uint8_t)decoded;
                    updated = true;
                    break;

                default:
                    break;
            }
        }

        offset = (uint8_t)(offset + count * size);
    }

    slot->feedback.id = id;
    if(updated) {
        slot->has_feedback = true;
    }

    if(feedback != 0) {
        *feedback = slot->feedback;
    }

    return updated ? MOTOR_STATUS_OK : MOTOR_STATUS_NO_FEEDBACK;
}

BusMotorStatus ht_motor_request_feedback(uint16_t id) {
    if(ht_motor_get_slot(id) == 0) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return ht_motor_request_state(id);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static HtMotorSlot* ht_motor_get_slot(uint16_t id) {
    if(id == 0u || id > HT_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static const HtMotorSlot* ht_motor_get_slot_const(uint16_t id) {
    if(id == 0u || id > HT_MOTOR_MAX_ID) {
        return 0;
    }

    return &s_slots[id - 1u];
}

static void ht_motor_reset_slot(HtMotorSlot* slot, uint16_t id) {
    if(slot == 0) {
        return;
    }

    memset(slot, 0, sizeof(*slot));
    slot->id = id;
    slot->feedback.id = id;
}

static bool ht_motor_is_valid_mode(BusMotorMode mode) {
    return mode <= HT_MOTOR_MODE_BRAKE;
}

static uint32_t ht_motor_make_request_id(uint16_t id, bool need_reply) {
    uint32_t frame_id = (uint32_t)(id & 0x7Fu);

    if(need_reply) {
        frame_id |= HT_MOTOR_REPLY_REQUEST_FLAG;
    }

    return frame_id;
}

static uint16_t ht_motor_get_reply_source_id(uint32_t frame_id) {
    uint16_t source_id = (uint16_t)((frame_id >> 8) & 0x7Fu);

    if(source_id != 0u) {
        return source_id;
    }

    return (uint16_t)(frame_id & 0x7Fu);
}

static BusMotorStatus ht_motor_send(uint32_t frame_id, const uint8_t* data, uint8_t len) {
    if(s_is_initialized == false) {
        return MOTOR_STATUS_NOT_INITIALIZE;
    }
    if(s_ops == 0 || s_ops->send == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }
    if(data == 0 || len == 0u || len > HT_MOTOR_MAX_FRAME_LEN) {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    return s_ops->send(frame_id, data, len)
        ? MOTOR_STATUS_OK
        : MOTOR_STATUS_PORT_ERROR;
}

static BusMotorStatus ht_motor_write_i8(uint16_t id, uint8_t reg, uint8_t value) {
    uint8_t data[3];

    data[0] = HT_MOTOR_CMD_WRITE_I8_1;
    data[1] = reg;
    data[2] = value;

    return ht_motor_send(ht_motor_make_request_id(id, false), data, sizeof(data));
}

static BusMotorStatus ht_motor_write_float(uint16_t id, uint8_t reg, float value) {
    uint8_t data[6];

    data[0] = HT_MOTOR_CMD_WRITE_F32_1;
    data[1] = reg;
    memcpy(&data[2], &value, sizeof(value));

    return ht_motor_send(ht_motor_make_request_id(id, false), data, sizeof(data));
}

static BusMotorStatus ht_motor_write_float2(uint16_t id,
    uint8_t reg,
    float value0,
    float value1) {
    uint8_t data[10];

    data[0] = HT_MOTOR_CMD_WRITE_F32_2;
    data[1] = reg;
    memcpy(&data[2], &value0, sizeof(value0));
    memcpy(&data[6], &value1, sizeof(value1));

    return ht_motor_send(ht_motor_make_request_id(id, false), data, sizeof(data));
}

static BusMotorStatus ht_motor_request_state(uint16_t id) {
    uint8_t data[4];

    data[0] = HT_MOTOR_CMD_READ_F32_3;
    data[1] = HT_MOTOR_REG_POSITION;
    data[2] = HT_MOTOR_CMD_READ_I8_1;
    data[3] = HT_MOTOR_REG_ERROR;

    return ht_motor_send(ht_motor_make_request_id(id, true), data, sizeof(data));
}

static BusMotorStatus ht_motor_wait_feedback(uint16_t id) {
    uint32_t start_ms;
    uint32_t frame_id;
    uint8_t data[HT_MOTOR_MAX_FRAME_LEN];
    uint8_t len;

    if(s_ops == 0 || s_ops->read == 0 || s_ops->now_ms == 0) {
        return MOTOR_STATUS_PORT_ERROR;
    }

    start_ms = s_ops->now_ms();
    while((uint32_t)(s_ops->now_ms() - start_ms) < s_timeout_ms) {
        len = sizeof(data);
        if(s_ops->read(&frame_id, data, &len)) {
            BusMotorStatus status = ht_motor_parse_feedback_frame(frame_id, data, len, 0);

            if(status == MOTOR_STATUS_OK && ht_motor_get_reply_source_id(frame_id) == id) {
                return MOTOR_STATUS_OK;
            }
        }
    }

    return MOTOR_STATUS_TIMEOUT;
}

static uint8_t ht_motor_type_size(uint8_t type) {
    switch(type) {
        case 0u: return 1u;
        case 1u: return 2u;
        case 2u: return 4u;
        case 3u: return 4u;
        default: return 0u;
    }
}

static float ht_motor_read_float_le(const uint8_t* data) {
    float value;

    memcpy(&value, data, sizeof(value));
    return value;
}

static int32_t ht_motor_read_i32_le(const uint8_t* data) {
    uint32_t value = (uint32_t)data[0]
        | ((uint32_t)data[1] << 8)
        | ((uint32_t)data[2] << 16)
        | ((uint32_t)data[3] << 24);

    return (int32_t)value;
}

static int16_t ht_motor_read_i16_le(const uint8_t* data) {
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    return (int16_t)value;
}

static void ht_motor_write_i16_le(uint8_t* data, int16_t value) {
    uint16_t raw = (uint16_t)value;

    data[0] = (uint8_t)(raw & 0xFFu);
    data[1] = (uint8_t)((raw >> 8) & 0xFFu);
}

static uint8_t ht_motor_fd_length(uint8_t payload_len) {
    static const uint8_t lengths[] = {
        1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u,
        12u, 16u, 20u, 24u, 32u, 48u, 64u
    };
    uint8_t i;

    for(i = 0u; i < sizeof(lengths); ++i) {
        if(payload_len <= lengths[i]) {
            return lengths[i];
        }
    }

    return 64u;
}

static uint32_t ht_motor_group_pvt_frame_id(uint16_t first_id) {
    switch(first_id) {
        case 1u: return HT_MOTOR_GROUP_PVT_1_10_ID;
        case 11u: return HT_MOTOR_GROUP_PVT_11_20_ID;
        case 21u: return HT_MOTOR_GROUP_PVT_21_30_ID;
        default: return 0u;
    }
}

static uint32_t ht_motor_group_motion_frame_id(uint16_t first_id) {
    if(first_id == 0u || first_id > 25u || ((first_id - 1u) % 6u) != 0u) {
        return 0u;
    }

    return HT_MOTOR_GROUP_MIT_1_6_ID + (uint32_t)((first_id - 1u) / 6u);
}
