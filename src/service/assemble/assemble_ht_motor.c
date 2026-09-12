#include "assemble.h"

#include "bus_motor/ht_motor.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"

#include <math.h>
#include <stddef.h>

#define HT_TEST_MOTOR_MODEL         "HTDW-4438-30"
#define HT_TEST_MOTOR_ID            2u
#define HT_SECOND_MOTOR_ID          8u
#define HT_FEEDBACK_PERIOD_MS       20u
#define HT_ID2_OLD_FEEDBACK_PERIOD_MS 100u
#define HT_ID2_OLD_TARGET_TX_PERIOD_MS 20u
#define HT_TWO_PI                   6.28318530717958647692f
#define HT_TEST_MOTOR_COUNT         2u

static bool ht_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static void ht_motor_can_rx_callback(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[64],
    void* user);
static uint8_t ht_motor_rx_length(uint32_t data_length);
static float ht_motor_wrap_one_turn(float position);
static float ht_motor_nearest_absolute_target(float current_position,
    float encoder_target);

static volatile bool s_feedback_updated[HT_TEST_MOTOR_COUNT];
static BusMotorFeedback s_feedback[HT_TEST_MOTOR_COUNT];
static volatile float s_raw_position[HT_TEST_MOTOR_COUNT];
static uint32_t s_last_feedback_request_ms[HT_TEST_MOTOR_COUNT];
static uint32_t s_last_target_tx_ms[HT_TEST_MOTOR_COUNT];
static bool s_position_command_sent[HT_TEST_MOTOR_COUNT];
static const uint16_t s_ht_motor_ids[HT_TEST_MOTOR_COUNT] = { HT_TEST_MOTOR_ID, HT_SECOND_MOTOR_ID };

static uint8_t ht_motor_index_by_id(uint16_t id) {
    uint8_t i;

    for(i = 0u; i < HT_TEST_MOTOR_COUNT; ++i) {
        if(s_ht_motor_ids[i] == id) {
            return i;
        }
    }
    return HT_TEST_MOTOR_COUNT;
}

static const BusMotorPortOps ht_motor_ops = {
    .send = ht_motor_can_send,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
};

SystemStatus assemble_ht_motor(void) {
    BusMotorConfig config = {
        .ops = &ht_motor_ops,
        .timeout_ms = 20u,
        .retry_count = 0u,
        .driver_config = NULL,
    };
    BusMotorStatus status;
    uint8_t i;

    BspCanStatus can_status;

    log_info("HT begin");

    can_status = can_register_rx_callback(&hfdcan1, ht_motor_can_rx_callback, NULL);
    if(can_status != STM32_HAL_CAN_OK) {
        log_error("HT rx callback: %s", can_error_code_to_str(can_status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("HT rx ready");

    can_status = can_filter_init();
    if(can_status != STM32_HAL_CAN_OK) {
        log_error("HT CAN filter: %s", can_error_code_to_str(can_status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("HT filter ready");

    can_status = can_start(&hfdcan1);
    if(can_status != STM32_HAL_CAN_OK) {
        log_error("HT CAN start: %s", can_error_code_to_str(can_status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("HT CAN started");
    log_info("HT CAN cfg: IDs 2 and 8 on FDCAN1, ID2 uses old FD-style tx");

    log_info("HT driver init");
    status = ht_motor_instance.init(&config);
    if(status != MOTOR_STATUS_OK) {
        log_error("HT_MOTOR driver init failed: %s", ht_motor_instance.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("HT driver ready");

    log_info("HT stop tx");
    for(i = 0u; i < HT_TEST_MOTOR_COUNT; ++i) {
        status = ht_motor_instance.stop(s_ht_motor_ids[i]);
        if(status != MOTOR_STATUS_OK) {
            log_error("HT_MOTOR stop id=%u failed: %s",
                      s_ht_motor_ids[i],
                      ht_motor_instance.status_str(status));
            return SYSTEM_STATUS_ERROR;
        }
    }
    log_info("HT waiting for encoder feedback");

    log_info("HT ready: %s ids=%u,%u on FDCAN1",
             HT_TEST_MOTOR_MODEL,
             HT_TEST_MOTOR_ID,
             HT_SECOND_MOTOR_ID);
    return SYSTEM_STATUS_OK;
}

void assemble_ht_motor_process(void) {
    uint32_t now = HAL_GetTick();
    BusMotorStatus status;
    uint8_t i;

    for(i = 0u; i < HT_TEST_MOTOR_COUNT; ++i) {
        uint32_t period_ms = s_ht_motor_ids[i] == HT_TEST_MOTOR_ID
            ? HT_ID2_OLD_FEEDBACK_PERIOD_MS
            : HT_FEEDBACK_PERIOD_MS;

        if((uint32_t)(now - s_last_feedback_request_ms[i]) >= period_ms) {
            s_last_feedback_request_ms[i] = now;
            status = ht_motor_request_feedback(s_ht_motor_ids[i]);
            if(status != MOTOR_STATUS_OK) {
                log_error("HT feedback request %u: %s", s_ht_motor_ids[i], ht_motor_instance.status_str(status));
            }
        }
    }

    for(i = 0u; i < HT_TEST_MOTOR_COUNT; ++i) {
        BusMotorFeedback feedback;
        float raw_position = 0.0f;
        uint32_t primask;
        bool updated = false;

        primask = __get_PRIMASK();
        __disable_irq();
        if(s_feedback_updated[i]) {
            feedback = s_feedback[i];
            raw_position = s_raw_position[i];
            s_feedback_updated[i] = false;
            updated = true;
        }
        if(primask == 0u) {
            __enable_irq();
        }

        if(updated) {
            int32_t position_mrad = (int32_t)(feedback.position * 1000.0f);
            int32_t speed_mrad_s = (int32_t)(feedback.speed * 1000.0f);

            (void)raw_position;
            (void)position_mrad;
            (void)speed_mrad_s;
            // log_info("HT id=%u pos=%ld speed=%ld err=%u",
            //     s_ht_motor_ids[i],
            //     (long)position_mrad,
            //     (long)speed_mrad_s,
            //     feedback.error_code);
        }
    }
}

bool assemble_ht_motor_has_feedback(uint16_t id) {
    uint8_t idx = ht_motor_index_by_id(id);

    return idx < HT_TEST_MOTOR_COUNT && s_feedback[idx].id == id;
}

bool assemble_ht_motor_get_raw_position(uint16_t id, float* position) {
    uint8_t idx = ht_motor_index_by_id(id);
    uint32_t primask;

    if(idx >= HT_TEST_MOTOR_COUNT || position == NULL || s_feedback[idx].id != id) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *position = s_raw_position[idx];
    if(primask == 0u) {
        __enable_irq();
    }

    return true;
}

bool assemble_ht_motor_set_target_position(uint16_t id, float position) {
    uint8_t idx = ht_motor_index_by_id(id);
    BusMotorStatus status;
    float target_position = position;
    float current_position = 0.0f;
    uint32_t now = HAL_GetTick();

    if(idx >= HT_TEST_MOTOR_COUNT) {
        return false;
    }

    if(id == HT_TEST_MOTOR_ID) {
        if(!assemble_ht_motor_get_raw_position(id, &current_position)) {
            return false;
        }
        target_position = ht_motor_nearest_absolute_target(current_position, position);

        if(s_position_command_sent[idx]
            && (uint32_t)(now - s_last_target_tx_ms[idx]) < HT_ID2_OLD_TARGET_TX_PERIOD_MS) {
            return true;
        }
    }

    if(!s_position_command_sent[idx]) {
        status = ht_motor_instance.enable(id);
        if(status != MOTOR_STATUS_OK) {
            log_error("HT enable failed id=%u: %s", id, ht_motor_instance.status_str(status));
            return false;
        }
        s_position_command_sent[idx] = true;

        if(id == HT_TEST_MOTOR_ID) {
            log_info("HT old ID2 angle target start encoder=%ld absolute=%ld",
                     (long)(ht_motor_wrap_one_turn(position) * 1000.0f),
                     (long)(target_position * 1000.0f));
        }
    }

    status = ht_motor_instance.set_pos(id, target_position);
    if(status != MOTOR_STATUS_OK) {
        log_error("HT target failed id=%u: %s", id, ht_motor_instance.status_str(status));
        return false;
    }
    s_last_target_tx_ms[idx] = now;

    return true;
}

bool assemble_ht_motor_set_target_position_speed(uint16_t id, float position, float speed) {
    uint8_t idx = ht_motor_index_by_id(id);
    BusMotorStatus status;

    if(idx >= HT_TEST_MOTOR_COUNT) {
        return false;
    }

    if(!s_position_command_sent[idx]) {
        status = ht_motor_instance.enable(id);
        if(status != MOTOR_STATUS_OK) {
            log_error("HT enable failed id=%u: %s", id, ht_motor_instance.status_str(status));
            return false;
        }
        s_position_command_sent[idx] = true;
    }

    status = ht_motor_instance.set_pos_vel(id, position, speed);
    if(status != MOTOR_STATUS_OK) {
        log_error("HT target pos_vel failed id=%u: %s", id, ht_motor_instance.status_str(status));
        return false;
    }

    return true;
}

bool assemble_ht_motor_set_target_speed(uint16_t id, float speed) {
    uint8_t idx = ht_motor_index_by_id(id);
    BusMotorStatus status;

    if(idx >= HT_TEST_MOTOR_COUNT) {
        return false;
    }

    if(!s_position_command_sent[idx]) {
        status = ht_motor_instance.enable(id);
        if(status != MOTOR_STATUS_OK) {
            log_error("HT enable failed id=%u: %s", id, ht_motor_instance.status_str(status));
            return false;
        }
        s_position_command_sent[idx] = true;
    }

    status = ht_motor_instance.set_spd(id, speed);
    if(status != MOTOR_STATUS_OK) {
        log_error("HT speed failed id=%u: %s", id, ht_motor_instance.status_str(status));
        return false;
    }

    return true;
}

static bool ht_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send(&hfdcan1, id, data, len) == STM32_HAL_CAN_OK;
}

static void ht_motor_can_rx_callback(FDCAN_HandleTypeDef* hcan,
                                     const FDCAN_RxHeaderTypeDef* header,
                                     const uint8_t data[64],
                                     void* user) {
    uint8_t len;
    BusMotorFeedback feedback;
    uint8_t idx;

    (void)hcan;
    (void)user;
    if(header == NULL) {
        return;
    }

    len = ht_motor_rx_length(header->DataLength);
    if(len != 0u) {
        if(ht_motor_parse_feedback_frame(header->Identifier, data, len, &feedback) == MOTOR_STATUS_OK) {
            idx = ht_motor_index_by_id(feedback.id);
            if(idx < HT_TEST_MOTOR_COUNT) {
                s_raw_position[idx] = feedback.position;
                feedback.position = ht_motor_wrap_one_turn(feedback.position);
                s_feedback[idx] = feedback;
                s_feedback_updated[idx] = true;
            }
        }
    }
}

static uint8_t ht_motor_rx_length(uint32_t data_length) {
    static const uint8_t lengths[16] = {
        0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u,
        8u, 12u, 16u, 20u, 24u, 32u, 48u, 64u
    };
    uint32_t dlc = data_length;

    return dlc < 16u ? lengths[dlc] : 0u;
}

static float ht_motor_wrap_one_turn(float position) {
    float wrapped = fmodf(position, HT_TWO_PI);

    if(wrapped < 0.0f) {
        wrapped += HT_TWO_PI;
    }
    return wrapped;
}

static float ht_motor_nearest_absolute_target(float current_position,
                                               float encoder_target) {
    float current_wrapped = ht_motor_wrap_one_turn(current_position);
    float target_wrapped = ht_motor_wrap_one_turn(encoder_target);
    float delta = target_wrapped - current_wrapped;

    if(delta > (HT_TWO_PI * 0.5f)) {
        delta -= HT_TWO_PI;
    }
    else if(delta < -(HT_TWO_PI * 0.5f)) {
        delta += HT_TWO_PI;
    }
    return current_position + delta;
}
