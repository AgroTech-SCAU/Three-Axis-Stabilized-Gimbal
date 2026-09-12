#if 0 /* Reference only: ID2 old HT test logic has been merged into assemble_ht_motor.c. */
#include "assemble.h"

#include "bus_motor/ht_motor.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"

#include <math.h>

#define HT_TEST_MOTOR_MODEL         "HTDW-4438-30"
#define HT_TEST_MOTOR_ID            2u
#define HT_TEST_TARGET_ENCODER_RAD  0.0f
#define HT_FEEDBACK_PERIOD_MS       100u
#define HT_FEEDBACK_TIMEOUT_MS      500u
#define HT_TWO_PI                   6.28318530717958647692f

static bool ht_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static void ht_motor_can_rx_callback(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[64],
    void* user);
static uint8_t ht_motor_rx_length(uint32_t data_length);
static float ht_motor_wrap_one_turn(float position);
static float ht_motor_nearest_absolute_target(float current_position,
    float encoder_target);

static volatile bool s_feedback_updated = false;
static BusMotorFeedback s_feedback;
static volatile float s_raw_position = 0.0f;
static volatile uint32_t s_last_feedback_ms = 0u;
static uint32_t s_last_feedback_request_ms = 0u;
static uint32_t s_last_feedback_warn_ms = 0u;
static bool s_process_started = false;
static bool s_first_request_logged = false;
static bool s_position_command_sent = false;
static volatile uint32_t s_rx_frame_count = 0u;
static volatile uint32_t s_last_rx_id = 0u;

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
    log_info("HT CAN cfg: FD no-BRS, nominal=1M data=1M");

    log_info("HT driver init");
    status = ht_motor_instance.init(&config);
    if(status != MOTOR_STATUS_OK) {
        log_error("HT_MOTOR driver init failed: %s", ht_motor_instance.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("HT driver ready");

    log_info("HT stop tx");
    status = ht_motor_instance.stop(HT_TEST_MOTOR_ID);
    if(status != MOTOR_STATUS_OK) {
        log_error("HT_MOTOR stop failed: %s", ht_motor_instance.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("HT waiting for encoder feedback");

    log_info("HT ready: %s id=%u", HT_TEST_MOTOR_MODEL, HT_TEST_MOTOR_ID);
    return SYSTEM_STATUS_OK;
}

void assemble_ht_motor_process(void) {
    uint32_t now = HAL_GetTick();
    BusMotorFeedback feedback;
    BusMotorStatus status;
    FDCAN_ErrorCountersTypeDef error_counters;
    uint32_t primask;
    float raw_position = 0.0f;
    bool updated = false;
    bool can_error_passive = false;

    if(!s_process_started) {
        s_process_started = true;
        log_info("HT feedback task started");
    }

    if(HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
        can_error_passive = error_counters.TxErrorCnt >= 128u;
    }

    if(!can_error_passive
        && (uint32_t)(now - s_last_feedback_request_ms) >= HT_FEEDBACK_PERIOD_MS) {
        s_last_feedback_request_ms = now;
        if(!s_first_request_logged) {
            log_info("HT feedback query tx");
        }
        status = ht_motor_request_feedback(HT_TEST_MOTOR_ID);
        if(status != MOTOR_STATUS_OK) {
            log_error("HT feedback request: %s", ht_motor_instance.status_str(status));
        }
        else if(!s_first_request_logged) {
            s_first_request_logged = true;
            log_info("HT feedback query sent");
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if(s_feedback_updated) {
        feedback = s_feedback;
        raw_position = s_raw_position;
        s_feedback_updated = false;
        updated = true;
    }
    if(primask == 0u) {
        __enable_irq();
    }

    if(updated) {
        int32_t position_mrad = (int32_t)(feedback.position * 1000.0f);
        int32_t speed_mrad_s = (int32_t)(feedback.speed * 1000.0f);

        if(!s_position_command_sent) {
            float absolute_target = ht_motor_nearest_absolute_target(
                raw_position, HT_TEST_TARGET_ENCODER_RAD);

            status = ht_motor_instance.enable(HT_TEST_MOTOR_ID);
            if(status == MOTOR_STATUS_OK) {
                status = ht_motor_instance.set_pos(HT_TEST_MOTOR_ID, absolute_target);
            }
            if(status != MOTOR_STATUS_OK) {
                log_error("HT position start failed: %s",
                    ht_motor_instance.status_str(status));
            }
            else {
                s_position_command_sent = true;
                log_info("HT encoder target=%ld absolute=%ld",
                    (long)(ht_motor_wrap_one_turn(HT_TEST_TARGET_ENCODER_RAD) * 1000.0f),
                    (long)(absolute_target * 1000.0f));
            }
        }

        log_info("HT pos=%ld speed=%ld err=%u",
            (long)position_mrad,
            (long)speed_mrad_s,
            feedback.error_code);
    }
    else if((uint32_t)(now - s_last_feedback_ms) >= HT_FEEDBACK_TIMEOUT_MS
        && (uint32_t)(now - s_last_feedback_warn_ms) >= 1000u) {
        FDCAN_ProtocolStatusTypeDef protocol_status;
        FDCAN_ErrorCountersTypeDef error_counters;

        s_last_feedback_warn_ms = now;
        log_warn("HT feedback timeout: id=%u", HT_TEST_MOTOR_ID);
        if(HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK
            && HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
            log_warn("HT CAN lec=%lu busoff=%lu txerr=%lu rxerr=%lu free=%lu",
                (unsigned long)protocol_status.LastErrorCode,
                (unsigned long)protocol_status.BusOff,
                (unsigned long)error_counters.TxErrorCnt,
                (unsigned long)error_counters.RxErrorCnt,
                (unsigned long)HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1));
        }
        log_warn("HT RX frames=%lu last_id=0x%lX",
            (unsigned long)s_rx_frame_count,
            (unsigned long)s_last_rx_id);
    }
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

    (void)hcan;
    (void)user;
    if(header == NULL) {
        return;
    }

    s_rx_frame_count++;
    s_last_rx_id = header->Identifier;

    len = ht_motor_rx_length(header->DataLength);
    if(len != 0u) {
        if(ht_motor_parse_feedback_frame(header->Identifier, data, len, &feedback) == MOTOR_STATUS_OK
            && feedback.id == HT_TEST_MOTOR_ID) {
            s_raw_position = feedback.position;
            feedback.position = ht_motor_wrap_one_turn(feedback.position);
            s_feedback = feedback;
            s_last_feedback_ms = HAL_GetTick();
            s_feedback_updated = true;
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
#endif
