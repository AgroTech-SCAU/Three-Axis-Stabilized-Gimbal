#include "assemble.h"

#include "bus_motor/ht_motor.h"
#include "delay.h"
#include "log.h"
#include "stm32_hal_can.h"

#include <math.h>
#include <stddef.h>

#define HT_PITCH_MOTOR_ID          2u
#define HT_ROLL_MOTOR_ID           8u
#define HT_FEEDBACK_PERIOD_MS      20u
#define HT_DISCOVERY_PERIOD_MS      250u
#define HT_DISCOVERY_MAX_PERIOD_MS  4000u
#define HT_TARGET_TX_PERIOD_MS     10u
#define HT_FEEDBACK_STALE_TIMEOUT_MS 200u
#define HT_CAN_DIAG_LOG_PERIOD_MS   1000u
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
static bool ht_feedback_is_fresh(uint8_t idx, uint32_t now_ms);
static void ht_handle_can_recovery(uint32_t now_ms);
static void ht_log_can_diagnostic(const char* action, uint16_t id, BusMotorStatus motor_status, uint32_t now_ms);

static volatile bool s_feedback_updated[HT_TEST_MOTOR_COUNT];
static BusMotorFeedback s_feedback[HT_TEST_MOTOR_COUNT];
static volatile float s_raw_position[HT_TEST_MOTOR_COUNT];
static volatile bool s_feedback_seen[HT_TEST_MOTOR_COUNT];
static volatile uint32_t s_last_feedback_seen_ms[HT_TEST_MOTOR_COUNT];
static uint32_t s_last_feedback_request_ms[HT_TEST_MOTOR_COUNT];
static uint32_t s_last_target_tx_ms[HT_TEST_MOTOR_COUNT];
static bool s_position_command_sent[HT_TEST_MOTOR_COUNT];
static bool s_stop_command_sent[HT_TEST_MOTOR_COUNT];
static bool s_discovery_logged[HT_TEST_MOTOR_COUNT];
static uint32_t s_discovery_period_ms[HT_TEST_MOTOR_COUNT];
static uint32_t s_last_can_diag_log_ms;
static uint32_t s_seen_can_recovery_count;
static const uint16_t s_ht_motor_ids[HT_TEST_MOTOR_COUNT] = { HT_PITCH_MOTOR_ID, HT_ROLL_MOTOR_ID };
static const HtMotorDeviceProfile s_ht_motor_profiles[HT_TEST_MOTOR_COUNT] = {
    { HT_PITCH_MOTOR_ID, HT_MOTOR_MODEL_4438_30 },
    { HT_ROLL_MOTOR_ID, HT_MOTOR_MODEL_5047_36 },
};
static const HtMotorDriverConfig s_ht_driver_config = {
    .profiles = s_ht_motor_profiles,
    .count = HT_TEST_MOTOR_COUNT,
};

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
        .driver_config = &s_ht_driver_config,
    };
    BusMotorStatus status;
    uint8_t i;

    BspCanStatus can_status;

    log_info("HT begin");

    for(i = 0u; i < HT_TEST_MOTOR_COUNT; ++i) {
        s_feedback_updated[i] = false;
        s_feedback[i] = (BusMotorFeedback){ 0 };
        s_raw_position[i] = 0.0f;
        s_feedback_seen[i] = false;
        s_last_feedback_seen_ms[i] = 0u;
        s_last_feedback_request_ms[i] = HAL_GetTick();
        s_last_target_tx_ms[i] = 0u;
        s_position_command_sent[i] = false;
        s_stop_command_sent[i] = false;
        s_discovery_logged[i] = false;
        s_discovery_period_ms[i] = HT_DISCOVERY_PERIOD_MS;
    }
    s_last_can_diag_log_ms = 0u;
    s_seen_can_recovery_count = can_get_recovery_count(&hfdcan1);

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
    log_info("HT CAN cfg: Classic CAN IDs 2(4438-30),8(5047-36) on FDCAN1");

    log_info("HT driver init");
    status = ht_motor_instance.init(&config);
    if(status != MOTOR_STATUS_OK) {
        log_error("HT_MOTOR driver init failed: %s", ht_motor_instance.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }
    log_info("HT driver ready");

    /*
     * Safe-stop is deliberately deferred to the runtime loop. This gives the
     * motor power rail/transceivers the remaining boot-settle time and avoids
     * entering Bus-Off before the recovery service is running.
     */
    log_info("HT safe stop deferred to runtime; waiting for encoder feedback");

    log_info("HT ready: Classic CAN pitch=4438-30(id=%u) roll=5047-36(id=%u) on FDCAN1",
             HT_PITCH_MOTOR_ID,
             HT_ROLL_MOTOR_ID);
    return SYSTEM_STATUS_OK;
}

void assemble_ht_motor_process(void) {
    uint32_t now = HAL_GetTick();
    BusMotorStatus status;
    uint8_t i;

    ht_handle_can_recovery(now);

    for(i = 0u; i < HT_TEST_MOTOR_COUNT; ++i) {
        bool fresh = ht_feedback_is_fresh(i, now);

        if(s_feedback_seen[i] && !fresh) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            s_feedback_seen[i] = false;
            s_position_command_sent[i] = false;
            s_stop_command_sent[i] = false;
            s_discovery_period_ms[i] = HT_DISCOVERY_PERIOD_MS;
            if(primask == 0u) {
                __enable_irq();
            }
            fresh = false;
            if((uint32_t)(now - s_last_can_diag_log_ms) >= HT_CAN_DIAG_LOG_PERIOD_MS) {
                s_last_can_diag_log_ms = now;
                log_warn("HT feedback stale id=%u age=%lu ms; return to discovery",
                         s_ht_motor_ids[i],
                         (unsigned long)(now - s_last_feedback_seen_ms[i]));
            }
        }

        /*
         * Never hammer an unready CAN bus at the normal 20 ms feedback rate.
         * Before the first valid reply (or after feedback goes stale), send a
         * single discovery query only every 250 ms.  This gives motor power
         * rails/transceivers time to boot and prevents application-level retry
         * traffic from driving TEC to Bus-Off when no node is ready to ACK.
         */
        if(!fresh) {
            if(can_tx_ready(&hfdcan1) &&
               (uint32_t)(now - s_last_feedback_request_ms[i]) >= s_discovery_period_ms[i]) {
                uint32_t next_period;

                s_last_feedback_request_ms[i] = now;
                status = ht_motor_request_feedback(s_ht_motor_ids[i]);
                if(status != MOTOR_STATUS_OK) {
                    ht_log_can_diagnostic("discovery", s_ht_motor_ids[i], status, now);
                }
                else {
                    if(!s_discovery_logged[i]) {
                        s_discovery_logged[i] = true;
                        log_info("HT discovery id=%u query sent; waiting first feedback",
                                 s_ht_motor_ids[i]);
                    }

                    /* Exponential backoff while the node is still silent. */
                    next_period = s_discovery_period_ms[i] << 1u;
                    if(next_period > HT_DISCOVERY_MAX_PERIOD_MS ||
                       next_period < s_discovery_period_ms[i]) {
                        next_period = HT_DISCOVERY_MAX_PERIOD_MS;
                    }
                    s_discovery_period_ms[i] = next_period;
                }
            }
            continue;
        }

        /* Only command stop after the node has proved it is alive. */
        if(can_tx_ready(&hfdcan1) && !s_stop_command_sent[i]) {
            status = ht_motor_instance.stop(s_ht_motor_ids[i]);
            if(status != MOTOR_STATUS_OK) {
                ht_log_can_diagnostic("safe stop", s_ht_motor_ids[i], status, now);
                continue;
            }
            s_stop_command_sent[i] = true;
            s_last_feedback_request_ms[i] = now;
            continue;
        }

        if(can_tx_ready(&hfdcan1) &&
           (uint32_t)(now - s_last_feedback_request_ms[i]) >= HT_FEEDBACK_PERIOD_MS) {
            s_last_feedback_request_ms[i] = now;
            status = ht_motor_request_feedback(s_ht_motor_ids[i]);
            if(status != MOTOR_STATUS_OK) {
                ht_log_can_diagnostic("feedback request", s_ht_motor_ids[i], status, now);
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
        }
    }
}

bool assemble_ht_motor_has_feedback(uint16_t id) {
    uint8_t idx = ht_motor_index_by_id(id);

    return idx < HT_TEST_MOTOR_COUNT && ht_feedback_is_fresh(idx, HAL_GetTick());
}

bool assemble_ht_motor_get_raw_position(uint16_t id, float* position) {
    uint8_t idx = ht_motor_index_by_id(id);
    uint32_t primask;
    uint32_t now = HAL_GetTick();

    if(idx >= HT_TEST_MOTOR_COUNT || position == NULL || !ht_feedback_is_fresh(idx, now)) {
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

    if(idx >= HT_TEST_MOTOR_COUNT || !can_tx_ready(&hfdcan1)) {
        return false;
    }

    if(!assemble_ht_motor_get_raw_position(id, &current_position)) {
        return false;
    }
    target_position = ht_motor_nearest_absolute_target(current_position, position);

    if(s_position_command_sent[idx]
        && (uint32_t)(now - s_last_target_tx_ms[idx]) < HT_TARGET_TX_PERIOD_MS) {
        return true;
    }

    if(!s_position_command_sent[idx]) {
        status = ht_motor_instance.enable(id);
        if(status != MOTOR_STATUS_OK) {
            ht_log_can_diagnostic("enable", id, status, now);
            return false;
        }
        s_position_command_sent[idx] = true;

        log_info("HT Classic target start id=%u encoder=%ld absolute=%ld",
                 id,
                 (long)(ht_motor_wrap_one_turn(position) * 1000.0f),
                 (long)(target_position * 1000.0f));
    }

    status = ht_motor_instance.set_pos(id, target_position);
    if(status != MOTOR_STATUS_OK) {
        ht_log_can_diagnostic("target", id, status, now);
        return false;
    }
    s_last_target_tx_ms[idx] = now;

    return true;
}

bool assemble_ht_motor_set_target_position_speed(uint16_t id, float position, float speed) {
    uint8_t idx = ht_motor_index_by_id(id);
    BusMotorStatus status;
    uint32_t now = HAL_GetTick();

    if(idx >= HT_TEST_MOTOR_COUNT || !can_tx_ready(&hfdcan1) ||
       !ht_feedback_is_fresh(idx, now)) {
        return false;
    }

    if(!s_position_command_sent[idx]) {
        status = ht_motor_instance.enable(id);
        if(status != MOTOR_STATUS_OK) {
            ht_log_can_diagnostic("enable pos_vel", id, status, now);
            return false;
        }
        s_position_command_sent[idx] = true;
    }

    status = ht_motor_instance.set_pos_vel(id, position, speed);
    if(status != MOTOR_STATUS_OK) {
        ht_log_can_diagnostic("target pos_vel", id, status, now);
        return false;
    }

    return true;
}

bool assemble_ht_motor_set_target_speed(uint16_t id, float speed) {
    uint8_t idx = ht_motor_index_by_id(id);
    BusMotorStatus status;
    uint32_t now = HAL_GetTick();

    if(idx >= HT_TEST_MOTOR_COUNT || !can_tx_ready(&hfdcan1) ||
       !ht_feedback_is_fresh(idx, now)) {
        return false;
    }

    if(!s_position_command_sent[idx]) {
        status = ht_motor_instance.enable(id);
        if(status != MOTOR_STATUS_OK) {
            ht_log_can_diagnostic("enable speed", id, status, now);
            return false;
        }
        s_position_command_sent[idx] = true;
    }

    status = ht_motor_instance.set_spd(id, speed);
    if(status != MOTOR_STATUS_OK) {
        ht_log_can_diagnostic("speed", id, status, now);
        return false;
    }

    return true;
}

static bool ht_motor_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send_classic(&hfdcan1, id, data, len) == STM32_HAL_CAN_OK;
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
                s_last_feedback_seen_ms[idx] = HAL_GetTick();
                s_feedback_seen[idx] = true;
                s_feedback_updated[idx] = true;
                s_discovery_period_ms[idx] = HT_DISCOVERY_PERIOD_MS;
            }
        }
    }
}

static bool ht_feedback_is_fresh(uint8_t idx, uint32_t now_ms) {
    bool seen;
    uint32_t last_ms;
    uint32_t primask;

    if(idx >= HT_TEST_MOTOR_COUNT) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    seen = s_feedback_seen[idx];
    last_ms = s_last_feedback_seen_ms[idx];
    if(primask == 0u) {
        __enable_irq();
    }

    return seen && (uint32_t)(now_ms - last_ms) <= HT_FEEDBACK_STALE_TIMEOUT_MS;
}

static void ht_handle_can_recovery(uint32_t now_ms) {
    uint32_t recovery_count = can_get_recovery_count(&hfdcan1);
    uint8_t i;

    if(recovery_count == s_seen_can_recovery_count) {
        return;
    }

    s_seen_can_recovery_count = recovery_count;
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        for(i = 0u; i < HT_TEST_MOTOR_COUNT; ++i) {
            s_feedback_seen[i] = false;
            s_feedback_updated[i] = false;
            s_position_command_sent[i] = false;
            s_stop_command_sent[i] = false;
            s_discovery_logged[i] = false;
            s_discovery_period_ms[i] = HT_DISCOVERY_PERIOD_MS;
            s_last_feedback_request_ms[i] = now_ms;
            s_last_target_tx_ms[i] = now_ms;
        }
        if(primask == 0u) {
            __enable_irq();
        }
    }
    log_warn("HT CAN recovery observed count=%lu; feedback/enable handshake reset",
             (unsigned long)recovery_count);
}

static void ht_log_can_diagnostic(const char* action,
                                  uint16_t id,
                                  BusMotorStatus motor_status,
                                  uint32_t now_ms) {
    BspCanHealth health;

    if((uint32_t)(now_ms - s_last_can_diag_log_ms) < HT_CAN_DIAG_LOG_PERIOD_MS) {
        return;
    }
    s_last_can_diag_log_ms = now_ms;

    if(can_get_health(&hfdcan1, &health) == STM32_HAL_CAN_OK) {
        log_error("HT %s id=%u: %s can=%s lec=%lu fault_lec=%lu fault_irq=0x%lX busoff=%lu txerr=%lu rxerr=%lu free=%lu fail=%lu recovery=%lu",
                  action,
                  id,
                  ht_motor_instance.status_str(motor_status),
                  can_error_code_to_str(health.last_tx_status),
                  (unsigned long)health.last_error_code,
                  (unsigned long)health.last_fault_error_code,
                  (unsigned long)health.last_fault_irq_flags,
                  (unsigned long)health.bus_off,
                  (unsigned long)health.tx_error_count,
                  (unsigned long)health.rx_error_count,
                  (unsigned long)health.tx_fifo_free_level,
                  (unsigned long)health.consecutive_tx_failures,
                  (unsigned long)health.recovery_count);
    }
    else {
        log_error("HT %s id=%u: %s", action, id, ht_motor_instance.status_str(motor_status));
    }
}

static uint8_t ht_motor_rx_length(uint32_t data_length) {
    switch(data_length) {
        case FDCAN_DLC_BYTES_0: return 0u;
        case FDCAN_DLC_BYTES_1: return 1u;
        case FDCAN_DLC_BYTES_2: return 2u;
        case FDCAN_DLC_BYTES_3: return 3u;
        case FDCAN_DLC_BYTES_4: return 4u;
        case FDCAN_DLC_BYTES_5: return 5u;
        case FDCAN_DLC_BYTES_6: return 6u;
        case FDCAN_DLC_BYTES_7: return 7u;
        case FDCAN_DLC_BYTES_8: return 8u;
        default:
            /* Some HAL versions expose the raw DLC nibble instead of encoded macros. */
            return data_length <= 8u ? (uint8_t)data_length : 0u;
    }
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
