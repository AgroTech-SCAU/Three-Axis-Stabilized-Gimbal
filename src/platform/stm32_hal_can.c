#include "stm32_hal_can.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief CAN 发送等待邮箱空闲的超时时间，单位 ms
 */
/**
 * @brief 可注册的 CAN 接收回调槽位数量
 */
#define CAN_RX_CALLBACK_SLOT_NUM 4u
#define CAN_RX_MAX_FRAMES_PER_IRQ 8u
#define CAN_RUNTIME_SLOT_NUM 2u
#define CAN_RECOVERY_TX_FAIL_THRESHOLD 3u
#define CAN_RECOVERY_POLL_PERIOD_MS 5u
#define CAN_RECOVERY_BACKOFF_INITIAL_MS 250u
#define CAN_RECOVERY_BACKOFF_MAX_MS 4000u


/**
 * @brief CAN 接收回调注册槽位
 * @param hcan 对应的 CAN 句柄
 * @param callback 已注册的回调函数
 * @param user 用户上下文指针
 */
typedef struct {
    FDCAN_HandleTypeDef* hcan;
    STM32HalCanRxCallback callback;
    void* user;
} CanRxCallbackSlot;

/**
 * @brief CAN 接收回调槽位表
 */
static CanRxCallbackSlot s_rx_slots[CAN_RX_CALLBACK_SLOT_NUM];

typedef struct {
    FDCAN_HandleTypeDef* hcan;
    BspCanStatus last_tx_status;
    uint32_t total_tx_failures;
    uint32_t consecutive_tx_failures;
    uint32_t recovery_count;
    bool recovery_in_progress;
    bool recovery_bus_off;
    uint32_t recovery_started_ms;
    uint32_t recovery_resume_not_before_ms;
    uint32_t recovery_backoff_ms;
    uint32_t last_recovery_poll_ms;
    uint32_t last_recovery_error_code;
    uint32_t last_recovery_bus_off;
    uint32_t last_recovery_tx_error_count;
    uint32_t last_recovery_rx_error_count;
    uint32_t last_recovery_tx_fifo_free_level;
    uint32_t last_fault_irq_flags;
    uint32_t last_fault_event_ms;
    uint32_t last_fault_error_code;
    uint32_t last_fault_activity;
    uint32_t last_fault_error_passive;
    uint32_t last_fault_warning;
    uint32_t last_fault_bus_off;
    uint32_t last_fault_tx_error_count;
    uint32_t last_fault_rx_error_count;
} CanRuntimeState;

static CanRuntimeState s_runtime_slots[CAN_RUNTIME_SLOT_NUM];

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将字节长度转换为 FDCAN DLC 编码
 * @param len 数据长度
 * @return uint32_t DLC 编码值
 */
static uint32_t can_len_to_dlc(uint8_t len);
static BspCanStatus can_send_with_format(FDCAN_HandleTypeDef* hcan,
                                          uint32_t id,
                                          const uint8_t* data,
                                          uint8_t len,
                                          uint32_t fd_format);
/**
 * @brief 配置指定 CAN 句柄的全局过滤器
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
static BspCanStatus can_config_global_filter(FDCAN_HandleTypeDef* hcan);
/**
 * @brief 分发一帧接收到的数据到已注册回调
 * @param hcan CAN 句柄
 * @param header 接收帧头
 * @param data 接收数据
 */
static void can_dispatch_rx(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[64]);
/**
 * @brief 使能 CAN 收发器
 * @param hcan CAN 句柄
 */
static void can_enable_transceiver(FDCAN_HandleTypeDef* hcan);
static CanRuntimeState* can_runtime_state(FDCAN_HandleTypeDef* hcan);
static void can_record_tx_result(FDCAN_HandleTypeDef* hcan, BspCanStatus status);
static void can_abort_pending_tx(FDCAN_HandleTypeDef* hcan);
static BspCanStatus can_begin_busoff_recovery(FDCAN_HandleTypeDef* hcan, CanRuntimeState* runtime, const BspCanHealth* health, uint32_t now_ms);
static BspCanStatus can_begin_tx_quiet_recovery(FDCAN_HandleTypeDef* hcan, CanRuntimeState* runtime, const BspCanHealth* health, uint32_t now_ms);
static void can_finish_recovery(CanRuntimeState* runtime);
static bool can_busoff_recovery_complete(const BspCanHealth* health);
static void can_note_rx_activity(FDCAN_HandleTypeDef* hcan);
static void can_latch_error_status(FDCAN_HandleTypeDef* hcan, uint32_t irq_flags);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化全局 CAN 过滤器
 * @return BspCanStatus 状态码
 */
BspCanStatus can_filter_init(void) {
    BspCanStatus status;

    status = can_config_global_filter(&hfdcan1);
    if(status != STM32_HAL_CAN_OK) {
        return status;
    }

    status = can_config_global_filter(&hfdcan2);
    if(status != STM32_HAL_CAN_OK) {
        return status;
    }

    return STM32_HAL_CAN_OK;
}

/**
 * @brief 启动指定 CAN 外设并使能接收中断
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
BspCanStatus can_start(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    if(HAL_FDCAN_GetState(hcan) == HAL_FDCAN_STATE_BUSY) {
        (void)can_runtime_state(hcan);
        return STM32_HAL_CAN_OK;
    }

    /*
     * FDCAN1 carries continuously refreshed motor-control commands. Replaying a
     * stale command indefinitely after an ACK/physical-layer fault is unsafe,
     * and a single unacknowledged frame with hardware auto-retransmit enabled
     * can drive TEC to Bus-Off before the application gets CPU time.
     *
     * Disable automatic retransmission on the shared motor bus. DM/HT already
     * implement software retries/periodic setpoints, so the next fresh command
     * supersedes a dropped frame. Keep hfdcan1.Init in sync with the register so
     * diagnostics/regeneration do not disagree about the active policy.
     */
    if(hcan->Instance == FDCAN1) {
        hcan->Init.AutoRetransmission = DISABLE;
        SET_BIT(hcan->Instance->CCCR, FDCAN_CCCR_CCE);
        SET_BIT(hcan->Instance->CCCR, FDCAN_CCCR_DAR);
    }

    {
        const uint32_t notification_mask = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                           FDCAN_IT_ERROR_WARNING |
                                           FDCAN_IT_ERROR_PASSIVE |
                                           FDCAN_IT_BUS_OFF;

        if(HAL_FDCAN_ConfigInterruptLines(hcan, notification_mask, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
            return STM32_HAL_CAN_NOTIFICATION_FAILED;
        }

        if(HAL_FDCAN_ActivateNotification(hcan, notification_mask, 0u) != HAL_OK) {
            return STM32_HAL_CAN_NOTIFICATION_FAILED;
        }
    }

    can_enable_transceiver(hcan);

    if(HAL_FDCAN_Start(hcan) != HAL_OK) {
        return STM32_HAL_CAN_START_FAILED;
    }

    (void)can_runtime_state(hcan);
    return STM32_HAL_CAN_OK;
}

/**
 * @brief 发送一帧经典 CAN 数据
 * @param hcan CAN 句柄
 * @param id 帧 ID
 * @param data 数据缓冲区
 * @param len 数据长度，最大 8 字节
 * @return BspCanStatus 状态码
 */
BspCanStatus can_send(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len) {
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    return can_send_with_format(hcan,
                                id,
                                data,
                                len,
                                hcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC
                                    ? FDCAN_CLASSIC_CAN
                                    : FDCAN_FD_CAN);
}

BspCanStatus can_send_classic(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send_with_format(hcan, id, data, len, FDCAN_CLASSIC_CAN);
}

BspCanStatus can_send_fd(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len) {
    return can_send_with_format(hcan, id, data, len, FDCAN_FD_CAN);
}

static BspCanStatus can_send_with_format(FDCAN_HandleTypeDef* hcan,
                                          uint32_t id,
                                          const uint8_t* data,
                                          uint8_t len,
                                          uint32_t fd_format) {
    FDCAN_TxHeaderTypeDef tx_header = { 0 };

    CanRuntimeState* runtime;

    if(hcan == NULL || data == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    runtime = can_runtime_state(hcan);
    if(runtime == NULL) {
        return STM32_HAL_CAN_NO_CALLBACK_SLOT;
    }
    if(runtime->recovery_in_progress ||
       (HAL_FDCAN_GetState(hcan) == HAL_FDCAN_STATE_BUSY &&
        (hcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0u)) {
        runtime->last_tx_status = STM32_HAL_CAN_RECOVERY_IN_PROGRESS;
        return STM32_HAL_CAN_RECOVERY_IN_PROGRESS;
    }

    if(can_len_to_dlc(len) == 0xFFFFFFFFu ||
       (fd_format == FDCAN_CLASSIC_CAN && len > 8u)) {
        return STM32_HAL_CAN_INVALID_DLC;
    }

    tx_header.Identifier = id;
    tx_header.IdType = (id <= 0x7FFu) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = can_len_to_dlc(len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    /*
     * Classic CAN never uses BRS, even when the FDCAN peripheral itself is
     * configured in FD+BRS mode for other devices on the same bus.  DMG6220
     * V4 control frames are classic CAN frames, so CLASSIC+BRS_ON is invalid
     * and can make disable/mode/enable appear to be sent while the motor never
     * receives them.
     */
    if(fd_format == FDCAN_CLASSIC_CAN) {
        tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    }
    else {
        tx_header.BitRateSwitch = hcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS
                                      ? FDCAN_BRS_ON
                                      : FDCAN_BRS_OFF;
    }
    tx_header.FDFormat = fd_format;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0u;

    if(HAL_FDCAN_GetTxFifoFreeLevel(hcan) == 0u) {
        /*
         * Shared motor bus: do not abort pending frames from the producer path.
         * A centralized recovery service owns abort/stop/start so DM/HT/chassis
         * cannot cancel each other's traffic opportunistically.
         */
        can_record_tx_result(hcan, STM32_HAL_CAN_TX_MAILBOX_TIMEOUT);
        return STM32_HAL_CAN_TX_MAILBOX_TIMEOUT;
    }

    if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, (uint8_t*)data) != HAL_OK) {
        can_record_tx_result(hcan, STM32_HAL_CAN_TX_FAILED);
        return STM32_HAL_CAN_TX_FAILED;
    }

    can_record_tx_result(hcan, STM32_HAL_CAN_OK);
    return STM32_HAL_CAN_OK;
}

/**
 * @brief 注册指定 CAN 句柄的接收回调函数
 * @param hcan CAN 句柄
 * @param callback 回调函数
 * @param user 用户上下文指针
 * @return BspCanStatus 状态码
 */
BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* hcan, STM32HalCanRxCallback callback, void* user) {
    uint8_t i;

    if(hcan == NULL || callback == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback == callback) {
            s_rx_slots[i].user = user;
            return STM32_HAL_CAN_OK;
        }
    }

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].callback == NULL) {
            s_rx_slots[i].hcan = hcan;
            s_rx_slots[i].callback = callback;
            s_rx_slots[i].user = user;
            return STM32_HAL_CAN_OK;
        }
    }

    return STM32_HAL_CAN_NO_CALLBACK_SLOT;
}

/**
 * @brief 将 CAN 抽象层状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
#define X(name, str)           \
    case STM32_HAL_CAN_##name: \
        return str;
const char* can_error_code_to_str(BspCanStatus status) {
    switch(status) {
        STM32_HAL_CAN_STATUS_TABLE
        default:
            return "UNKNOWN";
    }
}
#undef X


BspCanStatus can_get_health(FDCAN_HandleTypeDef* hcan, BspCanHealth* health) {
    FDCAN_ProtocolStatusTypeDef protocol_status;
    FDCAN_ErrorCountersTypeDef error_counters;
    CanRuntimeState* runtime;

    if(hcan == NULL || health == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    runtime = can_runtime_state(hcan);
    if(runtime == NULL) {
        return STM32_HAL_CAN_NO_CALLBACK_SLOT;
    }

    if(HAL_FDCAN_GetProtocolStatus(hcan, &protocol_status) != HAL_OK ||
       HAL_FDCAN_GetErrorCounters(hcan, &error_counters) != HAL_OK) {
        return STM32_HAL_CAN_DIAGNOSTIC_FAILED;
    }

    health->last_tx_status = runtime->last_tx_status;
    health->total_tx_failures = runtime->total_tx_failures;
    health->consecutive_tx_failures = runtime->consecutive_tx_failures;
    health->tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(hcan);
    health->last_error_code = protocol_status.LastErrorCode;
    health->activity = protocol_status.Activity;
    health->error_passive = protocol_status.ErrorPassive;
    health->warning = protocol_status.Warning;
    health->bus_off = protocol_status.BusOff;
    health->tx_error_count = error_counters.TxErrorCnt;
    health->rx_error_count = error_counters.RxErrorCnt;
    health->recovery_count = runtime->recovery_count;
    health->recovery_in_progress = runtime->recovery_in_progress;
    health->recovery_backoff_ms = runtime->recovery_in_progress
                                      ? (uint32_t)(runtime->recovery_resume_not_before_ms - runtime->recovery_started_ms)
                                      : runtime->recovery_backoff_ms;
    health->recovery_started_ms = runtime->recovery_started_ms;
    health->last_recovery_error_code = runtime->last_recovery_error_code;
    health->last_recovery_bus_off = runtime->last_recovery_bus_off;
    health->last_recovery_tx_error_count = runtime->last_recovery_tx_error_count;
    health->last_recovery_rx_error_count = runtime->last_recovery_rx_error_count;
    health->last_recovery_tx_fifo_free_level = runtime->last_recovery_tx_fifo_free_level;
    health->last_fault_irq_flags = runtime->last_fault_irq_flags;
    health->last_fault_event_ms = runtime->last_fault_event_ms;
    health->last_fault_error_code = runtime->last_fault_error_code;
    health->last_fault_activity = runtime->last_fault_activity;
    health->last_fault_error_passive = runtime->last_fault_error_passive;
    health->last_fault_warning = runtime->last_fault_warning;
    health->last_fault_bus_off = runtime->last_fault_bus_off;
    health->last_fault_tx_error_count = runtime->last_fault_tx_error_count;
    health->last_fault_rx_error_count = runtime->last_fault_rx_error_count;
    return STM32_HAL_CAN_OK;
}

BspCanStatus can_service_recovery(FDCAN_HandleTypeDef* hcan, bool* recovered) {
    BspCanHealth health;
    CanRuntimeState* runtime;
    BspCanStatus status;
    uint32_t now_ms;

    if(recovered != NULL) {
        *recovered = false;
    }
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    runtime = can_runtime_state(hcan);
    if(runtime == NULL) {
        return STM32_HAL_CAN_NO_CALLBACK_SLOT;
    }

    now_ms = HAL_GetTick();
    if((uint32_t)(now_ms - runtime->last_recovery_poll_ms) < CAN_RECOVERY_POLL_PERIOD_MS) {
        return STM32_HAL_CAN_OK;
    }
    runtime->last_recovery_poll_ms = now_ms;

    status = can_get_health(hcan, &health);
    if(status != STM32_HAL_CAN_OK) {
        return status;
    }

    /*
     * Bosch M_CAN Bus-Off recovery is a state machine, not a peripheral
     * restart.  Bus-Off sets CCCR.INIT itself. Software must clear INIT once,
     * then leave the controller alone while it observes 129 Bus Idle
     * occurrences. Repeated HAL_FDCAN_Stop()/Start() re-enters INIT and can
     * keep restarting/obscuring that protocol-level recovery.
     */
    if(runtime->recovery_in_progress) {
        if(runtime->recovery_bus_off && !can_busoff_recovery_complete(&health)) {
            return STM32_HAL_CAN_OK;
        }

        if((int32_t)(now_ms - runtime->recovery_resume_not_before_ms) < 0) {
            return STM32_HAL_CAN_OK;
        }

        can_finish_recovery(runtime);
        if(recovered != NULL) {
            *recovered = true;
        }
        return STM32_HAL_CAN_OK;
    }

    if(health.bus_off != 0u) {
        return can_begin_busoff_recovery(hcan, runtime, &health, now_ms);
    }

    if(health.tx_fifo_free_level == 0u &&
       health.consecutive_tx_failures >= CAN_RECOVERY_TX_FAIL_THRESHOLD) {
        return can_begin_tx_quiet_recovery(hcan, runtime, &health, now_ms);
    }

    return STM32_HAL_CAN_OK;
}

bool can_tx_ready(FDCAN_HandleTypeDef* hcan) {
    CanRuntimeState* runtime = can_runtime_state(hcan);
    return runtime != NULL && !runtime->recovery_in_progress;
}

uint32_t can_get_recovery_count(FDCAN_HandleTypeDef* hcan) {
    CanRuntimeState* runtime = can_runtime_state(hcan);
    return runtime == NULL ? 0u : runtime->recovery_count;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL FDCAN 错误状态回调入口
 * @details 只锁存故障发生瞬间的协议状态，不在中断里做恢复或打印日志。
 *          Bus-Off 的 CCCR.INIT 清除由 can_service_recovery() 在主循环统一完成。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t error_status_its) {
    if(hfdcan == NULL) {
        return;
    }

    if((error_status_its & (FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF)) != 0u) {
        can_latch_error_status(hfdcan, error_status_its);
    }
}

/**
 * @brief HAL FDCAN FIFO0 接收回调入口
 * @param hfdcan 触发回调的 CAN 句柄
 * @param rx_fifo0_its FIFO0 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t rx_fifo0_its) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    uint8_t frame_count = 0u;

    if((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u) {
        return;
    }

    while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u && frame_count < CAN_RX_MAX_FRAMES_PER_IRQ) {
        memset(rx_data, 0, sizeof(rx_data));
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            return;
        }
        can_note_rx_activity(hfdcan);
        can_dispatch_rx(hfdcan, &rx_header, rx_data);
        ++frame_count;
    }
}

/**
 * @brief 将字节长度转换为 FDCAN DLC 编码
 * @param len 数据长度
 * @return uint32_t DLC 编码值
 */
static uint32_t can_len_to_dlc(uint8_t len) {
    switch(len) {
        case 0u:
            return FDCAN_DLC_BYTES_0;
        case 1u:
            return FDCAN_DLC_BYTES_1;
        case 2u:
            return FDCAN_DLC_BYTES_2;
        case 3u:
            return FDCAN_DLC_BYTES_3;
        case 4u:
            return FDCAN_DLC_BYTES_4;
        case 5u:
            return FDCAN_DLC_BYTES_5;
        case 6u:
            return FDCAN_DLC_BYTES_6;
        case 7u:
            return FDCAN_DLC_BYTES_7;
        case 8u:
            return FDCAN_DLC_BYTES_8;
        case 12u:
            return FDCAN_DLC_BYTES_12;
        case 16u:
            return FDCAN_DLC_BYTES_16;
        case 20u:
            return FDCAN_DLC_BYTES_20;
        case 24u:
            return FDCAN_DLC_BYTES_24;
        case 32u:
            return FDCAN_DLC_BYTES_32;
        case 48u:
            return FDCAN_DLC_BYTES_48;
        case 64u:
            return FDCAN_DLC_BYTES_64;
        default:
            return 0xFFFFFFFFu;
    }
}

/**
 * @brief 配置指定 CAN 句柄的全局过滤器
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
static BspCanStatus can_config_global_filter(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    if(HAL_FDCAN_GetState(hcan) == HAL_FDCAN_STATE_BUSY) {
        (void)can_runtime_state(hcan);
        return STM32_HAL_CAN_OK;
    }

    if(HAL_FDCAN_ConfigGlobalFilter(hcan,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_ACCEPT_IN_RX_FIFO0,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK) {
        return STM32_HAL_CAN_FILTER_CONFIG_FAILED;
    }

    return STM32_HAL_CAN_OK;
}

/**
 * @brief 分发一帧接收到的数据到已注册回调
 * @param hcan CAN 句柄
 * @param header 接收帧头
 * @param data 接收数据
 */
static void can_dispatch_rx(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[64]) {
    uint8_t i;

    for(i = 0u; i < CAN_RX_CALLBACK_SLOT_NUM; ++i) {
        if(s_rx_slots[i].hcan == hcan && s_rx_slots[i].callback != NULL) {
            s_rx_slots[i].callback(hcan, header, data, s_rx_slots[i].user);
        }
    }
}

/**
 * @brief 使能 CAN 收发器
 * @param hcan CAN 句柄
 */
static void can_enable_transceiver(FDCAN_HandleTypeDef* hcan) {
    if(hcan == NULL) {
        return;
    }

    if(hcan->Instance == FDCAN1) {
        HAL_GPIO_WritePin(CAN1_EN_GPIO_Port, CAN1_EN_Pin, GPIO_PIN_SET);
    }
    else if(hcan->Instance == FDCAN2) {
        HAL_GPIO_WritePin(CAN2_EN_GPIO_Port, CAN2_EN_Pin, GPIO_PIN_SET);
    }
}

static CanRuntimeState* can_runtime_state(FDCAN_HandleTypeDef* hcan) {
    uint8_t i;

    if(hcan == NULL) {
        return NULL;
    }

    for(i = 0u; i < CAN_RUNTIME_SLOT_NUM; ++i) {
        if(s_runtime_slots[i].hcan == hcan) {
            return &s_runtime_slots[i];
        }
    }

    for(i = 0u; i < CAN_RUNTIME_SLOT_NUM; ++i) {
        if(s_runtime_slots[i].hcan == NULL) {
            s_runtime_slots[i].hcan = hcan;
            s_runtime_slots[i].last_tx_status = STM32_HAL_CAN_OK;
            s_runtime_slots[i].total_tx_failures = 0u;
            s_runtime_slots[i].consecutive_tx_failures = 0u;
            s_runtime_slots[i].recovery_count = 0u;
            s_runtime_slots[i].recovery_in_progress = false;
            s_runtime_slots[i].recovery_bus_off = false;
            s_runtime_slots[i].recovery_started_ms = 0u;
            s_runtime_slots[i].recovery_resume_not_before_ms = 0u;
            s_runtime_slots[i].recovery_backoff_ms = CAN_RECOVERY_BACKOFF_INITIAL_MS;
            s_runtime_slots[i].last_recovery_poll_ms = 0u;
            s_runtime_slots[i].last_recovery_error_code = 0u;
            s_runtime_slots[i].last_recovery_bus_off = 0u;
            s_runtime_slots[i].last_recovery_tx_error_count = 0u;
            s_runtime_slots[i].last_recovery_rx_error_count = 0u;
            s_runtime_slots[i].last_recovery_tx_fifo_free_level = 0u;
            s_runtime_slots[i].last_fault_irq_flags = 0u;
            s_runtime_slots[i].last_fault_event_ms = 0u;
            s_runtime_slots[i].last_fault_error_code = 0u;
            s_runtime_slots[i].last_fault_activity = 0u;
            s_runtime_slots[i].last_fault_error_passive = 0u;
            s_runtime_slots[i].last_fault_warning = 0u;
            s_runtime_slots[i].last_fault_bus_off = 0u;
            s_runtime_slots[i].last_fault_tx_error_count = 0u;
            s_runtime_slots[i].last_fault_rx_error_count = 0u;
            return &s_runtime_slots[i];
        }
    }

    return NULL;
}

static void can_record_tx_result(FDCAN_HandleTypeDef* hcan, BspCanStatus status) {
    CanRuntimeState* runtime = can_runtime_state(hcan);

    if(runtime == NULL) {
        return;
    }

    runtime->last_tx_status = status;
    if(status == STM32_HAL_CAN_OK) {
        runtime->consecutive_tx_failures = 0u;
        return;
    }

    runtime->total_tx_failures++;
    if(runtime->consecutive_tx_failures < UINT32_MAX) {
        runtime->consecutive_tx_failures++;
    }
}

static void can_abort_pending_tx(FDCAN_HandleTypeDef* hcan) {
    uint32_t i;
    uint32_t tx_location_count;

    if(hcan == NULL || HAL_FDCAN_GetState(hcan) != HAL_FDCAN_STATE_BUSY) {
        return;
    }

    tx_location_count = hcan->Init.TxBuffersNbr + hcan->Init.TxFifoQueueElmtsNbr;
    if(tx_location_count > 32u) {
        tx_location_count = 32u;
    }
    for(i = 0u; i < tx_location_count; ++i) {
        (void)HAL_FDCAN_AbortTxRequest(hcan, (uint32_t)1u << i);
    }
}

static void can_snapshot_recovery_cause(CanRuntimeState* runtime,
                                        const BspCanHealth* health,
                                        uint32_t now_ms,
                                        bool bus_off) {
    uint32_t backoff;

    runtime->last_recovery_error_code = health->last_error_code;
    runtime->last_recovery_bus_off = health->bus_off;
    runtime->last_recovery_tx_error_count = health->tx_error_count;
    runtime->last_recovery_rx_error_count = health->rx_error_count;
    runtime->last_recovery_tx_fifo_free_level = health->tx_fifo_free_level;
    runtime->recovery_in_progress = true;
    runtime->recovery_bus_off = bus_off;
    runtime->recovery_started_ms = now_ms;

    backoff = runtime->recovery_backoff_ms;
    if(backoff < CAN_RECOVERY_BACKOFF_INITIAL_MS) {
        backoff = CAN_RECOVERY_BACKOFF_INITIAL_MS;
    }
    runtime->recovery_resume_not_before_ms = now_ms + backoff;
    if(backoff < CAN_RECOVERY_BACKOFF_MAX_MS) {
        backoff <<= 1u;
        if(backoff > CAN_RECOVERY_BACKOFF_MAX_MS) {
            backoff = CAN_RECOVERY_BACKOFF_MAX_MS;
        }
    }
    runtime->recovery_backoff_ms = backoff;
    runtime->last_tx_status = STM32_HAL_CAN_RECOVERY_IN_PROGRESS;
}

static BspCanStatus can_begin_busoff_recovery(FDCAN_HandleTypeDef* hcan,
                                               CanRuntimeState* runtime,
                                               const BspCanHealth* health,
                                               uint32_t now_ms) {
    if(hcan == NULL || runtime == NULL || health == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    /* Drop stale motor commands before the controller is allowed back online. */
    can_abort_pending_tx(hcan);
    can_snapshot_recovery_cause(runtime, health, now_ms, true);

    /*
     * M_CAN sets CCCR.INIT when entering Bus-Off. The specified recovery
     * sequence starts only after software clears INIT. Do this once and then
     * wait for PSR.BO to clear; do not HAL_FDCAN_Stop()/Start() in a loop.
     */
    CLEAR_BIT(hcan->Instance->CCCR, FDCAN_CCCR_INIT);
    return STM32_HAL_CAN_OK;
}

static BspCanStatus can_begin_tx_quiet_recovery(FDCAN_HandleTypeDef* hcan,
                                                 CanRuntimeState* runtime,
                                                 const BspCanHealth* health,
                                                 uint32_t now_ms) {
    if(hcan == NULL || runtime == NULL || health == NULL) {
        return STM32_HAL_CAN_INVALID_PARAM;
    }

    /*
     * A full FIFO with repeated producer failures is quarantined without
     * resetting FDCAN. Aborting stale requests prevents AutoRetransmission
     * from monopolising all TX slots while the physical bus is unhealthy.
     */
    can_abort_pending_tx(hcan);
    can_snapshot_recovery_cause(runtime, health, now_ms, false);
    return STM32_HAL_CAN_OK;
}

static void can_finish_recovery(CanRuntimeState* runtime) {
    if(runtime == NULL) {
        return;
    }

    runtime->recovery_in_progress = false;
    runtime->recovery_bus_off = false;
    runtime->recovery_started_ms = 0u;
    runtime->recovery_resume_not_before_ms = 0u;
    runtime->consecutive_tx_failures = 0u;
    runtime->last_tx_status = STM32_HAL_CAN_OK;
    if(runtime->recovery_count < UINT32_MAX) {
        runtime->recovery_count++;
    }
}

static bool can_busoff_recovery_complete(const BspCanHealth* health) {
    if(health == NULL) {
        return false;
    }

    /*
     * Bosch M_CAN specifies that after the 129 Bus-Idle recovery sequence,
     * PSR.BO clears, ACT leaves Synchronizing, and both error counters reset.
     * Requiring all of those avoids reopening motor traffic on a transient or
     * partially observed register state.
     */
    return health->bus_off == 0u &&
           health->activity != 0u &&
           health->error_passive == 0u &&
           health->warning == 0u &&
           health->tx_error_count == 0u &&
           health->rx_error_count == 0u;
}

static void can_latch_error_status(FDCAN_HandleTypeDef* hcan, uint32_t irq_flags) {
    FDCAN_ProtocolStatusTypeDef protocol_status;
    FDCAN_ErrorCountersTypeDef error_counters;
    CanRuntimeState* runtime;

    runtime = can_runtime_state(hcan);
    if(runtime == NULL) {
        return;
    }

    if(HAL_FDCAN_GetProtocolStatus(hcan, &protocol_status) != HAL_OK ||
       HAL_FDCAN_GetErrorCounters(hcan, &error_counters) != HAL_OK) {
        return;
    }

    runtime->last_fault_irq_flags = irq_flags;
    runtime->last_fault_event_ms = HAL_GetTick();
    runtime->last_fault_error_code = protocol_status.LastErrorCode;
    runtime->last_fault_activity = protocol_status.Activity;
    runtime->last_fault_error_passive = protocol_status.ErrorPassive;
    runtime->last_fault_warning = protocol_status.Warning;
    runtime->last_fault_bus_off = protocol_status.BusOff;
    runtime->last_fault_tx_error_count = error_counters.TxErrorCnt;
    runtime->last_fault_rx_error_count = error_counters.RxErrorCnt;
}

static void can_note_rx_activity(FDCAN_HandleTypeDef* hcan) {
    CanRuntimeState* runtime = can_runtime_state(hcan);

    if(runtime == NULL) {
        return;
    }

    /* A valid received frame is hard evidence that the physical bus is alive. */
    runtime->recovery_backoff_ms = CAN_RECOVERY_BACKOFF_INITIAL_MS;
    runtime->consecutive_tx_failures = 0u;
}



uint32_t stm32_hal_can_api_version(void) {
    return STM32_HAL_CAN_API_VERSION;
}
