#ifndef _stm32_hal_can_h_
#define _stm32_hal_can_h_

#include <stdbool.h>
#include <stdint.h>

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 底盘转向电机所用 FDCAN1 句柄
 */
extern FDCAN_HandleTypeDef hfdcan1;
/**
 * @brief 底盘驱动电机所用 FDCAN2 句柄
 */
extern FDCAN_HandleTypeDef hfdcan2;

/**
 * @brief STM32 HAL CAN 抽象层状态码表
 */
#define STM32_HAL_CAN_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_DLC, "Invalid DLC") \
    X(FILTER_CONFIG_FAILED, "CAN Filter Config Failed") \
    X(START_FAILED, "CAN Start Failed") \
    X(NOTIFICATION_FAILED, "CAN Notification Activation Failed") \
    X(TX_MAILBOX_TIMEOUT, "CAN TX Mailbox Timeout") \
    X(TX_FAILED, "CAN TX Failed") \
    X(RX_FAILED, "CAN RX Failed") \
    X(NO_CALLBACK_SLOT, "No CAN RX Callback Slot") \
    X(DIAGNOSTIC_FAILED, "CAN Diagnostic Failed") \
    X(RECOVERY_IN_PROGRESS, "CAN Recovery In Progress") \
    X(RECOVERY_FAILED, "CAN Recovery Failed")

/**
 * @brief STM32 HAL CAN 抽象层状态码
 */
#define X(name, str) STM32_HAL_CAN_##name,
typedef enum {
    STM32_HAL_CAN_STATUS_TABLE
} BspCanStatus;
#undef X

/**
 * @brief CAN 运行时健康快照
 * @details 用于区分“发送 API 失败”和“总线错误/Bus-Off”，并观察共享总线恢复次数。
 */
typedef struct {
    BspCanStatus last_tx_status;
    uint32_t total_tx_failures;
    uint32_t consecutive_tx_failures;
    uint32_t tx_fifo_free_level;
    uint32_t last_error_code;
    uint32_t activity;
    uint32_t error_passive;
    uint32_t warning;
    uint32_t bus_off;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t recovery_count;
    bool recovery_in_progress;
    uint32_t recovery_backoff_ms;
    uint32_t recovery_started_ms;
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
} BspCanHealth;

/**
 * @brief CAN 接收回调函数类型
 * @param hcan 触发回调的 CAN 句柄
 * @param header 接收帧头
 * @param data 接收数据区，固定为 8 字节缓冲区
 * @param user 注册时绑定的用户指针
 */
typedef void (*STM32HalCanRxCallback)(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[64],
    void* user);

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化全局 CAN 过滤器
 * @return BspCanStatus 状态码
 */
BspCanStatus can_filter_init(void);

/**
 * @brief 启动指定 CAN 外设并使能接收中断
 * @param hcan CAN 句柄
 * @return BspCanStatus 状态码
 */
BspCanStatus can_start(FDCAN_HandleTypeDef* hcan);

/**
 * @brief 发送一帧经典 CAN 数据
 * @param hcan CAN 句柄
 * @param id 帧 ID
 * @param data 数据缓冲区
 * @param len 数据长度，最大 8 字节
 * @return BspCanStatus 状态码
 */
BspCanStatus can_send(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len);
BspCanStatus can_send_classic(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len);
BspCanStatus can_send_fd(FDCAN_HandleTypeDef* hcan, uint32_t id, const uint8_t* data, uint8_t len);

/**
 * @brief 注册指定 CAN 句柄的接收回调函数
 * @param hcan CAN 句柄
 * @param callback 回调函数
 * @param user 用户上下文指针
 * @return BspCanStatus 状态码
 */
BspCanStatus can_register_rx_callback(FDCAN_HandleTypeDef* hcan, STM32HalCanRxCallback callback, void* user);

/**
 * @brief 将 CAN 抽象层状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
const char* can_error_code_to_str(BspCanStatus status);

/**
 * @brief 获取 CAN 当前健康状态
 */
BspCanStatus can_get_health(FDCAN_HandleTypeDef* hcan, BspCanHealth* health);

/**
 * @brief 服务共享 CAN 总线的自动恢复
 * @param hcan CAN 句柄
 * @param recovered 可选输出，本次调用是否完成了一次恢复
 * @return BspCanStatus 状态码
 * @details Bus-Off 时只清除 M_CAN CCCR.INIT 并等待硬件完成 129 次 Bus Idle 恢复序列，
 *          不再循环 Stop/Start。TX FIFO 卡死时只清理挂起帧并进入退避期。
 */
BspCanStatus can_service_recovery(FDCAN_HandleTypeDef* hcan, bool* recovered);

/**
 * @brief 当前总线是否允许业务层继续发送
 * @details Bus-Off 恢复和退避期间返回 false，防止电机层继续填充 TX FIFO。
 */
bool can_tx_ready(FDCAN_HandleTypeDef* hcan);

/**
 * @brief 获取指定 CAN 总线累计恢复次数
 */
uint32_t can_get_recovery_count(FDCAN_HandleTypeDef* hcan);


/**
 * @brief Platform CAN API version used to detect stale incremental-build objects.
 */
#define STM32_HAL_CAN_API_VERSION 0x00050000u
uint32_t stm32_hal_can_api_version(void);

#endif
