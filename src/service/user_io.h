#ifndef _service_user_io_h_
#define _service_user_io_h_

/**
 * @file user_io.h
 * @brief 云台按键事件与 RGB 状态指示服务
 * @details 主循环调用，按键只产生事件
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 用户交互服务状态码
 */
typedef enum {
    USER_IO_STATUS_OK = 0,          /**< 本轮处理成功，或没有需要发送的新灯态 */
    USER_IO_STATUS_INVALID_PARAM,   /**< 配置或指示状态无效 */
    USER_IO_STATUS_NOT_INITIALIZED, /**< 服务尚未初始化 */
    USER_IO_STATUS_LED_BUSY,        /**< RGB 上一帧仍在发送，下轮延后重试 */
    USER_IO_STATUS_LED_ERROR,       /**< RGB 接口拒绝发送，下轮延后重试 */
} UserIoStatus;

/**
 * @brief 云台实际运行状态对应的指示灯状态
 */
typedef enum {
    USER_IO_INDICATOR_WAIT = 0, /**< 红快闪：初始化、标定或反馈等待 */
    USER_IO_INDICATOR_RUN,      /**< 熄灭：正常运行 */
    USER_IO_INDICATOR_FAULT,    /**< 红快闪：异常或安全锁存 */
} UserIoIndicator;

/** @brief LED 传输端口的实际统计；不把 DMA 错误当作成功完成 */
typedef struct {
    uint32_t started_count;
    uint32_t completed_count;
    uint32_t error_count;
    uint32_t rejected_count;
    uint32_t last_error;
    uint32_t last_dma_error;
    uint32_t last_start_status;
    bool busy;
} UserIoLedTxStats;

/** @brief 主循环可读诊断；目标和提交结果均不代表实物颜色确认 */
typedef struct {
    UserIoIndicator indicator;       /**< 当前指示目标 */
    UserIoStatus last_submit_status; /**< 最后一次实际 fill/show 尝试；空轮不覆盖 */
    uint32_t busy_count;             /**< 实际尝试返回 BUSY 的累计次数 */
    uint32_t error_count;            /**< 实际尝试返回其他 LED 错误的累计次数 */
    UserIoLedTxStats tx;
    bool initialized;
    bool led_init_ok; /**< RGB 装配结果，仅用于诊断，不阻断按键 */
    bool submit_attempted;
} UserIoState;

/**
 * @brief 按键底层端口，由装配层注入
 */
typedef struct {
    bool (*read_button_level)(void);                   /**< 读取原始 GPIO 电平，true 为高电平 */
    void (*get_led_tx_stats)(UserIoLedTxStats* stats); /**< 可选，复制硬件传输统计 */
} UserIoPortOps;

/**
 * @brief 用户交互服务配置，初始化时复制配置与端口函数表
 */
typedef struct {
    const UserIoPortOps* ops; /**< 按键读取端口，不能为空 */
    bool button_active_high;  /**< true 为高电平按下，false 为低电平按下 */
    uint8_t brightness;       /**< 单通道亮度，范围 1~32 */
    bool led_init_ok;         /**< RGB 装配结果，失败仍初始化按键和诊断服务 */
} UserIoConfig;

/**
 * @brief 用户交互服务统一接口
 * @details RGB 先由 assemble_rgb 初始化，使用可回绕的 uint32_t 毫秒时钟
 */
typedef struct {
    UserIoStatus (*init)(const UserIoConfig* config);         /**< 初始化，默认指示 WAIT；上电需先观察稳定释放 */
    const char* (*status_str)(UserIoStatus status);           /**< 状态码转静态字符串 */
    UserIoStatus (*process)(uint32_t now_ms);                 /**< 非阻塞推进 5 ms 采样、25 ms 消抖和 RGB 刷新 */
    bool (*take_button_press)(void);                          /**< 取走一次已消抖的按下事件；长按不重复触发 */
    UserIoStatus (*set_indicator)(UserIoIndicator indicator); /**< 设置实际状态，下次 process 更新 RGB */
    const UserIoState* (*get_state)(void);                    /**< 主循环拥有的只读诊断快照 */
} UserIoInterface;

/**
 * @brief 云台用户交互服务单例
 */
extern const UserIoInterface user_io;

#endif
