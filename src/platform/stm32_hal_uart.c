/**
 * @file stm32_hal_uart.c
 * @brief USART1 日志队列与命令接收
 */

#include "stm32_hal_uart.h"

#include "usart.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define UART1_TX_QUEUE_SIZE 2048u
#define UART1_RX_QUEUE_SIZE 256u

typedef struct {
    uint8_t buffer[UART1_TX_QUEUE_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t active_len;
    volatile bool busy;
} UartTxQueue;

static UartTxQueue s_uart1_tx;
static uint8_t s_uart1_rx_buffer[UART1_RX_QUEUE_SIZE];
static uint8_t s_uart1_rx_byte;
static volatile uint16_t s_uart1_rx_head;
static volatile uint16_t s_uart1_rx_tail;
static volatile bool s_uart1_rx_initialized;
static volatile bool s_uart1_rx_discard_line;
static volatile bool s_uart1_rx_reset_pending;
static volatile uint32_t s_uart1_rx_dropped;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void uart_tx_start(void);
static bool uart_rx_start(void);
static void uart_rx_reset_line(void);
static void uart_rx_push(uint8_t byte);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool uart1_write_it(const char* data, uint32_t len) {
    uint16_t used;
    uint16_t first_len;
    uint32_t primask;

    if(data == NULL || len == 0u || len >= UART1_TX_QUEUE_SIZE) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    used = (uint16_t)((s_uart1_tx.head + UART1_TX_QUEUE_SIZE - s_uart1_tx.tail) %
                      UART1_TX_QUEUE_SIZE);
    if(len > UART1_TX_QUEUE_SIZE - 1u - used) {
        if(primask == 0u) {
            __enable_irq();
        }
        uart_tx_start();
        return false;
    }

    first_len = (uint16_t)(UART1_TX_QUEUE_SIZE - s_uart1_tx.head);
    if(first_len > len) {
        first_len = (uint16_t)len;
    }
    memcpy(&s_uart1_tx.buffer[s_uart1_tx.head], data, first_len);
    if(first_len < len) {
        memcpy(s_uart1_tx.buffer, &data[first_len], len - first_len);
    }
    s_uart1_tx.head = (uint16_t)((s_uart1_tx.head + len) % UART1_TX_QUEUE_SIZE);
    if(primask == 0u) {
        __enable_irq();
    }

    uart_tx_start();
    return true;
}

bool uart1_console_init(void) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if(!s_uart1_rx_initialized) {
        s_uart1_rx_head = 0u;
        s_uart1_rx_tail = 0u;
        s_uart1_rx_discard_line = false;
        s_uart1_rx_reset_pending = false;
        s_uart1_rx_dropped = 0u;
        s_uart1_rx_initialized = true;
    }
    if(primask == 0u) {
        __enable_irq();
    }
    return uart_rx_start();
}

bool uart1_read_byte(uint8_t* byte) {
    uint32_t primask;
    bool available = false;

    if(byte == NULL || !s_uart1_rx_initialized) {
        return false;
    }
    (void)uart_rx_start();
    uart_tx_start();

    primask = __get_PRIMASK();
    __disable_irq();
    if(!s_uart1_rx_reset_pending && s_uart1_rx_head != s_uart1_rx_tail) {
        *byte = s_uart1_rx_buffer[s_uart1_rx_tail];
        s_uart1_rx_tail = (uint16_t)((s_uart1_rx_tail + 1u) % UART1_RX_QUEUE_SIZE);
        available = true;
    }
    if(primask == 0u) {
        __enable_irq();
    }
    return available;
}

bool uart1_take_rx_reset(void) {
    uint32_t primask = __get_PRIMASK();
    bool pending;

    __disable_irq();
    pending = s_uart1_rx_reset_pending;
    s_uart1_rx_reset_pending = false;
    if(primask == 0u) {
        __enable_irq();
    }
    return pending;
}

uint32_t uart1_rx_dropped_count(void) {
    uint32_t primask = __get_PRIMASK();
    uint32_t dropped;

    __disable_irq();
    dropped = s_uart1_rx_dropped;
    if(primask == 0u) {
        __enable_irq();
    }
    return dropped;
}

// ! ========================= HAL 回 调 实 现 ========================= ! //

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart != &huart1 || !s_uart1_tx.busy) {
        return;
    }

    s_uart1_tx.tail = (uint16_t)((s_uart1_tx.tail + s_uart1_tx.active_len) % UART1_TX_QUEUE_SIZE);
    s_uart1_tx.active_len = 0u;
    s_uart1_tx.busy = false;
    uart_tx_start();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart != &huart1 || !s_uart1_rx_initialized) {
        return;
    }
    if(huart->ErrorCode != HAL_UART_ERROR_NONE) {
        /* HAL 的错误分支可能先调用接收完成；此时重挂会清除 ErrorCode */
        uart_rx_reset_line();
        ++s_uart1_rx_dropped;
        return;
    }

    uart_rx_push(s_uart1_rx_byte);
    (void)uart_rx_start();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart != &huart1 || !s_uart1_rx_initialized) {
        return;
    }

    /* 接收错误只重置命令边界，不丢弃独立运行的日志发送队列 */
    uart_rx_reset_line();
    (void)uart_rx_start();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 从发送环启动一段连续传输，HAL 忙时保留数据等待主循环重试
 */
static void uart_tx_start(void) {
    uint32_t primask = __get_PRIMASK();
    uint16_t len;

    __disable_irq();
    if(!s_uart1_tx.busy && s_uart1_tx.head != s_uart1_tx.tail) {
        len = s_uart1_tx.head > s_uart1_tx.tail ? (uint16_t)(s_uart1_tx.head - s_uart1_tx.tail) : (uint16_t)(UART1_TX_QUEUE_SIZE - s_uart1_tx.tail);
        s_uart1_tx.active_len = len;
        s_uart1_tx.busy = true;
        if(HAL_UART_Transmit_IT(&huart1, &s_uart1_tx.buffer[s_uart1_tx.tail], len) != HAL_OK) {
            s_uart1_tx.active_len = 0u;
            s_uart1_tx.busy = false;
        }
    }
    if(primask == 0u) {
        __enable_irq();
    }
}

/**
 * @brief 启动或恢复单字节接收，保留调用前的中断屏蔽状态
 * @return 当前已有接收挂起时返回 true
 */
static bool uart_rx_start(void) {
    uint32_t primask = __get_PRIMASK();
    bool active;

    __disable_irq();
    if(s_uart1_rx_initialized && huart1.RxState == HAL_UART_STATE_READY) {
        /* 保留待处理的硬件错误标志，由 HAL IRQ 报错后统一重置命令边界 */
        if(HAL_UART_Receive_IT(&huart1, &s_uart1_rx_byte, 1u) != HAL_OK) {
            /* 重新挂接间隙可能丢字节，必须重新寻找完整命令的边界 */
            uart_rx_reset_line();
        }
    }
    active = s_uart1_rx_initialized && huart1.RxState == HAL_UART_STATE_BUSY_RX;
    if(primask == 0u) {
        __enable_irq();
    }
    return active;
}

/**
 * @brief 清空环并丢弃至下一个 LF；仅在 ISR 或关中断区调用
 */
static void uart_rx_reset_line(void) {
    s_uart1_rx_dropped += (uint16_t)((s_uart1_rx_head + UART1_RX_QUEUE_SIZE - s_uart1_rx_tail) %
                                     UART1_RX_QUEUE_SIZE);
    s_uart1_rx_head = 0u;
    s_uart1_rx_tail = 0u;
    s_uart1_rx_discard_line = true;
    s_uart1_rx_reset_pending = true;
}

/**
 * @brief 将收到的字节写入环；失步时丢弃完整行而不是输出截断的命令
 * @param byte 本次中断收到的字节
 */
static void uart_rx_push(uint8_t byte) {
    uint16_t next;

    if(s_uart1_rx_discard_line) {
        ++s_uart1_rx_dropped;
        if(byte == '\n') {
            s_uart1_rx_discard_line = false;
        }
        return;
    }
    next = (uint16_t)((s_uart1_rx_head + 1u) % UART1_RX_QUEUE_SIZE);
    if(next == s_uart1_rx_tail) {
        uart_rx_reset_line();
        ++s_uart1_rx_dropped;
        if(byte == '\n') {
            s_uart1_rx_discard_line = false;
        }
        return;
    }
    s_uart1_rx_buffer[s_uart1_rx_head] = byte;
    s_uart1_rx_head = next;
}
