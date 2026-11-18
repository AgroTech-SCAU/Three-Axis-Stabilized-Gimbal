/**
 * @file stm32_hal_uart.h
 * @brief USART1 日志与命令
 */

#ifndef _stm32_hal_uart_h_
#define _stm32_hal_uart_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 将整段日志复制到 USART1 中断发送队列
 * @param data 数据缓冲区，函数返回后可以立即复用
 * @param len 数据长度，单位 byte
 * @return 整段入队成功时为 true
 * @details 由中断发送，入队后可复用缓冲
 */
bool uart1_write_it(const char* data, uint32_t len);

/**
 * @brief 在 MX_USART1_UART_Init 之后启动命令接收
 * @return 接收挂起成功时为 true
 * @details 主循环调用，重复初始化保留已有数据
 */
bool uart1_console_init(void);

/**
 * @brief 读取一个字节并推进收发恢复
 * @param byte 接收字节的输出地址
 * @return 取到字节时为 true
 * @details 主循环按 LF 或 CRLF 解析命令
 */
bool uart1_read_byte(uint8_t* byte);

/**
 * @brief 取走失步事件，调用方清空未完成命令
 * @return 存在未消费的接收错误时为 true
 * @details 先处理本事件再读字节，失步后丢弃至下一 LF
 */
bool uart1_take_rx_reset(void);

/**
 * @brief 读取已知的软件丢弃字节累计数
 * @return 自首次初始化起累计丢弃的字节数，不含硬件无法计数的丢字节
 */
uint32_t uart1_rx_dropped_count(void);

#endif
