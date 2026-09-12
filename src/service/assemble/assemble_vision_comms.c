/**
 * @file assemble_vision_comms.c
 * @brief 云台相机串口装配：把 UART 接收字节接入 vision_comms 服务
 *
 * 使用 UART7（115200 8N1，无流控，RX 不反相），需在 CubeMX 中把 UART7 波特率改为 115200。
 * 引脚：PE7=UART7_RX、PE8=UART7_TX。
 */

#include "assemble.h"

#include "delay.h"
#include "log.h"
#include "protocol_parser.h"
#include "stm32_hal_uart.h"
#include "vision_comms.h"

// ! ========================= 宏 定 义 声 明 ========================= ! //

/**
 * @brief 云台相机串口句柄
 * @details 相机输出 115200 8N1。使用 UART7（PE7=RX / PE8=TX）。
 */
#define VISION_COMMS_UART (&huart7)

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t s_vision_comms_rx_byte = 0u;
static uint32_t s_protocol_parser_saved_primask = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static uint32_t assemble_vision_comms_now_ms(void);
static void assemble_protocol_parser_critical_enter(void);
static void assemble_protocol_parser_critical_exit(void);
static void assemble_vision_comms_on_rx_complete(void);
static void assemble_vision_comms_on_error(void);
static void assemble_vision_comms_start_receive(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_vision_comms(void) {
    static const ProtocolParserCriticalOps critical_ops = {
        .enter = assemble_protocol_parser_critical_enter,
        .exit = assemble_protocol_parser_critical_exit,
    };
    VisionCommsConfig config;

    /* UART RX ISR 写 RingBuf、主循环读 RingBuf，共享 size/index，必须原子保护。
       注册为 protocol_parser 默认临界区后，后续创建的 vision/pc/pi RingBuf 都继承该保护。 */
    protocol_parser_register_critical_ops(&critical_ops);

    config.port_ops.now_ms = assemble_vision_comms_now_ms;
    if(vision_comms_init(&config) != VISION_COMMS_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    uart_register_rx_complete_callback(VISION_COMMS_UART, assemble_vision_comms_on_rx_complete);
    uart_register_error_callback(VISION_COMMS_UART, assemble_vision_comms_on_error);
    assemble_vision_comms_start_receive();
    log_info("VISION_COMMS init done");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void assemble_protocol_parser_critical_enter(void) {
    s_protocol_parser_saved_primask = __get_PRIMASK();
    __disable_irq();
}

static void assemble_protocol_parser_critical_exit(void) {
    if(s_protocol_parser_saved_primask == 0u) {
        __enable_irq();
    }
}

static uint32_t assemble_vision_comms_now_ms(void) {
    return delay_now_ms();
}

static void assemble_vision_comms_on_rx_complete(void) {
    vision_comms_on_rx_byte(s_vision_comms_rx_byte);
    assemble_vision_comms_start_receive();
}

static void assemble_vision_comms_on_error(void) {
    assemble_vision_comms_start_receive();
}

static void assemble_vision_comms_start_receive(void) {
    (void)uart_receive_it(VISION_COMMS_UART, &s_vision_comms_rx_byte, 1u);
}
