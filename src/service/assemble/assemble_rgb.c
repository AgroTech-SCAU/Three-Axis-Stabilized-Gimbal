#include "assemble.h"

#include "rgb_led/rgb_led.h"
#include "rgb_led/ws2812_rgb_led.h"
#include "stm32_hal_spi.h"

// ! ========================= 变 量 声 明 ========================= ! //

/* 6 MHz 下 240 字节尾零保证 320 us 低电平，不依赖 SPI 关闭后的引脚电平 */
#define RGB_RESET_BYTES 240u

static uint8_t rgb_color_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * RGB_LED_COLOR_BYTES];
static uint8_t rgb_tx_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * WS2812_RGB_LED_BITS_PER_PIXEL + RGB_RESET_BYTES]
    __attribute__((section(".ram_d3"), aligned(32)));

static const RgbLedPortOps rgb_ops = {
    .write = spi_write,
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void rgb_led_write_complete_callback(SPI_HandleTypeDef* hspi);
static void rgb_led_write_error_callback(SPI_HandleTypeDef* hspi);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_rgb(void) {
    RgbLedConfig rgb_config;

    if(rgb_led_set_instance(&ws2812_rgb_led_instance) != RGB_LED_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    if(ws2812_rgb_led_make_config(
           &rgb_config,
           &rgb_ops,
           rgb_color_buffer,
           sizeof(rgb_color_buffer),
           rgb_tx_buffer,
           sizeof(rgb_tx_buffer)) != RGB_LED_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    rgb_config.async_write = true;
    rgb_config.reset_bytes = RGB_RESET_BYTES;

    if(rgb_led.init(&rgb_config) != RGB_LED_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    spi_register_tx_complete_callback(&hspi6, rgb_led_write_complete_callback);
    spi_register_error_callback(&hspi6, rgb_led_write_error_callback);
    /* 首帧熄灯，后续由 user_io 显示状态 */
    if(rgb_led.fill(0U, 0U, 0U) != RGB_LED_STATUS_OK || rgb_led.show() != RGB_LED_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void rgb_led_write_complete_callback(SPI_HandleTypeDef* hspi) {
    (void)hspi;
    (void)rgb_led_write_complete();
}

static void rgb_led_write_error_callback(SPI_HandleTypeDef* hspi) {
    (void)hspi;
    /* 错误只释放缓冲占用，由 SPI6 独立计数 */
    (void)rgb_led_write_complete();
}
