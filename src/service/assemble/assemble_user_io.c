/**
 * @file assemble_user_io.c
 * @brief 装配 PA15 按键与已有 SPI6 DMA RGB 指示灯
 */

#include "assemble.h"

#include "log.h"
#include "stm32_hal_gpio.h"
#include "stm32_hal_spi.h"
#include "user_io.h"

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void user_io_read_led_tx_stats(UserIoLedTxStats* stats);

// ! ========================= 变 量 声 明 ========================= ! //

static const UserIoPortOps user_io_ops = {
    .read_button_level = stm32_hal_user_key_read_level,
    .get_led_tx_stats = user_io_read_led_tx_stats,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 先初始化 RGB，再初始化按键消抖和灯态服务
 * @details PA15 输入上拉，低电平按下
 * @return 装配成功返回 SYSTEM_STATUS_OK
 */
SystemStatus assemble_user_io(void) {
    bool led_init_ok = assemble_rgb() == SYSTEM_STATUS_OK;
    const UserIoConfig config = {
        .ops = &user_io_ops,
        .button_active_high = false,
        .brightness = 16u,
        .led_init_ok = led_init_ok,
    };

    if(!led_init_ok) {
        log_warn("LED init failed | PA15 active | retry pending");
    }
    return user_io.init(&config) == USER_IO_STATUS_OK ? SYSTEM_STATUS_OK : SYSTEM_STATUS_ERROR;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void user_io_read_led_tx_stats(UserIoLedTxStats* stats) {
    Spi6TxStats tx;
    spi6_get_tx_stats(&tx);
    *stats = (UserIoLedTxStats){
        .started_count = tx.started_count,
        .completed_count = tx.completed_count,
        .error_count = tx.error_count,
        .rejected_count = tx.rejected_count,
        .last_error = tx.last_error,
        .last_dma_error = tx.last_dma_error,
        .last_start_status = tx.last_start_status,
        .busy = tx.busy,
    };
}
