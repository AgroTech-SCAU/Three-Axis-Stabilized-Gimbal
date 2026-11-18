#ifndef _stm32_hal_gpio_h_
#define _stm32_hal_gpio_h_

/**
 * @file stm32_hal_gpio.h
 * @brief 用户按键 GPIO 的板级读取接口
 */

#include <stdbool.h>

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 读取 CubeMX 已配置的 USER_KEY 原始电平
 * @return true 表示高电平；按键有效极性由 service 配置决定
 */
bool stm32_hal_user_key_read_level(void);

#endif
