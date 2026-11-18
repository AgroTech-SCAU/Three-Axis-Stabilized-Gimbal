/**
 * @file stm32_hal_gpio.c
 * @brief PA15 USER_KEY 输入适配，不重复初始化 CubeMX GPIO
 */

#include "stm32_hal_gpio.h"

#include "main.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 读取 PA15 USER_KEY 输入电平
 * @return true 为 GPIO_PIN_SET，false 为 GPIO_PIN_RESET
 */
bool stm32_hal_user_key_read_level(void) {
    return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_SET;
}
