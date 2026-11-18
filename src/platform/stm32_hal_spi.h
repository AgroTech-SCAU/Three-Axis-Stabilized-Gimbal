#ifndef _stm32_hal_spi_h_
#define _stm32_hal_spi_h_

#include <stdbool.h>
#include <stdint.h>

#include "main.h" // IWYU pragma: keep


// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief SPI 阻塞超时时间，单位 ms
 */
#define SPI_TIMEOUT 10

/**
 * @brief RGB 灯带所用 SPI6 句柄
 */
extern SPI_HandleTypeDef hspi6;
/**
 * @brief BMI088 所用 SPI2 句柄
 */
extern SPI_HandleTypeDef hspi2;

/** @brief SPI6 RGB 发送诊断；接受提交与真实完成分别计数 */
typedef struct {
    uint32_t started_count;     /**< HAL 接受的 DMA 提交数 */
    uint32_t completed_count;   /**< SPI EOT 成功完成数，不含错误回调 */
    uint32_t error_count;       /**< 实际 SPI6/DMA 错误回调累计数 */
    uint32_t rejected_count;    /**< 参数或 HAL 拒绝的提交累计数 */
    uint32_t last_error;        /**< 最近异步错误的 HAL SPI 错误位 */
    uint32_t last_dma_error;    /**< 最近异步错误的 HAL DMA 错误位 */
    uint32_t last_start_status; /**< 最近提交的 HAL_StatusTypeDef 数值 */
    bool busy;                  /**< 当前有本端口接受但尚未完成的 DMA 帧 */
} Spi6TxStats;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 通过 SPI6 发送一段 DMA 数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool spi_write(const uint8_t* data, uint32_t len);

/**
 * @brief 在短临界区复制 SPI6 发送统计，保留调用前的中断屏蔽状态
 * @param out 快照输出地址；NULL 时不操作
 */
void spi6_get_tx_stats(Spi6TxStats* out);

/**
 * @brief 注册 SPI 发送完成回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_tx_complete_callback(SPI_HandleTypeDef* hspi, void (*callback)(SPI_HandleTypeDef* hspi));

/**
 * @brief 注册 SPI 收发完成回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_txrx_complete_callback(SPI_HandleTypeDef* hspi, void (*callback)(SPI_HandleTypeDef* hspi));

/**
 * @brief 注册 SPI2 或 SPI6 独立错误回调；错误不会伪装成发送成功
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_error_callback(SPI_HandleTypeDef* hspi, void (*callback)(SPI_HandleTypeDef* hspi));

#endif
