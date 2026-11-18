#include "stm32_hal_spi.h"

#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief SPI6 发送完成回调
 */
static void (*spi_tx_complete_callback)(SPI_HandleTypeDef* hspi) = 0;
/**
 * @brief SPI2 收发完成回调
 */
static void (*spi_txrx_complete_callback)(SPI_HandleTypeDef* hspi) = 0;
/**
 * @brief SPI2 错误回调
 */
static void (*spi_error_callback)(SPI_HandleTypeDef* hspi) = 0;
/** @brief SPI6 独立错误回调；不与成功完成混用 */
static void (*spi6_error_callback)(SPI_HandleTypeDef* hspi) = 0;
static volatile Spi6TxStats spi6_tx_stats;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 通过 SPI6 发送一段 DMA 数据
 * @param data 数据缓冲区
 * @param len 数据长度，单位 byte
 * @return bool `true` 表示发送启动成功
 */
bool spi_write(const uint8_t* data, uint32_t len) {
    uint32_t primask = __get_PRIMASK();
    HAL_StatusTypeDef status;

    __disable_irq();
    if((data == 0) || (len == 0U) || (len > UINT16_MAX)) {
        ++spi6_tx_stats.rejected_count;
        spi6_tx_stats.last_start_status = HAL_ERROR;
        if(primask == 0u) {
            __enable_irq();
        }
        return false;
    }

    if(spi6_tx_stats.busy) {
        ++spi6_tx_stats.rejected_count;
        spi6_tx_stats.last_start_status = HAL_BUSY;
        if(primask == 0u) {
            __enable_irq();
        }
        return false;
    }
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
    /* D-cache 已开启时先清理发送缓存，保证 BDMA 读取最新数据 */
    if((SCB->CCR & SCB_CCR_DC_Msk) != 0u) {
        uintptr_t start = (uintptr_t)data & ~(uintptr_t)31u;
        uintptr_t end = ((uintptr_t)data + len + 31u) & ~(uintptr_t)31u;
        SCB_CleanDCache_by_Addr((uint32_t*)start, (int32_t)(end - start));
    }
#endif
    __DMB();
    spi6_tx_stats.busy = true;
    status = HAL_SPI_Transmit_DMA(&hspi6, data, (uint16_t)len);
    spi6_tx_stats.last_start_status = (uint32_t)status;
    if(status == HAL_OK) {
        ++spi6_tx_stats.started_count;
    }
    else {
        spi6_tx_stats.busy = false;
        ++spi6_tx_stats.rejected_count;
    }
    if(primask == 0u) {
        __enable_irq();
    }
    return status == HAL_OK;
}

void spi6_get_tx_stats(Spi6TxStats* out) {
    if(out == NULL) {
        return;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = spi6_tx_stats;
    if(primask == 0u) {
        __enable_irq();
    }
}

/**
 * @brief 注册 SPI 发送完成回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_tx_complete_callback(SPI_HandleTypeDef* hspi, void (*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi6) {
        spi_tx_complete_callback = callback;
    }
}

/**
 * @brief 注册 SPI 收发完成回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_txrx_complete_callback(SPI_HandleTypeDef* hspi, void (*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi2) {
        spi_txrx_complete_callback = callback;
    }
}

/**
 * @brief 注册 SPI 错误回调
 * @param hspi SPI 句柄
 * @param callback 回调函数
 */
void spi_register_error_callback(SPI_HandleTypeDef* hspi, void (*callback)(SPI_HandleTypeDef* hspi)) {
    if(hspi == &hspi2) {
        spi_error_callback = callback;
    }
    else if(hspi == &hspi6) {
        spi6_error_callback = callback;
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief HAL SPI 发送完成回调入口
 * @param hspi 触发回调的 SPI 句柄
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi6 && spi6_tx_stats.busy) {
        spi6_tx_stats.busy = false;
        ++spi6_tx_stats.completed_count;
        if(spi_tx_complete_callback != 0) {
            spi_tx_complete_callback(hspi);
        }
    }
}

/**
 * @brief HAL SPI 收发完成回调入口
 * @param hspi 触发回调的 SPI 句柄
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi2) {
        if(spi_txrx_complete_callback != 0) {
            spi_txrx_complete_callback(hspi);
        }
    }
}

/**
 * @brief HAL SPI 错误回调入口
 * @param hspi 触发回调的 SPI 句柄
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
    if(hspi == &hspi2) {
        if(spi_error_callback != 0) {
            spi_error_callback(hspi);
        }
    }

    if(hspi == &hspi6) {
        ++spi6_tx_stats.error_count;
        spi6_tx_stats.last_error = hspi->ErrorCode;
        spi6_tx_stats.last_dma_error = hspi->hdmatx != NULL ? hspi->hdmatx->ErrorCode : 0u;
        spi6_tx_stats.busy = false;
        if(spi6_error_callback != 0) {
            spi6_error_callback(hspi);
        }
    }
}
