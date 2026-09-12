#include "stm32_hal_uart.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define UART1_TX_QUEUE_SIZE 2048u

typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t* buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t active_len;
    volatile bool busy;
} UartTxQueue;

static uint8_t s_uart1_tx_buffer[UART1_TX_QUEUE_SIZE] = { 0 };
static UartTxQueue s_uart1_tx = {
    .huart = &huart1,
    .buffer = s_uart1_tx_buffer,
    .size = UART1_TX_QUEUE_SIZE,
};

static bool uart_write_it(UartTxQueue* queue, const char* data, uint32_t len);
static void uart_tx_start(UartTxQueue* queue);
static void uart_tx_complete(UartTxQueue* queue);
static void uart_tx_error(UartTxQueue* queue);

static PiTxHalResult s_uart10_last_tx_result = PI_TX_HAL_OK;

static void (*uart1_tx_complete_callback)(void) = NULL;
static void (*uart1_rx_complete_callback)(void) = NULL;
static void (*uart1_rx_event_callback)(uint16_t size) = NULL;
static void (*uart1_error_callback)(void) = NULL;

static void (*uart5_rx_complete_callback)(void) = NULL;
static void (*uart5_rx_event_callback)(uint16_t size) = NULL;
static void (*uart5_error_callback)(void) = NULL;

static void (*uart10_rx_complete_callback)(void) = NULL;
static void (*uart10_rx_event_callback)(uint16_t size) = NULL;
static void (*uart10_error_callback)(void) = NULL;

static void (*uart7_rx_complete_callback)(void) = NULL;
static void (*uart7_error_callback)(void) = NULL;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

bool uart1_write(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, (uint16_t)len) == HAL_OK;
}

bool uart1_write_it(const char* data, uint32_t len) {
    return uart_write_it(&s_uart1_tx, data, len);
}

bool uart1_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart1, (uint8_t*)data, (uint16_t)len, 10) == HAL_OK;
}

bool uart7_write_blocking(const char* data, uint32_t len) {
    if(data == NULL || len == 0u || len > UINT16_MAX) {
        return false;
    }

    return HAL_UART_Transmit(&huart7, (uint8_t*)data, (uint16_t)len, 10) == HAL_OK;
}

bool uart10_write_blocking(const char* data, uint32_t len) {
    HAL_StatusTypeDef status;

    if(data == NULL || len == 0u || len > UINT16_MAX) {
        s_uart10_last_tx_result = PI_TX_HAL_ERROR;
        return false;
    }

    status = HAL_UART_Transmit(&huart10, (uint8_t*)data, (uint16_t)len, 10);
    switch(status) {
        case HAL_OK:
            s_uart10_last_tx_result = PI_TX_HAL_OK;
            return true;

        case HAL_BUSY:
            s_uart10_last_tx_result = PI_TX_HAL_BUSY;
            return false;

        case HAL_TIMEOUT:
            s_uart10_last_tx_result = PI_TX_HAL_TIMEOUT;
            return false;

        case HAL_ERROR:
        default:
            s_uart10_last_tx_result = PI_TX_HAL_ERROR;
            return false;
    }
}

PiTxHalResult uart10_get_last_tx_result(void) {
    return s_uart10_last_tx_result;
}

bool uart_receive_it(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    if(huart == NULL || data == NULL || len == 0u) {
        return false;
    }

    return HAL_UART_Receive_IT(huart, data, len) == HAL_OK;
}

bool uart_receive_to_idle_dma(UART_HandleTypeDef* huart, uint8_t* data, uint16_t len) {
    HAL_StatusTypeDef status;

    if(huart == NULL || data == NULL || len == 0u) {
        return false;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    status = HAL_UARTEx_ReceiveToIdle_DMA(huart, data, len);
    if(status != HAL_OK) {
        return false;
    }

    if(huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    return true;
}

bool uart_abort_receive_it(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive_IT(huart) == HAL_OK;
}

bool uart_abort_receive_dma(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return false;
    }

    return HAL_UART_AbortReceive(huart) == HAL_OK;
}

void uart_register_tx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart1_tx_complete_callback = callback;
    }
}

void uart_register_rx_complete_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart1_rx_complete_callback = callback;
    }
    else if(huart == &huart5) {
        uart5_rx_complete_callback = callback;
    }
    else if(huart == &huart10) {
        uart10_rx_complete_callback = callback;
    }
    else if(huart == &huart7) {
        uart7_rx_complete_callback = callback;
    }
}

void uart_register_rx_event_callback(UART_HandleTypeDef* huart, void (*callback)(uint16_t size)) {
    if(huart == &huart1) {
        uart1_rx_event_callback = callback;
    }
    else if(huart == &huart5) {
        uart5_rx_event_callback = callback;
    }
    else if(huart == &huart10) {
        uart10_rx_event_callback = callback;
    }
}

bool uart_rx_complete_callback_registered(UART_HandleTypeDef* huart) {
    if(huart == &huart1) {
        return uart1_rx_complete_callback != NULL;
    }
    if(huart == &huart5) {
        return uart5_rx_complete_callback != NULL;
    }
    if(huart == &huart10) {
        return uart10_rx_complete_callback != NULL;
    }
    if(huart == &huart7) {
        return uart7_rx_complete_callback != NULL;
    }
    return false;
}

void uart_register_error_callback(UART_HandleTypeDef* huart, void (*callback)(void)) {
    if(huart == &huart1) {
        uart1_error_callback = callback;
    }
    else if(huart == &huart5) {
        uart5_error_callback = callback;
    }
    else if(huart == &huart10) {
        uart10_error_callback = callback;
    }
    else if(huart == &huart7) {
        uart7_error_callback = callback;
    }
}

// ! ========================= HAL 回 调 实 现 ========================= ! //

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1) {
        uart_tx_complete(&s_uart1_tx);
        if(uart1_tx_complete_callback != NULL) {
            uart1_tx_complete_callback();
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1 && uart1_rx_complete_callback != NULL) {
        uart1_rx_complete_callback();
    }
    else if(huart == &huart5 && uart5_rx_complete_callback != NULL) {
        uart5_rx_complete_callback();
    }
    else if(huart == &huart10 && uart10_rx_complete_callback != NULL) {
        uart10_rx_complete_callback();
    }
    else if(huart == &huart7 && uart7_rx_complete_callback != NULL) {
        uart7_rx_complete_callback();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if(huart == &huart1 && uart1_rx_event_callback != NULL) {
        uart1_rx_event_callback(Size);
    }
    else if(huart == &huart5 && uart5_rx_event_callback != NULL) {
        uart5_rx_event_callback(Size);
    }
    else if(huart == &huart10 && uart10_rx_event_callback != NULL) {
        uart10_rx_event_callback(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if(huart == &huart1) {
        uart_tx_error(&s_uart1_tx);
        if(uart1_error_callback != NULL) {
            uart1_error_callback();
        }
    }
    else if(huart == &huart5 && uart5_error_callback != NULL) {
        uart5_error_callback();
    }
    else if(huart == &huart10 && uart10_error_callback != NULL) {
        uart10_error_callback();
    }
    else if(huart == &huart7 && uart7_error_callback != NULL) {
        uart7_error_callback();
    }
}


// ! ========================= USART1 非阻塞 TX 队列 ========================= ! //

static bool uart_write_it(UartTxQueue* queue, const char* data, uint32_t len) {
    uint16_t used;
    uint16_t free_len;
    uint16_t first_len;
    uint32_t primask;

    if(queue == NULL || data == NULL || len == 0u || len >= queue->size) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if(queue->head >= queue->tail) {
        used = (uint16_t)(queue->head - queue->tail);
    }
    else {
        used = (uint16_t)(queue->size - queue->tail + queue->head);
    }
    free_len = (uint16_t)(queue->size - 1u - used);
    if(len > free_len) {
        if(primask == 0u) {
            __enable_irq();
        }
        return false;
    }

    first_len = (uint16_t)(queue->size - queue->head);
    if(first_len > len) {
        first_len = (uint16_t)len;
    }
    memcpy(&queue->buffer[queue->head], data, first_len);
    if(first_len < len) {
        memcpy(queue->buffer, &data[first_len], len - first_len);
    }
    queue->head = (uint16_t)((queue->head + len) % queue->size);
    if(primask == 0u) {
        __enable_irq();
    }

    uart_tx_start(queue);
    return true;
}

static void uart_tx_start(UartTxQueue* queue) {
    uint16_t len;

    if(queue == NULL || queue->busy || queue->head == queue->tail) {
        return;
    }

    if(queue->head > queue->tail) {
        len = (uint16_t)(queue->head - queue->tail);
    }
    else {
        len = (uint16_t)(queue->size - queue->tail);
    }

    queue->active_len = len;
    queue->busy = true;
    if(HAL_UART_Transmit_IT(queue->huart, &queue->buffer[queue->tail], len) != HAL_OK) {
        queue->active_len = 0u;
        queue->busy = false;
    }
}

static void uart_tx_complete(UartTxQueue* queue) {
    if(queue == NULL || !queue->busy) {
        return;
    }

    queue->tail = (uint16_t)((queue->tail + queue->active_len) % queue->size);
    queue->active_len = 0u;
    queue->busy = false;
    uart_tx_start(queue);
}

static void uart_tx_error(UartTxQueue* queue) {
    if(queue == NULL || !queue->busy) {
        return;
    }

    queue->tail = (uint16_t)((queue->tail + queue->active_len) % queue->size);
    queue->active_len = 0u;
    queue->busy = false;
    uart_tx_start(queue);
}
