//
// Created by CaoKangqi on 2026/6/10.
//

#ifndef H7_FRAMEWORK_BSP_UART_H
#define H7_FRAMEWORK_BSP_UART_H

#include "stm32h7xx_hal.h"

typedef void (*BSP_UART_Callback_t)(uint8_t *pData, void *device_ptr, uint16_t Size);

typedef struct {
    uint8_t *rx_buf0;
    uint8_t *rx_buf1;
    uint16_t dma_rx_size;
    uint16_t expected_size;
    void *device_ptr;
    BSP_UART_Callback_t resolve;
} BSP_UART_Slot_t;

void BSP_UART_Register_Slot(UART_HandleTypeDef *huart,
                            uint16_t expected_size,
                            uint8_t *rx_buf0,
                            uint8_t *rx_buf1,
                            uint16_t dma_size,
                            void *device_ptr,
                            BSP_UART_Callback_t callback);

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t baudrate;              // 期望的波特率，0表示不修改
    uint16_t expected_size;
    uint8_t *rx_buf0;
    uint8_t *rx_buf1;
    uint16_t dma_rx_size;
    void *device_ptr;
    BSP_UART_Callback_t resolve;
} Auto_UART_Reg_t;

#define _MACRO_CONCAT_IMPL(a, b) a##b
#define MACRO_CONCAT(a, b) _MACRO_CONCAT_IMPL(a, b)

/* --- UART 自动注册节点 --- */
#define UART_RX_NODE(huart_ptr, baud, exp_size, buf0, buf1, dma_size, dev_ptr_arg, callback) \
__attribute__((used, section("UART_Reg_Sec"))) \
static const Auto_UART_Reg_t MACRO_CONCAT(_uart_reg_, __LINE__) = { \
.huart = huart_ptr, \
.baudrate = baud, \
.expected_size = exp_size, \
.rx_buf0 = buf0, \
.rx_buf1 = buf1, \
.dma_rx_size = dma_size, \
.device_ptr = dev_ptr_arg, \
.resolve = callback \
}

void Auto_UART_Router_Init(void);

HAL_StatusTypeDef UART_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#endif //H7_FRAMEWORK_BSP_UART_H