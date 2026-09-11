//
// Created by CaoKangqi on 2026/6/10.
//

#ifndef F4_FRAMEWORK_BSP_UART_H
#define F4_FRAMEWORK_BSP_UART_H

#include "stm32f4xx_hal.h"

// 统一的底层回调函数指针类型：只抛出接收数据的指针、实际长度，不涉及任何应用层特有结构体
typedef void (*BSP_UART_Callback_t)(uint8_t *pData, void *device_ptr, uint16_t Size);

typedef struct {
    uint8_t *rx_buf0;
    uint8_t *rx_buf1;
    uint16_t dma_rx_size;
    uint16_t expected_size;
    void *device_ptr;               // 摆渡指针，底层只负责保管，不关心它是啥
    BSP_UART_Callback_t resolve;
} BSP_UART_Slot_t;

// 严格按照你在 Comm_Router 里调用的参数顺序声明
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
    uint32_t parity;                // 校验位，0表示不修改
    uint32_t stopbits;              // 停止位，0表示不修改
    uint16_t expected_size;         // 接收长度
    uint8_t *rx_buf0;               // 接收缓冲区0
    uint8_t *rx_buf1;               // 接收缓冲区1
    uint16_t dma_rx_size;           // DMA接收大小
    void *device_ptr;               // 摆渡指针
    BSP_UART_Callback_t resolve;    // 数据解算
} Auto_UART_Reg_t;

/* --- UART 自动注册节点 --- */
#define UART_RX_NODE(huart_ptr, baud, parity_arg, stop_arg, exp_size, buf0, buf1, dma_size, dev_ptr_arg, callback) \
__attribute__((used, section("UART_Reg_Sec"))) \
static const Auto_UART_Reg_t MACRO_CONCAT(_uart_reg_, __LINE__) = { \
.huart = huart_ptr, \
.baudrate = baud, \
.parity = parity_arg, \
.stopbits = stop_arg, \
.expected_size = exp_size, \
.rx_buf0 = buf0, \
.rx_buf1 = buf1, \
.dma_rx_size = dma_size, \
.device_ptr = dev_ptr_arg, \
.resolve = callback \
}

void Auto_UART_Router_Init(void);

HAL_StatusTypeDef UART_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

#endif //F4_FRAMEWORK_BSP_UART_H