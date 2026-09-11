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
    uint32_t parity;                // 校验位，0表示不修改
    uint32_t stopbits;              // 停止位，0表示不修改
    uint16_t expected_size;         // 接收长度
    uint8_t *rx_buf0;               // 接收缓冲区0
    uint8_t *rx_buf1;               // 接收缓冲区1
    uint16_t dma_rx_size;           // DMA接收大小
    void *device_ptr;               // 摆渡指针
    BSP_UART_Callback_t resolve;    // 数据解算
} Auto_UART_Reg_t;

#define _MACRO_CONCAT_IMPL(a, b) a##b
#define MACRO_CONCAT(a, b) _MACRO_CONCAT_IMPL(a, b)

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

/* 单线半双工 DMA：端口对象须长期有效，同一端口由一个任务管理。
 * 以下接口从主循环或任务调用，不在中断中调用。 */
#define BSP_UART_HD_MAX_SIZE 64U
typedef enum {
    BSP_UART_HD_OK = 0, BSP_UART_HD_INVALID_ARGUMENT, BSP_UART_HD_INVALID_CONFIG,
    BSP_UART_HD_BUSY, BSP_UART_HD_TX_ERROR, BSP_UART_HD_RX_TIMEOUT,
    BSP_UART_HD_LENGTH_ERROR, BSP_UART_HD_RX_ERROR,
    BSP_UART_HD_IDLE, BSP_UART_HD_TX_TIMEOUT
} BSP_UART_HD_Result_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
    uint16_t rx_length;                /* Poll 完成后，本次实际长度。 */
    volatile uint32_t hal_error;
    volatile BSP_UART_HD_Result_t result;
    /* 以下字段由 BSP 维护，App 不应修改。 */
    volatile uint32_t phase_tick;
    volatile uint8_t phase;
    uint8_t rx_started;
    uint8_t *rx_buffer;
    uint16_t rx_expected;
} BSP_UART_HD_t;

/* 绑定 CubeMX 已初始化的 8N1 半双工串口，TX/RX DMA 均须为 Normal 字节模式。
 * 不重初始化串口；不可同时使用 UART_RX_NODE 或 BSP_UART_Register_Slot。 */
BSP_UART_HD_Result_t BSP_UART_HalfDuplex_Init(BSP_UART_HD_t *port,
    UART_HandleTypeDef *huart, uint32_t tx_timeout_ms, uint32_t rx_timeout_ms);
/* 非阻塞启动：OK 仅表示已启动。TX 当场复制；RX 缓冲须保留到 Poll 完成。
 * 两种长度均为 1～BSP_UART_HD_MAX_SIZE；内部缓冲已处理 DMA 内存和缓存限制。 */
BSP_UART_HD_Result_t BSP_UART_HalfDuplex_Start(BSP_UART_HD_t *port,
    const uint8_t *tx, uint16_t tx_length, uint8_t *rx, uint16_t rx_length);
/* 在任务中周期调用，BUSY 表示未完成，可通过 RTOS 延时让出 CPU。
 * 超时从各阶段开始计时，在 Poll 中处理；不得只等 result 而不调用 Poll。
 * 仅 Poll 返回 OK 后才解析 RX；恢复失败时禁止重发，继续 Poll 重试恢复。 */
BSP_UART_HD_Result_t BSP_UART_HalfDuplex_Poll(BSP_UART_HD_t *port);

#endif //H7_FRAMEWORK_BSP_UART_H
