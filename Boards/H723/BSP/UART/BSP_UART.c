//
// Created by CaoKangqi on 2026/6/10.
//
#include "stm32h7xx_hal.h"
#include "BSP_UART.h"

extern const Auto_UART_Reg_t __start_UART_Reg_Sec;
extern const Auto_UART_Reg_t __stop_UART_Reg_Sec;

void Auto_UART_Router_Init(void)
{
    const Auto_UART_Reg_t *node = &__start_UART_Reg_Sec;
    for (; node < &__stop_UART_Reg_Sec; node++)
    {
        // 为了防止因为CubeMX生成代码导致波特率被更改，这里加入手动设置波特率的功能
        // 如果传入了非零的波特率，且与当前初始化波特率不同，则进行重配置
        if (node->baudrate != 0 && node->huart->Init.BaudRate != node->baudrate)
        {
            // 中止可能正在进行的传输
            HAL_UART_Abort(node->huart);
            // 修改 Init 结构体里的波特率
            node->huart->Init.BaudRate = node->baudrate;
            // 重新调用 HAL_UART_Init，底层会重新写入 BRR 寄存器
            if (HAL_UART_Init(node->huart) != HAL_OK)
            {
                continue;
            }
        }
        BSP_UART_Register_Slot(node->huart, node->expected_size,
                               node->rx_buf0, node->rx_buf1,
                               node->dma_rx_size, node->device_ptr, node->resolve);
    }
}

#define MAX_UART_BUS_NUM  11

// 驱动路由槽位映射表
static BSP_UART_Slot_t BSP_UART_Table[MAX_UART_BUS_NUM] = {0};
static uint8_t g_uart_registered_mask[MAX_UART_BUS_NUM] = {0};

/**
 * @brief 辅助函数：根据寄存器基地址快速获取数组索引
 */
static inline uint8_t Get_UART_Bus_Index(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)  return 1;
    if (huart->Instance == USART2)  return 2;
    if (huart->Instance == USART3)  return 3;
    if (huart->Instance == UART4)   return 4;
    if (huart->Instance == UART5)   return 5;
    if (huart->Instance == USART6)  return 6;
    if (huart->Instance == UART7)   return 7;
    if (huart->Instance == UART8)   return 8;
    if (huart->Instance == UART9)   return 9;
    if (huart->Instance == USART10) return 10;
    return 0;
}

/**
 * @brief   UART 槽位注册函数：完成硬件层信息登记并直接开启 DMA 接收
 */
void BSP_UART_Register_Slot(UART_HandleTypeDef *huart,
                            uint16_t expected_size,
                            uint8_t *rx_buf0,
                            uint8_t *rx_buf1,
                            uint16_t dma_size,
                            void *device_ptr,
                            BSP_UART_Callback_t callback)
{
    uint8_t idx = Get_UART_Bus_Index(huart);
    if (idx == 0 || idx >= MAX_UART_BUS_NUM) return;

    BSP_UART_Table[idx].rx_buf0       = rx_buf0;
    BSP_UART_Table[idx].rx_buf1       = rx_buf1;
    BSP_UART_Table[idx].dma_rx_size   = dma_size;
    BSP_UART_Table[idx].expected_size = expected_size;
    BSP_UART_Table[idx].device_ptr    = device_ptr; // 存下应用层变量地址
    BSP_UART_Table[idx].resolve       = callback;
    g_uart_registered_mask[idx]       = 1;

    UART_ReceiveToIdle_DMA(huart, rx_buf0, dma_size);
}

/**
 * @brief   UART DMA 空闲中断接收
 * @param  huart: HAL串口句柄指针
 * @param  pData: 接收缓冲区指针
 * @param  Size:  预期的最大接收字节数
 * @return HAL_StatusTypeDef: HAL_OK 启动成功, HAL_ERROR 配置错误或句柄为空
 */
HAL_StatusTypeDef UART_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size) {
    // 防止传入空指针导致 HardFault
    if (huart == NULL || pData == NULL || Size == 0) {
        return HAL_ERROR;
    }
    // 检查 DMA 句柄是否配置：防止 CubeMX 没开 DMA 导致死机
    if (huart->hdmarx == NULL) {
        return HAL_ERROR;
    }
    // 彻底清除硬件错误标志
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
    // 通过 RQR 寄存器请求丢弃当前接收行为
    huart->Instance->RQR |= USART_RQR_RXFRQ;
    // 读出 RDR 缓存
    volatile uint32_t tmp = huart->Instance->RDR;
    (void)tmp;
    // 重新启动 DMA 接收
    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size) != HAL_OK) {
        return HAL_ERROR;
    }
    // 关闭 DMA 半传输中断（HAL 库启动后默认会开启，这里必须关闭）
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return HAL_OK;
}

/**
 * @brief HAL库空闲中断回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint8_t idx = Get_UART_Bus_Index(huart);
    if (idx == 0 || g_uart_registered_mask[idx] == 0) return;

    BSP_UART_Slot_t *slot = &BSP_UART_Table[idx];
    uint8_t *pData = huart->pRxBuffPtr;

    uint8_t *next_buf = slot->rx_buf0;
    if (slot->rx_buf1 != NULL) {
        next_buf = (pData == slot->rx_buf0) ? slot->rx_buf1 : slot->rx_buf0;
    }
    UART_ReceiveToIdle_DMA(huart, next_buf, slot->dma_rx_size);

    if (slot->expected_size != 0 && Size != slot->expected_size) return;
    if (slot->resolve != NULL) {
        slot->resolve(pData, slot->device_ptr, Size);
    }
}

/**
 * @brief HAL库串口错误回调函数
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t idx = Get_UART_Bus_Index(huart);
    if (idx == 0 || g_uart_registered_mask[idx] == 0) return;
    UART_ReceiveToIdle_DMA(huart, BSP_UART_Table[idx].rx_buf0, BSP_UART_Table[idx].dma_rx_size);
}