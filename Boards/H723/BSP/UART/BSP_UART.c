//
// Created by CaoKangqi on 2026/6/10.
//
#include "stm32h7xx_hal.h"
#include "BSP_UART.h"
#include <string.h>

extern const Auto_UART_Reg_t __start_UART_Reg_Sec;
extern const Auto_UART_Reg_t __stop_UART_Reg_Sec;

#define MAX_UART_BUS_NUM  11
static BSP_UART_HD_t *hd_ports[MAX_UART_BUS_NUM];

void Auto_UART_Router_Init(void)
{
    const Auto_UART_Reg_t *node = &__start_UART_Reg_Sec;
    for (; node < &__stop_UART_Reg_Sec; node++)
    {
        // 为了防止因为CubeMX生成代码导致波特率被更改，这里加入手动设置波特率的功能
        // 如果传入了非零的波特率，且与当前初始化波特率不同，则进行重配置
        if (node->baudrate != 0 || node->parity != 0 || node->stopbits != 0)
        {
            HAL_UART_Abort(node->huart);
            if (node->baudrate != 0) node->huart->Init.BaudRate = node->baudrate;
            if (node->parity   != 0) node->huart->Init.Parity   = node->parity;
            if (node->stopbits != 0) node->huart->Init.StopBits = node->stopbits;
            HAL_UART_Init(node->huart);
        }
        BSP_UART_Register_Slot(node->huart, node->expected_size,
                               node->rx_buf0, node->rx_buf1,
                               node->dma_rx_size, node->device_ptr, node->resolve);
    }
}


// 驱动路由槽位映射表
static BSP_UART_Slot_t BSP_UART_Table[MAX_UART_BUS_NUM] = {0};
static uint8_t g_uart_registered_mask[MAX_UART_BUS_NUM] = {0};

/**
 * @brief 辅助函数：根据寄存器基地址快速获取数组索引
 */
static inline uint8_t Get_UART_Bus_Index(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return 0;
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
    if (idx == 0 || hd_ports[idx] != NULL) return;

    BSP_UART_Table[idx].rx_buf0       = rx_buf0;
    BSP_UART_Table[idx].rx_buf1       = rx_buf1;
    BSP_UART_Table[idx].dma_rx_size   = dma_size;
    BSP_UART_Table[idx].expected_size = expected_size;
    BSP_UART_Table[idx].device_ptr    = device_ptr; // 存下应用层变量地址
    BSP_UART_Table[idx].resolve       = callback;
    g_uart_registered_mask[idx]       = 1;

    UART_ReceiveToIdle_DMA(huart, rx_buf0, dma_size);
}

/* 清错误和接收残留；额外标志由调用方指定。 */
static void UART_ClearRx(UART_HandleTypeDef *huart, uint32_t extra_flags)
{
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF |
                               UART_CLEAR_PEF | extra_flags);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    (void)huart->Instance->RDR;
}

/* 共用 DMA 接收启动；定长接收不清残留，避免丢掉刚到的反馈。 */
static HAL_StatusTypeDef UART_ReceiveDMA(UART_HandleTypeDef *huart,
    uint8_t *data, uint16_t size, uint8_t to_idle)
{
    if (huart == NULL || huart->Instance == NULL || huart->hdmarx == NULL ||
        data == NULL || size == 0) return HAL_ERROR;
    if (to_idle) UART_ClearRx(huart, 0);
    HAL_StatusTypeDef result = to_idle ? HAL_UARTEx_ReceiveToIdle_DMA(huart, data, size)
                                     : HAL_UART_Receive_DMA(huart, data, size);
    if (result != HAL_OK) return HAL_ERROR;
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return HAL_OK;
}

/* 保持原有空闲接收接口和返回值。 */
HAL_StatusTypeDef UART_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    return UART_ReceiveDMA(huart, pData, Size, 1);
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

enum { UART_HD_IDLE, UART_HD_TX, UART_HD_RX, UART_HD_FINISH };

/* DMA1/2 使用 D2 SRAM；每个缓冲独占完整缓存行。 */
static uint8_t hd_tx[MAX_UART_BUS_NUM][BSP_UART_HD_MAX_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));
static uint8_t hd_rx[MAX_UART_BUS_NUM][BSP_UART_HD_MAX_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));

static BSP_UART_HD_t *UART_HD_Find(UART_HandleTypeDef *h)
{
    BSP_UART_HD_t *port = hd_ports[Get_UART_Bus_Index(h)];
    return port != NULL && port->huart == h ? port : NULL;
}

static BSP_UART_HD_Result_t UART_HD_Check(UART_HandleTypeDef *h)
{
    if (h == NULL || h->Instance == NULL) return BSP_UART_HD_INVALID_ARGUMENT;
    uint8_t idx = Get_UART_Bus_Index(h);
    if (!idx || g_uart_registered_mask[idx]) return BSP_UART_HD_INVALID_CONFIG;
    for (const Auto_UART_Reg_t *n = &__start_UART_Reg_Sec; n < &__stop_UART_Reg_Sec; ++n)
        if (n->huart->Instance == h->Instance) return BSP_UART_HD_INVALID_CONFIG;
    if (!(h->Instance->CR1 & USART_CR1_UE) || !(h->Instance->CR3 & USART_CR3_HDSEL) ||
        h->Init.WordLength != UART_WORDLENGTH_8B || h->Init.Parity != UART_PARITY_NONE ||
        h->Init.StopBits != UART_STOPBITS_1 || h->Init.HwFlowCtl != UART_HWCONTROL_NONE ||
        (h->Instance->CR1 & USART_CR1_FIFOEN)) return BSP_UART_HD_INVALID_CONFIG;
    DMA_HandleTypeDef *dma[] = {h->hdmatx, h->hdmarx};
    for (unsigned i = 0; i < 2; ++i) {
        if (dma[i] == NULL || !IS_DMA_STREAM_INSTANCE(dma[i]->Instance) || dma[i]->Parent != h ||
            dma[i]->Init.Mode != DMA_NORMAL || dma[i]->Init.MemInc != DMA_MINC_ENABLE ||
            dma[i]->Init.PeriphInc != DMA_PINC_DISABLE ||
            dma[i]->Init.MemDataAlignment != DMA_MDATAALIGN_BYTE ||
            dma[i]->Init.PeriphDataAlignment != DMA_PDATAALIGN_BYTE ||
            dma[i]->Init.Direction != (i ? DMA_PERIPH_TO_MEMORY : DMA_MEMORY_TO_PERIPH))
            return BSP_UART_HD_INVALID_CONFIG;
        if (dma[i]->State != HAL_DMA_STATE_READY || dma[i]->Lock != HAL_UNLOCKED)
            return BSP_UART_HD_BUSY;
    }
    if (h->gState != HAL_UART_STATE_READY || h->RxState != HAL_UART_STATE_READY ||
        h->Lock != HAL_UNLOCKED ||
        (h->Instance->CR1 & (USART_CR1_RXNEIE_RXFNEIE | USART_CR1_TXEIE_TXFNFIE |
                            USART_CR1_TCIE | USART_CR1_PEIE | USART_CR1_IDLEIE | USART_CR1_RTOIE)) ||
        (h->Instance->CR3 & (USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_EIE |
                            USART_CR3_RXFTIE | USART_CR3_TXFTIE))) return BSP_UART_HD_BUSY;
    return BSP_UART_HD_OK;
}

/* 保存错误后才能清理；HAL 可能已清除硬件溢出标志。 */
static void UART_HD_SaveError(BSP_UART_HD_t *p)
{
    uint32_t isr = p->huart->Instance->ISR;
    p->hal_error |= p->huart->ErrorCode;
    if (isr & UART_FLAG_PE) p->hal_error |= HAL_UART_ERROR_PE;
    if (isr & UART_FLAG_FE) p->hal_error |= HAL_UART_ERROR_FE;
    if (isr & UART_FLAG_NE) p->hal_error |= HAL_UART_ERROR_NE;
    if (isr & UART_FLAG_ORE) p->hal_error |= HAL_UART_ERROR_ORE;
}

/* 半双工只记录错误，任务 Poll 负责恢复；普通串口沿用原接收流程。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h)
{
    BSP_UART_HD_t *p = UART_HD_Find(h);
    if (p != NULL) {
        if (p->phase != UART_HD_IDLE) {
            UART_HD_SaveError(p);
            if (p->phase == UART_HD_TX) p->result = BSP_UART_HD_TX_ERROR;
            else if (p->result != BSP_UART_HD_TX_ERROR && p->result != BSP_UART_HD_TX_TIMEOUT)
                p->result = BSP_UART_HD_RX_ERROR;
            p->phase = UART_HD_FINISH;
        }
        return;
    }
    uint8_t idx = Get_UART_Bus_Index(h);
    if (idx && g_uart_registered_mask[idx])
        UART_ReceiveToIdle_DMA(h, BSP_UART_Table[idx].rx_buf0, BSP_UART_Table[idx].dma_rx_size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    BSP_UART_HD_t *p = UART_HD_Find(h);
    if (p == NULL || p->phase != UART_HD_TX) return;
    /* Normal DMA 的此回调由 UART TC 触发，此时才启动接收过程。 */
    p->phase_tick = HAL_GetTick();
    p->phase = UART_HD_RX;
    uint8_t idx = Get_UART_Bus_Index(h);
    if (UART_ReceiveDMA(h, hd_rx[idx], p->rx_expected, 0) == HAL_OK) {
        p->rx_started = 1;
        /* DMA 就绪后再开 RE，切换后不清接收数据。 */
        if (HAL_HalfDuplex_EnableReceiver(h) == HAL_OK) return;
    }
    p->result = BSP_UART_HD_RX_ERROR;
    p->phase = UART_HD_FINISH;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    BSP_UART_HD_t *p = UART_HD_Find(h);
    if (p == NULL || p->phase != UART_HD_RX) return;
    p->result = BSP_UART_HD_OK;
    p->phase = UART_HD_FINISH;
}

BSP_UART_HD_Result_t BSP_UART_HalfDuplex_Init(BSP_UART_HD_t *p,
    UART_HandleTypeDef *h, uint32_t tx_ms, uint32_t rx_ms)
{
    if (p == NULL || !tx_ms || !rx_ms || tx_ms == HAL_MAX_DELAY || rx_ms == HAL_MAX_DELAY)
        return BSP_UART_HD_INVALID_ARGUMENT;
    BSP_UART_HD_Result_t result = UART_HD_Check(h);
    if (result != BSP_UART_HD_OK) return result;
    uint8_t idx = Get_UART_Bus_Index(h);
    for (unsigned i = 1; i < MAX_UART_BUS_NUM; ++i)
        if (hd_ports[i] != NULL && (hd_ports[i] == p || i == idx)) return BSP_UART_HD_BUSY;
    if (HAL_HalfDuplex_EnableReceiver(h) != HAL_OK) return BSP_UART_HD_BUSY;
    *p = (BSP_UART_HD_t){.huart = h, .tx_timeout_ms = tx_ms, .rx_timeout_ms = rx_ms,
                        .result = BSP_UART_HD_IDLE};
    hd_ports[idx] = p;
    return BSP_UART_HD_OK;
}

BSP_UART_HD_Result_t BSP_UART_HalfDuplex_Start(BSP_UART_HD_t *p,
    const uint8_t *tx, uint16_t tx_length, uint8_t *rx, uint16_t rx_length)
{
    if (p == NULL || UART_HD_Find(p->huart) != p) return BSP_UART_HD_INVALID_ARGUMENT;
    if (p->phase != UART_HD_IDLE) return BSP_UART_HD_BUSY;
    p->rx_length = 0;
    p->hal_error = HAL_UART_ERROR_NONE;
    p->result = BSP_UART_HD_INVALID_ARGUMENT;
    if (tx == NULL || rx == NULL || !tx_length || !rx_length ||
        tx_length > BSP_UART_HD_MAX_SIZE || rx_length > BSP_UART_HD_MAX_SIZE) return p->result;
    p->result = UART_HD_Check(p->huart);
    if (p->result != BSP_UART_HD_OK) return p->result;
    UART_HandleTypeDef *h = p->huart;
    uint8_t idx = Get_UART_Bus_Index(h);
    memcpy(hd_tx[idx], tx, tx_length);
    SCB_CleanDCache_by_Addr((uint32_t *)hd_tx[idx], BSP_UART_HD_MAX_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)hd_rx[idx], BSP_UART_HD_MAX_SIZE);
    p->rx_buffer = rx;
    p->rx_expected = rx_length;
    p->rx_started = 0;
    p->result = BSP_UART_HD_BUSY;
    p->phase_tick = HAL_GetTick();
    p->phase = UART_HD_TX;
    /* 先清残留，再启动 TX；接收由 TC 回调接续。 */
    if (HAL_HalfDuplex_EnableTransmitter(h) != HAL_OK) goto tx_error;
    UART_ClearRx(h, UART_CLEAR_IDLEF | UART_CLEAR_RTOF);
    if (HAL_UART_Transmit_DMA(h, hd_tx[idx], tx_length) != HAL_OK) goto tx_error;
    __HAL_DMA_DISABLE_IT(h->hdmatx, DMA_IT_HT);
    return BSP_UART_HD_OK;
tx_error:
    p->result = BSP_UART_HD_TX_ERROR;
    p->phase = UART_HD_FINISH;
    return BSP_UART_HalfDuplex_Poll(p);
}

BSP_UART_HD_Result_t BSP_UART_HalfDuplex_Poll(BSP_UART_HD_t *p)
{
    if (p == NULL || UART_HD_Find(p->huart) != p) return BSP_UART_HD_INVALID_ARGUMENT;
    /* 短临界区只判定超时，避免与完成中断竞争；清理时保持中断开启。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t phase = p->phase;
    if ((phase == UART_HD_TX || phase == UART_HD_RX) &&
        (uint32_t)(HAL_GetTick() - p->phase_tick) >=
            (phase == UART_HD_TX ? p->tx_timeout_ms : p->rx_timeout_ms)) {
        p->result = phase == UART_HD_TX ? BSP_UART_HD_TX_TIMEOUT : BSP_UART_HD_RX_TIMEOUT;
        p->phase = UART_HD_FINISH;
    }
    phase = p->phase;
    __set_PRIMASK(primask);
    /* 退出临界区后可能刚好收到完成中断；下一次 Poll 再提交完整结果。 */
    if (phase != UART_HD_FINISH) return phase == UART_HD_IDLE ? p->result : BSP_UART_HD_BUSY;

    UART_HandleTypeDef *h = p->huart;
    UART_HD_SaveError(p);
    /* HAL Abort 停止两路 DMA 并清除残留；失败时保留占用，禁止复用缓冲。 */
    if (HAL_UART_Abort(h) != HAL_OK ||
        (((DMA_Stream_TypeDef *)h->hdmatx->Instance)->CR & DMA_SxCR_EN) ||
        (((DMA_Stream_TypeDef *)h->hdmarx->Instance)->CR & DMA_SxCR_EN)) {
        p->hal_error |= HAL_UART_ERROR_DMA;
        goto rx_error;
    }
    if (HAL_HalfDuplex_EnableReceiver(h) != HAL_OK) goto rx_error;
    uint32_t remaining = __HAL_DMA_GET_COUNTER(h->hdmarx);
    p->rx_length = p->rx_started && remaining <= p->rx_expected ? p->rx_expected - remaining : 0;
    if (p->hal_error && p->result != BSP_UART_HD_TX_ERROR && p->result != BSP_UART_HD_TX_TIMEOUT)
        p->result = BSP_UART_HD_RX_ERROR;
    if (p->result == BSP_UART_HD_RX_TIMEOUT && p->rx_length) p->result = BSP_UART_HD_LENGTH_ERROR;
    if (p->result == BSP_UART_HD_OK && p->rx_length != p->rx_expected) p->result = BSP_UART_HD_LENGTH_ERROR;
    uint8_t idx = Get_UART_Bus_Index(h);
    SCB_InvalidateDCache_by_Addr((uint32_t *)hd_rx[idx], BSP_UART_HD_MAX_SIZE);
    if (p->rx_buffer != NULL) memcpy(p->rx_buffer, hd_rx[idx], p->rx_length);
    else p->rx_length = 0;  /* 已报告恢复失败后，不再写入调用者缓冲。 */
    p->phase = UART_HD_IDLE;
    return p->result;

rx_error:
    p->result = BSP_UART_HD_RX_ERROR;
    p->rx_buffer = NULL;
    return p->result;
}
