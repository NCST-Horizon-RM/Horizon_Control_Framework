/**
 * @file BSP_RS485.c
 * @brief H723 implementation of the RS485 transport interface.
 */

#include "BSP_RS485.h"

#include <string.h>

typedef struct {
    bool initialized;
    BSP_RS485_Config_t config;
    BSP_RS485_FrameCallback_t frame_callback;
    BSP_RS485_TimeoutCallback_t timeout_callback;
    void *context;

    volatile uint32_t last_rx_tick;
    volatile uint32_t frame_tick;
    volatile uint16_t rx_length;
    volatile bool rx_overflow;
    uint16_t dma_remaining;
    bool waiting_response;
    bool recovered_this_poll;
    uint32_t request_tick;

    uint8_t rx_frame[BSP_RS485_MAX_FRAME_SIZE];
    BSP_RS485_Diagnostics_t diagnostics;
} BSP_RS485_Port_t;

/* DMA1/2 cannot access the H723 default DTCM region. */
static uint8_t g_rx_dma[BSP_RS485_MAX_PORTS][2][BSP_RS485_MAX_FRAME_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));
static uint8_t g_tx_dma[BSP_RS485_MAX_PORTS][BSP_RS485_MAX_FRAME_SIZE]
    __attribute__((section(".RAM_D2"), aligned(32)));
static BSP_RS485_Port_t g_ports[BSP_RS485_MAX_PORTS];

static bool Port_IsValid(BSP_RS485_PortId_t port)
{
    return port < BSP_RS485_MAX_PORTS && g_ports[port].initialized;
}

static uint32_t Critical_Enter(void)
{
#if defined(__arm__) || defined(__thumb__)
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
#else
    return 0U;
#endif
}

static void Critical_Exit(uint32_t primask)
{
#if defined(__arm__) || defined(__thumb__)
    if (primask == 0U) __enable_irq();
#else
    (void)primask;
#endif
}

static void Cache_InvalidateRx(const uint8_t *data)
{
#if defined(STM32H723xx)
    /* DMA buffers are 32-byte aligned and exactly one cache line long. */
    SCB_InvalidateDCache_by_Addr((uint32_t *)data, BSP_RS485_MAX_FRAME_SIZE);
#else
    (void)data;
#endif
}

static void Cache_CleanTx(const uint8_t *data)
{
#if defined(STM32H723xx)
    SCB_CleanDCache_by_Addr((uint32_t *)data, BSP_RS485_MAX_FRAME_SIZE);
#else
    (void)data;
#endif
}

static void ClearRxState(BSP_RS485_Port_t *port, uint32_t now_ms)
{
    uint32_t primask = Critical_Enter();
    port->rx_length = 0U;
    port->rx_overflow = false;
    port->last_rx_tick = now_ms;
    port->frame_tick = now_ms;
    port->dma_remaining = BSP_RS485_MAX_FRAME_SIZE;
    Critical_Exit(primask);
}

static BSP_RS485_Status_e RestartRx(BSP_RS485_PortId_t id, uint32_t now_ms)
{
    BSP_RS485_Port_t *port = &g_ports[id];
    HAL_UART_AbortReceive(port->config.uart);
    ClearRxState(port, now_ms);

    if (UART_ReceiveToIdle_DMA(port->config.uart, g_rx_dma[id][0],
                               BSP_RS485_MAX_FRAME_SIZE) != HAL_OK)
    {
        port->diagnostics.io_error_count++;
        return BSP_RS485_IO_ERROR;
    }
    return BSP_RS485_OK;
}

static void BSP_RS485_UartRx(uint8_t *data, void *context, uint16_t length)
{
    BSP_RS485_Port_t *port = context;
    if (port == NULL || !port->initialized || data == NULL || length == 0U) return;

    Cache_InvalidateRx(data);
    uint32_t now_ms = HAL_GetTick();
    port->last_rx_tick = now_ms;
    port->frame_tick = now_ms;

    if (port->rx_overflow) return;
    if (length > BSP_RS485_MAX_FRAME_SIZE - port->rx_length)
    {
        length = (uint16_t)(BSP_RS485_MAX_FRAME_SIZE - port->rx_length);
        port->rx_overflow = true;
    }
    if (length != 0U)
    {
        memcpy(port->rx_frame + port->rx_length, data, length);
        port->rx_length = (uint16_t)(port->rx_length + length);
    }
}

BSP_RS485_Status_e BSP_RS485_Init(BSP_RS485_PortId_t id,
                                  const BSP_RS485_Config_t *config,
                                  BSP_RS485_FrameCallback_t frame_callback,
                                  BSP_RS485_TimeoutCallback_t timeout_callback,
                                  void *context)
{
    if (id >= BSP_RS485_MAX_PORTS || config == NULL || config->uart == NULL ||
        config->uart->hdmarx == NULL || config->uart->hdmatx == NULL ||
        config->quiet_time_ms == 0U || config->response_timeout_ms == 0U ||
        frame_callback == NULL)
    {
        return BSP_RS485_INVALID_PARAM;
    }

    memset(&g_ports[id], 0, sizeof(g_ports[id]));
    BSP_RS485_Port_t *port = &g_ports[id];
    port->initialized = true;
    port->config = *config;
    port->frame_callback = frame_callback;
    port->timeout_callback = timeout_callback;
    port->context = context;
    ClearRxState(port, HAL_GetTick());

    BSP_UART_Register_Slot(config->uart, 0U, g_rx_dma[id][0], g_rx_dma[id][1],
                           BSP_RS485_MAX_FRAME_SIZE, port, BSP_RS485_UartRx);
    if (config->uart->RxState != HAL_UART_STATE_BUSY_RX)
        port->diagnostics.io_error_count++;

    /* A registration failure is recoverable: PollAll() retries DMA reception. */
    return BSP_RS485_OK;
}

BSP_RS485_Status_e BSP_RS485_Send(BSP_RS485_PortId_t id,
                                  const uint8_t *data,
                                  uint16_t length,
                                  uint32_t now_ms)
{
    if (!Port_IsValid(id) || data == NULL || length == 0U ||
        length > BSP_RS485_MAX_FRAME_SIZE)
    {
        return BSP_RS485_INVALID_PARAM;
    }

    BSP_RS485_Port_t *port = &g_ports[id];
    UART_HandleTypeDef *uart = port->config.uart;
    if (port->waiting_response || port->recovered_this_poll ||
        uart->gState != HAL_UART_STATE_READY || uart->RxState != HAL_UART_STATE_BUSY_RX ||
        port->dma_remaining != BSP_RS485_MAX_FRAME_SIZE ||
        now_ms - port->last_rx_tick < port->config.quiet_time_ms)
    {
        return BSP_RS485_BUSY;
    }

    memcpy(g_tx_dma[id], data, length);
    if (length < BSP_RS485_MAX_FRAME_SIZE)
        memset(g_tx_dma[id] + length, 0, BSP_RS485_MAX_FRAME_SIZE - length);
    Cache_CleanTx(g_tx_dma[id]);

    if (HAL_UART_Transmit_DMA(uart, g_tx_dma[id], length) != HAL_OK)
    {
        port->diagnostics.io_error_count++;
        return BSP_RS485_IO_ERROR;
    }

    port->waiting_response = true;
    port->request_tick = now_ms;
    port->diagnostics.tx_count++;
    port->diagnostics.last_tx_tick = now_ms;
    port->diagnostics.last_tx_length = length;
    memset(port->diagnostics.last_tx_frame, 0, sizeof(port->diagnostics.last_tx_frame));
    memcpy(port->diagnostics.last_tx_frame, g_tx_dma[id], length);
    return BSP_RS485_OK;
}

static void PollPort(BSP_RS485_PortId_t id, uint32_t now_ms)
{
    BSP_RS485_Port_t *port = &g_ports[id];
    if (!port->initialized) return;
    UART_HandleTypeDef *uart = port->config.uart;
    uint8_t frame[BSP_RS485_MAX_FRAME_SIZE];
    uint16_t length = 0U;
    uint32_t frame_tick = 0U;
    bool overflow = false;
    bool quiet;
    bool pending_before_deadline;

    port->recovered_this_poll = false;
    uint32_t primask = Critical_Enter();
    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(uart->hdmarx);
    if (remaining != port->dma_remaining)
    {
        port->dma_remaining = remaining;
        if (remaining != BSP_RS485_MAX_FRAME_SIZE) port->last_rx_tick = now_ms;
    }
    quiet = remaining == BSP_RS485_MAX_FRAME_SIZE &&
            now_ms - port->last_rx_tick >= port->config.quiet_time_ms;
    pending_before_deadline = (port->rx_length != 0U || port->rx_overflow) &&
        port->frame_tick - port->request_tick < port->config.response_timeout_ms;
    if (quiet && (port->rx_length != 0U || port->rx_overflow))
    {
        length = port->rx_length;
        overflow = port->rx_overflow;
        frame_tick = port->frame_tick;
        memcpy(frame, port->rx_frame, length);
        port->rx_length = 0U;
        port->rx_overflow = false;
    }
    Critical_Exit(primask);

    if (length != 0U || overflow)
    {
        port->diagnostics.rx_frame_count++;
        port->diagnostics.last_rx_tick = frame_tick;
        port->diagnostics.last_rx_length = length;
        port->diagnostics.last_rx_overflow = overflow;
        memset(port->diagnostics.last_rx_frame, 0, sizeof(port->diagnostics.last_rx_frame));
        memcpy(port->diagnostics.last_rx_frame, frame, length);

        if (port->waiting_response &&
            frame_tick - port->request_tick < port->config.response_timeout_ms &&
            port->frame_callback(id, frame, length, overflow, frame_tick, port->context))
        {
            port->waiting_response = false;
        }
    }

    if (port->waiting_response && now_ms - port->request_tick >= port->config.response_timeout_ms)
    {
        /* A frame that arrived before the deadline may still need its quiet time. */
        if (pending_before_deadline && !quiet &&
            now_ms - port->request_tick <
                port->config.response_timeout_ms + port->config.quiet_time_ms)
        {
            return;
        }

        bool tx_stalled = uart->gState != HAL_UART_STATE_READY;
        port->waiting_response = false;
        port->diagnostics.timeout_count++;
        if (tx_stalled) HAL_UART_AbortTransmit(uart);
        (void)RestartRx(id, now_ms);
        if (port->timeout_callback != NULL) port->timeout_callback(id, port->context);
        return;
    }

    /* Recover a stopped receiver before the next request, then wait one Poll cycle. */
    if (!port->waiting_response && uart->RxState != HAL_UART_STATE_BUSY_RX)
    {
        (void)RestartRx(id, now_ms);
        port->recovered_this_poll = true;
    }
}

void BSP_RS485_PollAll(uint32_t now_ms)
{
    for (BSP_RS485_PortId_t id = 0U; id < BSP_RS485_MAX_PORTS; ++id)
        PollPort(id, now_ms);
}

BSP_RS485_Status_e BSP_RS485_Reset(BSP_RS485_PortId_t id)
{
    if (!Port_IsValid(id)) return BSP_RS485_INVALID_PARAM;

    BSP_RS485_Port_t *port = &g_ports[id];
    if (port->config.uart->gState != HAL_UART_STATE_READY)
        HAL_UART_AbortTransmit(port->config.uart);
    port->waiting_response = false;
    return RestartRx(id, HAL_GetTick());
}

bool BSP_RS485_IsWaitingResponse(BSP_RS485_PortId_t id)
{
    return Port_IsValid(id) && g_ports[id].waiting_response;
}

bool BSP_RS485_GetDiagnostics(BSP_RS485_PortId_t id,
                              BSP_RS485_Diagnostics_t *out)
{
    if (!Port_IsValid(id) || out == NULL) return false;
    *out = g_ports[id].diagnostics;
    return true;
}
