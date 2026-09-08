/**
 * @file BSP_RS485.h
 * @brief H723 RS485 bus transport built on BSP_UART and DMA.
 *
 * This module owns UART/DMA state, receive buffering, RTU quiet-time framing
 * and response timeouts.  It deliberately does not contain any Modbus logic.
 */

#ifndef HORIZON_BSP_RS485_H
#define HORIZON_BSP_RS485_H

#include "BSP_UART.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_RS485_MAX_PORTS       2U
#define BSP_RS485_MAX_FRAME_SIZE  32U

typedef uint8_t BSP_RS485_PortId_t;

typedef enum {
    BSP_RS485_OK = 0,
    BSP_RS485_BUSY,
    BSP_RS485_IO_ERROR,
    BSP_RS485_INVALID_PARAM
} BSP_RS485_Status_e;

typedef struct {
    UART_HandleTypeDef *uart;
    uint32_t quiet_time_ms;
    uint32_t response_timeout_ms;
} BSP_RS485_Config_t;

/**
 * Called from BSP_RS485_PollAll(), never from the UART interrupt.
 * Return true only when this frame completes the current request.
 */
typedef bool (*BSP_RS485_FrameCallback_t)(BSP_RS485_PortId_t port,
                                          const uint8_t *data,
                                          uint16_t length,
                                          bool overflow,
                                          uint32_t timestamp_ms,
                                          void *context);

/** Called from BSP_RS485_PollAll() after timeout recovery has started. */
typedef void (*BSP_RS485_TimeoutCallback_t)(BSP_RS485_PortId_t port,
                                            void *context);

typedef struct {
    uint32_t tx_count;
    uint32_t rx_frame_count;
    uint32_t timeout_count;
    uint32_t io_error_count;
    uint32_t last_tx_tick;
    uint32_t last_rx_tick;
    bool last_rx_overflow;
    uint16_t last_tx_length;
    uint16_t last_rx_length;
    uint8_t last_tx_frame[BSP_RS485_MAX_FRAME_SIZE];
    uint8_t last_rx_frame[BSP_RS485_MAX_FRAME_SIZE];
} BSP_RS485_Diagnostics_t;

/**
 * Register one physical RS485 UART. Reinitialising the same port replaces its
 * callbacks and buffers. Assign each physical UART to only one port.
 */
BSP_RS485_Status_e BSP_RS485_Init(BSP_RS485_PortId_t port,
                                  const BSP_RS485_Config_t *config,
                                  BSP_RS485_FrameCallback_t frame_callback,
                                  BSP_RS485_TimeoutCallback_t timeout_callback,
                                  void *context);

/**
 * Starts one half-duplex request.  The source data is copied before DMA starts,
 * so the caller may reuse its buffer when this function returns.
 */
BSP_RS485_Status_e BSP_RS485_Send(BSP_RS485_PortId_t port,
                                  const uint8_t *data,
                                  uint16_t length,
                                  uint32_t now_ms);

/** Drive framing, receive recovery and request timeout from task context. */
void BSP_RS485_PollAll(uint32_t now_ms);

/** Abort a pending request, discard a partial frame and restart DMA reception. */
BSP_RS485_Status_e BSP_RS485_Reset(BSP_RS485_PortId_t port);

bool BSP_RS485_IsWaitingResponse(BSP_RS485_PortId_t port);
bool BSP_RS485_GetDiagnostics(BSP_RS485_PortId_t port,
                              BSP_RS485_Diagnostics_t *out);

#ifdef __cplusplus
}
#endif

#endif /* HORIZON_BSP_RS485_H */
