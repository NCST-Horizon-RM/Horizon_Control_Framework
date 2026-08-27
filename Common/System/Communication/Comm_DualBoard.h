//
// Created by CaoKangqi on 2026/6/22.
//

#ifndef HORIZON_COMM_DUALBOARD_H
#define HORIZON_COMM_DUALBOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "BSP_UART.h"
#include "BSP_CAN.h"

typedef enum {
    LINK_CAN,
    LINK_UART
} Comm_Link_e;

#define DUALBOARD_MAX_PAYLOAD 256

// 暴露给外部 UART_RX_NODE 注册使用的双缓冲区
extern uint8_t DualBoard_UART_Rx_Buf0[DUALBOARD_MAX_PAYLOAD];
extern uint8_t DualBoard_UART_Rx_Buf1[DUALBOARD_MAX_PAYLOAD];

typedef void (*DualBoard_Rx_Callback_t)(Comm_Link_e link, uint8_t *data, uint16_t len);

typedef struct {
    hcan_t* can_handle;  // 绑定的 CAN 句柄指针
    uint32_t             tx_id;       // 发送数据的 CAN ID
    UART_HandleTypeDef* uart_handle; // 绑定的 UART 句柄指针
} DualBoard_Config_t;

void DualBoard_Comm_Init(DualBoard_Rx_Callback_t rx_cb, DualBoard_Config_t *config);

uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len);
void DualBoard_Task_Poll(void);

/* --- 暴露给框架宏注册的底层接收回调 --- */
void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data);
void DualBoard_UART_Rx(uint8_t *pData, void *device_ptr, uint16_t Size);

#endif //HORIZON_COMM_DUALBOARD_H