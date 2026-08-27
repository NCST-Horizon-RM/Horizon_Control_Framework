//
// Created by CaoKangqi on 2026/6/22.
//
#include "Comm_DualBoard.h"
#include <string.h>

typedef struct {
    uint8_t  buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t total_len;
    uint16_t offset;
    uint8_t  seq;
    bool     is_sending;
} CAN_TP_Tx_State_t;

typedef struct {
    uint8_t  buf[DUALBOARD_MAX_PAYLOAD];
    uint16_t current_len;
    uint8_t  next_seq;
    bool     is_active;
} CAN_TP_Rx_t;

static CAN_TP_Tx_State_t can_tx = {0};
static CAN_TP_Rx_t       can_rx = {0};

// UART 接收双缓冲
uint8_t DualBoard_UART_Rx_Buf0[DUALBOARD_MAX_PAYLOAD];
uint8_t DualBoard_UART_Rx_Buf1[DUALBOARD_MAX_PAYLOAD];

static DualBoard_Rx_Callback_t App_Rx_Callback = NULL;
static DualBoard_Config_t      Comm_Config     = {0};

/**
 * @brief 初始化双板通讯
 */
void DualBoard_Comm_Init(DualBoard_Rx_Callback_t rx_cb, DualBoard_Config_t *config)
{
    if (config == NULL) return;

    App_Rx_Callback = rx_cb;
    memcpy(&Comm_Config, config, sizeof(DualBoard_Config_t));
}

/**
 * @brief 发送数据接口
 */
uint8_t DualBoard_Send(Comm_Link_e link, void *data_ptr, uint16_t len)
{
    if (data_ptr == NULL || len > DUALBOARD_MAX_PAYLOAD || len == 0) return 1;

    if (link == LINK_UART) {
        if (Comm_Config.uart_handle == NULL) return 3;

        if (HAL_UART_Transmit_DMA(Comm_Config.uart_handle, (uint8_t*)data_ptr, len) != HAL_OK) {
            return 2;
        }
        return 0;
    }

    if (link == LINK_CAN) {
        if (Comm_Config.can_handle == NULL) return 3;

        if (can_tx.is_sending) return 2;
        can_tx.total_len = len;
        memcpy(can_tx.buf, data_ptr, len);
        can_tx.offset = 0;
        can_tx.seq = 0;
        can_tx.is_sending = true;
        return 0;
    }
    return 1;
}

/**
 * @brief 轮询任务，用于 CAN 拆包发送
 */
void DualBoard_Task_Poll(void)
{
    if (!can_tx.is_sending) return;

    while (can_tx.is_sending) {
        uint8_t tx_buf[8];
        uint16_t remain = can_tx.total_len - can_tx.offset;
        uint8_t chunk_size = (remain > 7) ? 7 : (uint8_t)remain;
        uint8_t is_last = (remain <= 7) ? 1 : 0;

        tx_buf[0] = (is_last << 7) | (can_tx.seq & 0x7F);
        memcpy(&tx_buf[1], &can_tx.buf[can_tx.offset], chunk_size);
        if (chunk_size < 7) memset(&tx_buf[1 + chunk_size], 0, 7 - chunk_size);

        if (CAN_Send_Msg(Comm_Config.can_handle, Comm_Config.tx_id, tx_buf, 8) == 0) {
            can_tx.offset += chunk_size;
            can_tx.seq++;

            if (is_last) {
                can_tx.is_sending = false;
                break;
            }
        } else {
            // 如果底邮箱满或者发送失败，跳出当前循环等待下一次 Poll 周期重试
            break;
        }
    }
}

/**
 * @brief CAN 接收回调
 */
void DualBoard_CAN_Rx(void *device_ptr, uint8_t *data)
{
    if (data == NULL) return;
    (void)device_ptr;

    uint8_t ctrl = data[0];
    uint8_t is_last = (ctrl >> 7) & 0x01;
    uint8_t seq = ctrl & 0x7F;
    uint8_t payload_len = 7;

    if (seq == 0) {
        can_rx.is_active = true;
        can_rx.current_len = 0;
        can_rx.next_seq = 0;
    }

    if (can_rx.is_active && seq == can_rx.next_seq) {
        if (can_rx.current_len + payload_len <= sizeof(can_rx.buf)) {
            memcpy(&can_rx.buf[can_rx.current_len], &data[1], payload_len);
            can_rx.current_len += payload_len;
            can_rx.next_seq++;

            if (is_last) {
                if (App_Rx_Callback != NULL) {
                    App_Rx_Callback(LINK_CAN, can_rx.buf, can_rx.current_len);
                }
                can_rx.is_active = false;
            }
        } else {
            can_rx.is_active = false;
        }
    }
}

/**
 * @brief UART 接收回调
 */
void DualBoard_UART_Rx(uint8_t *pData, void *device_ptr, uint16_t Size)
{
    if (pData == NULL || Size == 0) return;
    (void)device_ptr;

    if (App_Rx_Callback != NULL) {
        App_Rx_Callback(LINK_UART, pData, Size);
    }
}