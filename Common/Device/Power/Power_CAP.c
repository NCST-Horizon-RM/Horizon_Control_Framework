#include "Power_CAP.h"
#include <string.h>

int open_cap_flag = 0;

/**
 * @brief 电容接收数据解算
 */
void Power_Cap_Rx(void *instance, uint8_t *rx_buf)
{
    if (instance == NULL || rx_buf == NULL) return;

    Cap_t *cap_ptr = (Cap_t *)instance;

    cap_ptr->get.offline.last_feed_tick = HAL_GetTick();
    cap_ptr->get.cap_key   = rx_buf[0];
    cap_ptr->get.cap_state = rx_buf[1];

    uint16_t raw_voltage = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
    uint16_t raw_power   = ((uint16_t)rx_buf[4] << 8) | rx_buf[5];
    uint16_t raw_capacityt = ((uint16_t)rx_buf[6] << 8) | rx_buf[7];

    cap_ptr->get.bat_voltage = (float)raw_voltage / 100.0f;
    cap_ptr->get.nowPower    = (float)raw_power / 100.0f;
    cap_ptr->get.Cap_Capacity = (float)raw_capacityt / 100.0f;
}

/**
 * @brief 电容控制数据发送 (底层硬件接口)
 * @param hcan        CAN句柄
 * @param can_id      发送的CAN ID
 * @param tx_data_ptr 指向准备好数据的 CapSetData_t 结构体指针
 */
void Power_Cap_Tx(hcan_t *hcan, uint16_t can_id, CapSetData_t *tx_data_ptr)
{
    if (tx_data_ptr == NULL) return;

    tx_data_ptr->Control.check_code = 0xAA;

    CAN_Send_Msg(hcan, can_id, tx_data_ptr->raw_data, 8);
}