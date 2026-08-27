//
// Created by CaoKangqi on 2026/2/14.
//
#include "DJI_Motor.h"
#include "All_define.h"


/**
 * @brief DJI 电机协议解析内核 (3508/2006/6020 通用)
 */
void DJI_Motor_Resolve(void* instance, uint8_t* rx_data) {
    DJI_MOTOR_DATA_Typedef* DATA = instance;

    DATA->offline.last_feed_tick = HAL_GetTick();
    DATA->Angle_last = DATA->Angle_now;
    DATA->Angle_now  = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    DATA->Speed_last = DATA->Speed_now;
    int16_t spd_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    DATA->Speed_now  = OneFilter1(spd_raw,DATA->Speed_last, 25000);
    DATA->current    = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    DATA->temperature = rx_data[6]; // 6020/3508有温度，2006不看即可

    // 统一处理越界/圈数逻辑
    int16_t diff = DATA->Angle_now - DATA->Angle_last;
    if      (diff < -4000) DATA->Laps++;
    else if (diff >  4000) DATA->Laps--;

    // 圈数异常保护
    if (DATA->Laps > 32500 || DATA->Laps < -32500) {
        DATA->Laps = 0;
    }

    DATA->Angle_Infinite = (int32_t)((DATA->Laps << 13) + DATA->Angle_now);
}

/**
 * @brief 通用发送函数
 * @param hcan
 */
void DJI_Motor_Send(hcan_t* hcan, uint32_t stdid, int16_t n1, int16_t n2, int16_t n3, int16_t n4) {
    uint8_t data[8];
    data[0] = n1 >> 8; data[1] = n1;
    data[2] = n2 >> 8; data[3] = n2;
    data[4] = n3 >> 8; data[5] = n3;
    data[6] = n4 >> 8; data[7] = n4;
    if (CAN_GetTxFreeLevel(hcan) > 0) {
        CAN_Send_Msg(hcan, stdid, data, 8);
    }
}