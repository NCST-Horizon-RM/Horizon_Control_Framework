//
// Created by CaoKangqi on 2026/3/15.
//

#ifndef HORIZON_LK_MOTOR_H
#define HORIZON_LK_MOTOR_H

#include "BSP_CAN.h"
#include "Offline_Detector.h"


typedef struct
{
    Offline_Check_t offline;
    uint8_t  temp;          // 电机温度
    uint16_t Current;       // 电流

    int16_t  rawSpeed;
    int16_t  lastRawSpeed;

    uint16_t rawEncode;
    uint16_t lastRawEncode;

    int32_t round;

    float conEncode;
    float lastConEncode;

    uint8_t State;

}LK_MOTOR_DATA_Typedef;

void LK_Motor_Resolve(void *instance, uint8_t *RxMessage);
void LK_Motor_Iq_Send(hcan_t* hcan, uint16_t motor_id, int16_t iq);
void LK_Motor_Data_Read(hcan_t* hcan, uint16_t motor_id);

#endif //HORIZON_LK_MOTOR_H