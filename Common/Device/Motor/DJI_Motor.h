//
// Created by CaoKangqi on 2026/2/14.
//

#ifndef HORIZON_DJI_MOTOR_H
#define HORIZON_DJI_MOTOR_H

#include "BSP_CAN.h"
#include "Horizon_MATH.h"
#include "Offline_Detector.h"

typedef struct {
    Offline_Check_t offline;
    int16_t Angle_last;
    int16_t Angle_now;
    int32_t conEncode;
    int16_t Speed_last;
    int16_t Speed_now;
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int64_t Stuck_Time;
    int16_t Recovery_Count;
    uint16_t Stuck_Flag[2];
    int16_t Laps;
} DJI_MOTOR_DATA_Typedef;

void DJI_Motor_Dispatch(hcan_t *hcan, uint32_t FIFO_x);
void DJI_Motor_Resolve(void* instance, uint8_t* rx_data);
void DJI_Motor_Send(hcan_t* hcan, uint32_t stdid, int16_t n1, int16_t n2, int16_t n3, int16_t n4);

#endif //HORIZON_DJI_MOTOR_H