//
// Created by R0602 on 26-7-10.
//

#ifndef GIMBAL_CTRL_H
#define GIMBAL_CTRL_H
#include <stdint.h>
#include "Robot_Config.h"
#include "IMU_Task.h"
typedef struct {
    PID_t Pitch_P;
    PID_t Pitch_S;
    PID_t Yaw_P;
    PID_t Yaw_S;
} Gimbal_Ctrl_Block_t;
uint8_t Gimbal_Control_Init(void);
void Gimbal_Control_Task(const Gimbal_Motor_Group_t *g_motor,const IMU_Data_t *g_imu);
#endif //GIMBAL_CTRL_H
