//
// Created by CaoKangqi on 2026/6/20.
//

#ifndef F4_FRAMEWORK_CHASSIS_CTRL_H
#define F4_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "Chassis_Calc.h"
#include "IMU_Task.h"

typedef struct {
    PID_t Drive_S[4];
    PID_t Follow_Pos;       // 跟随外环：角度位置环
    PID_t Follow_Spd;       // 跟随内环：角速度速度环
    mecanumInit_typdef Mecanum;
} Chassis_Ctrl_Block_t;

uint8_t Chassis_Control_Init(void);
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, const IMU_Data_t *imu, float dt);

#endif //F4_FRAMEWORK_CHASSIS_CTRL_H
