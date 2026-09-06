//
// Created by CaoKangqi on 2026/6/20.
//

#ifndef H7_FRAMEWORK_CHASSIS_CTRL_H
#define H7_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "Chassis_Calc.h"
#include "Chassis_Kinematics.h"
#include "IMU_Task.h"

typedef struct {
    PID_t Steer_P[4];  // 舵轮 PID 控制器
    PID_t Steer_S[4];
    PID_t PID_Vx;
    PID_t PID_Vy;
    PID_t PID_Vw;
    Chassis_Cfg_t Chassis_Config;
    Chassis_Feedback_t Chassis_Feedback;
    Chassis_Command_t Chassis_Command;

} Chassis_Ctrl_Block_t;

uint8_t Chassis_Control_Init(void);
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, float dt);

#endif //H7_FRAMEWORK_CHASSIS_CTRL_H
