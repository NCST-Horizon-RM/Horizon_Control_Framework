//
// Created by CaoKangqi on 2026/6/20.
//

#ifndef H7_FRAMEWORK_CHASSIS_CTRL_H
#define H7_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "Chassis_Calc.h"
#include "IMU_Task.h"

typedef struct {
    PID_t Steer_P[4];  // 舵轮 PID 控制器
    PID_t Steer_S[4];
    PID_t Drive_S[4];
    PID_t PID_Vx;
    PID_t PID_Vy;
    PID_t PID_Vw;

    Swerve_Feedback_t swerve_fb;   // 喂给解算器的输入结构体
    Swerve_Command_t  swerve_cmd;  // 解算器输出的输出结构体
} Chassis_Ctrl_Block_t;

uint8_t Chassis_Control_Init(void);
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, float dt);

#endif //H7_FRAMEWORK_CHASSIS_CTRL_H
