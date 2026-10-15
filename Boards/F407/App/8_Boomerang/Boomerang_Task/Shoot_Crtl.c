//
// Created by lemon on 2026/10/11.
//

#include "Shoot_Crtl.h"



static Shoot_Ctrl_Block_t shoot_ctrl;

uint8_t Rack_Control_Init()
{
    float PID_SPRING_S[3] = {15.0f, 0.7f, 0.0f};
    PID_Init(&shoot_ctrl.Spring_S, 50.0f, 30.0f, PID_SPRING_S,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //向系统下发发射当前状态，准备中
    System_State_Report(ID_SHOOT, STATUS_PREPARING);
    return 1;
}