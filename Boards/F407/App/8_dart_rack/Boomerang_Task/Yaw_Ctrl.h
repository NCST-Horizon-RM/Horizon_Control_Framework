//
// Created by lemon on 2026/10/16.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_YAW_CTRL_H
#define HORIZON_CONTROL_FRAMEWORK_YAW_CTRL_H
#include "Classic_Control.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "Robot_Config.h"

typedef struct
{
    PID_t Yaw_P;
    PID_t Yaw_S;
    float yaw_target;
} Yaw_Ctrl_Block_t;

uint8_t Yaw_Init();
void Yaw_track();

#endif //HORIZON_CONTROL_FRAMEWORK_YAW_CTRL_H
