//
// Created by lemon on 2026/10/16.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_RACK_CTRL_H
#define HORIZON_CONTROL_FRAMEWORK_RACK_CTRL_H

#include "Gimbal_Ctrl.h"
#include "Shoot_Ctrl.h"
#include "Yaw_Ctrl.h"

//镖架发射状态
typedef enum {
    FIRE_STATE1 = 0,
    FIRE_STATE2,
    FIRE_STATE3,
    FIRE_STATE3_HALF,
    FIRE_STATE4,
    STOP_FIRE
} Rack_State_e;

void Rack_Task();

#endif //HORIZON_CONTROL_FRAMEWORK_RACK_CTRL_H
