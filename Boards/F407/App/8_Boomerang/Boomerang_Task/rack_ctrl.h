//
// Created by lemon on 2026/10/16.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_RACK_CTRL_H
#define HORIZON_CONTROL_FRAMEWORK_RACK_CTRL_H

#include "Gimbal_Ctrl.h"
#include "Shoot_Ctrl.h"
#include "Yaw_Ctrl.h"

typedef enum {
    SHOOT_MOVING = 0,
    APPROVE_FIRE
} rack_State_e;

#endif //HORIZON_CONTROL_FRAMEWORK_RACK_CTRL_H
