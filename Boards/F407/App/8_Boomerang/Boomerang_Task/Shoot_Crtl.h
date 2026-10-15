//
// Created by lemon on 2026/10/11.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_CONTROL_H
#define HORIZON_CONTROL_FRAMEWORK_CONTROL_H

#include "Classic_Control.h"
#include "Robot_Cmd.h"
#include "System_State.h"

typedef struct
{
    PID_t Spring_S;
} Shoot_Ctrl_Block_t;

#endif //HORIZON_CONTROL_FRAMEWORK_CONTROL_H
