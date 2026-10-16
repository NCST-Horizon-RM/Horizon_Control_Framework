//
// Created by lemon on 2026/10/11.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_CONTROL_H
#define HORIZON_CONTROL_FRAMEWORK_CONTROL_H

#include "Classic_Control.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "tim.h"
#include "Robot_Config.h"

typedef enum {
    SHOOT_MOVING = 0,
    APPROVE_FIRE
} Shoot_State_e;

typedef struct
{
    Shoot_State_e shoot_state;
    PID_t Spring_S;
    int32_t spring_target_speed;
    uint8_t close_pwm;
    uint8_t fire_pwm;
    uint8_t pin_switch_down;
    uint8_t pin_switch_up;
} Shoot_Ctrl_Block_t;

extern Shoot_Ctrl_Block_t shoot_ctrl;

void Dart_Fire(void);
uint32_t  Angle_To_CCR(float angle);

#endif //HORIZON_CONTROL_FRAMEWORK_CONTROL_H
