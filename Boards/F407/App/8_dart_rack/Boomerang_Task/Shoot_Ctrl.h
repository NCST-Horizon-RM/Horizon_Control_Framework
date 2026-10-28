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

//拉簧电机状态
typedef enum {
    SHOOT_MOVING = 0,
    APPROVE_FIRE
} Shoot_State_e;

typedef struct
{
    Shoot_State_e shoot_state;
    PID_t Spring_S;
    float spring_target_speed;
    uint8_t close_pwm;              //不发射时舵机PWM
    uint8_t fire_pwm;               //发射时扣下扳机的舵机PWM
    uint8_t pin_switch_down;        //判断是否成功储能
    uint8_t pin_switch_up;          //判断是否成功上膛
} Shoot_Ctrl_Block_t;

extern Shoot_Ctrl_Block_t shoot_ctrl;

void Spring_Move();
uint8_t Spring_Control_Init();
void Dart_Fire(void);
uint32_t  Angle_To_CCR(float angle);

#endif //HORIZON_CONTROL_FRAMEWORK_CONTROL_H
