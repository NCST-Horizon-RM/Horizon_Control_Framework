//
// Created by lemon on 2026/10/11.
//

#include "Shoot_Ctrl.h"

static void Auto_Move();

Shoot_Ctrl_Block_t shoot_ctrl ={
    .close_pwm = 60,
    .fire_pwm = 180,
};

uint8_t Spring_Control_Init()
{
    float PID_SPRING_S[3] = {15.0f, 0.7f, 0.0f};
    PID_Init(&shoot_ctrl.Spring_S, 50.0f, 30.0f, PID_SPRING_S,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //向系统下发发射当前状态，准备中
    System_State_Report(ID_SHOOT, STATUS_PREPARING);
    return 1;
}

void Spring_move()
{
    // 状态汇报
    if (!Is_Group_Online(SHOOT)) {
        System_State_Report(ID_SHOOT, STATUS_LOST);
    }
    else{System_State_Report(ID_SHOOT, STATUS_RUN);}
    // 判断系统状态
    bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             sys_state.global_mode == GLOBAL_STANDBY ||
                             sys_state.global_mode == GLOBAL_INIT_STAGE);
    if (shoot_cmd.mode == SHOOT_CMD_SAFE || is_system_locked)
    {
        PID_Clear(&shoot_ctrl.Spring_S);
    }else
    {
        if (shoot_cmd.mode == SHOOT_CMD_AUTO)
        {
            Auto_Move();
        }else if (shoot_cmd.mode == SHOOT_CMD_REMOTE)
        {
            shoot_ctrl.spring_target_speed = shoot_cmd.target_spring;
        }
        PID_Calculate(&shoot_ctrl.Spring_S,dart_rack_motors.DJI_3508_SPRING.Speed_now,shoot_ctrl.spring_target_speed);
    }
}

static void Auto_Move()
{
    shoot_ctrl.pin_switch_down = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
    shoot_ctrl.pin_switch_up = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14);
    switch(shoot_ctrl.pin_switch_down)
    {
    case 1:
        shoot_ctrl.shoot_state = SHOOT_MOVING;
        shoot_ctrl.spring_target_speed =-2500;
        break;
    case 0:
        switch(shoot_ctrl.pin_switch_up)
        {
        case 1:
            shoot_ctrl.shoot_state = SHOOT_MOVING;
            shoot_ctrl.spring_target_speed =7000;
            break;
        case 0:
            shoot_ctrl.spring_target_speed =0;
            shoot_ctrl.shoot_state = APPROVE_FIRE;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void Dart_Fire(void)
{
    // 打开扳机
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(shoot_ctrl.fire_pwm));
    osDelay(500);

    // 关闭扳机（完成一次发射）
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Angle_To_CCR(shoot_ctrl.close_pwm));
    osDelay(500);
}

uint32_t  Angle_To_CCR(float angle)
{
    // 高电平时间范围：0.5ms  2.5ms
    float pulse_min = 0.5;  // 0° 对应 0.5ms
    float pulse_max = 2.5;  // 180° 对应 2.5ms
    float pulse_width = pulse_min + (angle / 180.0) * (pulse_max - pulse_min);

    // 计算 CCR 值：CCR = (pulse_width / 20ms) * (ARR + 1)
    return (uint32_t)((pulse_width / 20.0) * 20000); // ARR+1 = 19999+1=20000
}