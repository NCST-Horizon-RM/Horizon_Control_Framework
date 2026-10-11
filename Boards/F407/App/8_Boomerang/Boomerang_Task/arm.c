//
// Created by lemon on 2026/10/11.
//

#include "arm.h"

uint8_t serial_servo_ids[] = {1, 2};

Grab_Encoder gb_encoder = {
    .wait_encoder = {905, 460},
    .put1_encoder = {400, 567},
    .put2_encoder = {140, 700},
    .dart3_adsorb_encoder = {100, 705},
    .dart3_remove_encoder = {400, 550},
    .dart4_adsorb_encoder = {120, 700},
    .dart4_remove_encoder = {400, 550},
};



Middle_Encoder mid_encoding = {
    .dart3 = 3450,
    .dart4 = 3404
};




void Motor_TurnTo_Angle(float target_angle)
{
    // while (1)
    // {
    //     // 1. 计算目标值（带缓启动）
    //     ALL_MOTOR.DJI_6020_turn.DATA.Aim =
    //         target_ramp((float)ALL_MOTOR.DJI_6020_turn.DATA.Angle_Infinite, target_angle);
    //
    //     // 2. 延时，给RTOS调度
    //     osDelay(1);
    //
    //     // 3. 判断是否到达目标
    //     if (fabsf(ALL_MOTOR.DJI_6020_turn.DATA.Angle_Infinite - target_angle) < 25.0f)
    //     {
    //         break;
    //     }
    // }
}


