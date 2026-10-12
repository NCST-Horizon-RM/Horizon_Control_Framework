//
// Created by lemon on 2026/10/11.
//

#include "arm.h"

// uint8_t serial_servo_ids[] = {1, 2};

Arm_Number arm_num = {
    .Servo_Encoder.act_time = 1000,
    .Servo_Encoder.servo_ids = {1, 2},
    .Servo_Encoder.wait_encoder = {905, 460},
    .Servo_Encoder.put1_encoder = {400, 567},
    .Servo_Encoder.put2_encoder = {140, 700},
    .Servo_Encoder.dart3_adsorb_encoder = {100, 705},
    .Servo_Encoder.dart3_remove_encoder = {400, 550},
    .Servo_Encoder.dart4_adsorb_encoder = {120, 700},
    .Servo_Encoder.dart4_remove_encoder = {400, 550},
    .Gimbal_Encoder.dart3_put_encoder = 3450,
    .Gimbal_Encoder.dart3_grab_encoder = 4092,
    .Gimbal_Encoder.dart4_put_encoder = 4500,
    .Gimbal_Encoder.dart4_grab_encoder = 2829,
};

void Arm_Grab(uint8_t dart_number)
{
    switch (dart_number)
    {
        case 3:
            Motor_Increment_Encoder(arm_num.Gimbal_Encoder.dart3_grab_encoder);
            Servo_Grab(dart_number);
            Motor_Increment_Encoder(arm_num.Gimbal_Encoder.dart3_put_encoder);
        break;

        case 4:
            Motor_Increment_Encoder(arm_num.Gimbal_Encoder.dart4_grab_encoder);
            Servo_Grab(dart_number);
            Motor_Increment_Encoder(arm_num.Gimbal_Encoder.dart4_put_encoder);
        break;
    }
}

void Motor_Increment_Encoder(float target_angle)
{
    while (1)
    {
        // 1. 计算目标值（带缓启动）
        arm_num.Gimbal_Encoder.increment_gimbal_encoder =
            Target_Ramp((float)boomerang_motors.DJI_6020_Turn.Angle_Infinite, target_angle);

        // 2. 延时，给RTOS调度
        osDelay(1);

        // 3. 判断是否到达目标
        if (fabsf(boomerang_motors.DJI_6020_Turn.Angle_Infinite - target_angle) < 25.0f)
        {
            break;
        }
    }
}

//逐渐增加电机目标值
float Target_Ramp(float now, float target)
{

    static float out = 0;
    static int init = 0;

    float step = 0.35f;

    if (!init)
    {
        out = now;
        init = 1;
    }

    if (out < target)
    {
        out += step;
        if (out > target) out = target;
    }
    else if (out > target)
    {
        out -= step;
        if (out < target) out = target;
    }

    return out;
}

void Servo_Grab(uint8_t dart_number)
{
    ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.wait_encoder, arm_num.Servo_Encoder.act_time);
    osDelay(1000);
    switch (dart_number)
    {
        case 3:
            ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.dart3_adsorb_encoder, arm_num.Servo_Encoder.act_time);
            osDelay(1000);
            ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.dart3_remove_encoder, arm_num.Servo_Encoder.act_time);
            osDelay(1000);
        break;

        case 4:
            ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.dart4_adsorb_encoder, arm_num.Servo_Encoder.act_time);
            osDelay(1000);
            ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.dart4_remove_encoder, arm_num.Servo_Encoder.act_time);
            osDelay(1000);
        break;
    }
    ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.wait_encoder, arm_num.Servo_Encoder.act_time);
    osDelay(1000);
}

void Servo_Put()
{
    ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.wait_encoder, arm_num.Servo_Encoder.act_time);
    osDelay(1000);
    ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.put1_encoder, arm_num.Servo_Encoder.act_time);
    osDelay(1000);
    ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.put2_encoder, arm_num.Servo_Encoder.act_time);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_SET);//电磁铁下磁
    ServoMoveMulti(2, arm_num.Servo_Encoder.servo_ids, arm_num.Servo_Encoder.wait_encoder, arm_num.Servo_Encoder.act_time);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);//电磁铁重新上磁
}