//
// Created by lemon on 2026/10/11.
//

#include "Gimbal.h"

// uint8_t serial_servo_ids[] = {1, 2};

Gimbal_Number gimbal_num = {
    .Servo_Encoder.act_time = 1000,
    .Servo_Encoder.servo_ids = {1, 2},
    .Servo_Encoder.close_pwm = 60,
    .Servo_Encoder.fire_pwm = 180,
    .Servo_Encoder.wait_encoder = {905, 460},
    .Servo_Encoder.put1_encoder = {400, 567},
    .Servo_Encoder.put2_encoder = {140, 700},
    .Servo_Encoder.dart3_adsorb_encoder = {100, 705},
    .Servo_Encoder.dart3_remove_encoder = {400, 550},
    .Servo_Encoder.dart4_adsorb_encoder = {120, 700},
    .Servo_Encoder.dart4_remove_encoder = {400, 550},
    .Motor_Encoder.dart3_put_encoder = 3450,
    .Motor_Encoder.dart3_grab_encoder = 4092,
    .Motor_Encoder.dart4_put_encoder = 4500,
    .Motor_Encoder.dart4_grab_encoder = 2829,
};

void Gimbal_Grab(uint8_t dart_number)
{
    switch (dart_number)
    {
        case 3:
            Motor_Increment_Encoder(gimbal_num.Motor_Encoder.dart3_grab_encoder);
            Servo_Grab(dart_number);
            Motor_Increment_Encoder(gimbal_num.Motor_Encoder.dart3_put_encoder);
        break;

        case 4:
            Motor_Increment_Encoder(gimbal_num.Motor_Encoder.dart4_grab_encoder);
            Servo_Grab(dart_number);
            Motor_Increment_Encoder(gimbal_num.Motor_Encoder.dart4_put_encoder);
        break;
    }
}

void Motor_Increment_Encoder(float target_angle)
{
    while (1)
    {
        // 1. 计算目标值（带缓启动）
        gimbal_num.Motor_Encoder.increment_motor_encoder =
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
    ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.wait_encoder, gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    switch (dart_number)
    {
        case 3:
            ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.dart3_adsorb_encoder, gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
            ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.dart3_remove_encoder, gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
        break;

        case 4:
            ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.dart4_adsorb_encoder, gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
            ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.dart4_remove_encoder, gimbal_num.Servo_Encoder.act_time);
            osDelay(1000);
        break;
    }
    ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.wait_encoder, gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
}

void Servo_Put()
{
    ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.wait_encoder, gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.put1_encoder, gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.put2_encoder, gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_SET);//电磁铁下磁
    ServoMoveMulti(2, gimbal_num.Servo_Encoder.servo_ids, gimbal_num.Servo_Encoder.wait_encoder, gimbal_num.Servo_Encoder.act_time);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOC ,GPIO_PIN_6 ,GPIO_PIN_RESET);//电磁铁重新上磁
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