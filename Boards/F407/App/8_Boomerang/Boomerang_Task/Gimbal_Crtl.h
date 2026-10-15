//
// Created by lemon on 2026/10/11.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_ARM_H
#define HORIZON_CONTROL_FRAMEWORK_ARM_H

#include <stdint.h>
#include <math.h>

#include "Classic_Control.h"
#include "cmsis_os.h"
#include "Robot_Config.h"
#include "serial_servo.h"
#include "System_State.h"
#include "Robot_Cmd.h"

//飞镖机械臂各项参数
typedef struct
{
        //两个舵机参数
        struct
        {
            uint16_t act_time;
            uint8_t servo_ids[2];
            uint8_t close_pwm;
            uint8_t fire_pwm;
            uint16_t wait_encoder[2];
            uint16_t put1_encoder[2];
            uint16_t put2_encoder[2];
            uint16_t dart3_adsorb_encoder[2];
            uint16_t dart3_remove_encoder[2];
            uint16_t dart4_adsorb_encoder[2];
            uint16_t dart4_remove_encoder[2];
        }Servo_Encoder;
        //电机参数
        struct
        {
            struct
            {
                int16_t dart3_grab_encoder;
                int16_t dart3_put_encoder;
                int16_t dart4_grab_encoder;
                int16_t dart4_put_encoder;
                int16_t increment_motor_encoder;
            }Turn_Encoder;
            struct
            {
                int16_t yaw_target;
            }Yaw_Target;
        }Motor_Encoder;
}Gimbal_Number;

typedef struct {
    PID_t Turn_P;
    PID_t Turn_S;
    PID_t Yaw_P;
    PID_t Yaw_S;
    float real_yaw_target;
} Gimbal_Ctrl_Block_t;

void Motor_Increment_Encoder(float target_angle);
void Gimbal_Grab(uint8_t dart_number);
void Servo_Grab(uint8_t dart_number);
void Dart_put();
float Target_Ramp(float now, float target);
void Servo_Put();
uint32_t  Angle_To_CCR(float angle);
void Yaw_track();

#endif //HORIZON_CONTROL_FRAMEWORK_ARM_H
