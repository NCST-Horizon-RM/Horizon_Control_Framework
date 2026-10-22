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

#define GIMBAL_MOTOR_MID 3435

//飞镖机械臂各项参数
typedef struct
{
        //两个舵机参数
        struct
        {
            uint16_t act_time;
            uint8_t servo_ids[2];
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
                float dart3_grab_encoder;
                float dart3_put_encoder;
                float dart4_grab_encoder;
                float dart4_put_encoder;
                float increment_motor_encoder;
        }Motor_Encoder;
}Gimbal_Number;

typedef struct {
    PID_t Turn_P;
    PID_t Turn_S;
    Gimbal_Number gimbal_num;
} Gimbal_Ctrl_Block_t;

uint8_t Gimbal_Init();
void Arm_Motor_Increment_Encoder(float target_angle);
void Arm_Grab(uint8_t dart_number);
void Arm_Servo_Grab(uint8_t dart_number);
void Dart_put();
void Arm_Servo_Put();
void Yaw_track();

#endif //HORIZON_CONTROL_FRAMEWORK_ARM_H
