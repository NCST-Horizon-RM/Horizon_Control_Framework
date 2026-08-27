//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef H7_FRAMEWORK_ROBOT_CONFIG_H
#define H7_FRAMEWORK_ROBOT_CONFIG_H

#include "BSP_TIM.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "LK_Motor.h"

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_3508_Chassis[4];
    DJI_MOTOR_DATA_Typedef DJI_6020_Steer[4];
} Chassis_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_3508_Yaw;
    DM_MOTOR_DATA_Typedef DM4310_Pitch;
    DM_MOTOR_DATA_Typedef DM4310_Yaw;
} Gimbal_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_3508_Shoot_L;
    DJI_MOTOR_DATA_Typedef DJI_3508_Shoot_R;
    DJI_MOTOR_DATA_Typedef DJI_3508_Shoot_M;
    DM_MOTOR_DATA_Typedef DM4310_Feed;
    DJI_MOTOR_DATA_Typedef DJI_3508_Pull;
    DJI_MOTOR_DATA_Typedef DJI_3508_Travel;
    DJI_MOTOR_DATA_Typedef DJI_2006_bo;
} Shoot_Motor_Group_t;

extern Chassis_Motor_Group_t chassis_motors;
extern Gimbal_Motor_Group_t  gimbal_motors;
extern Shoot_Motor_Group_t   shoot_motors;

extern BSP_PWM_t imu_heater_pwm;
extern BSP_PWM_t trigger_pwm;

void Robot_Config_Init(void);

#endif //H7_FRAMEWORK_ROBOT_CONFIG_H
