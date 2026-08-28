//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef F4_FRAMEWORK_ROBOT_CONFIG_H
#define F4_FRAMEWORK_ROBOT_CONFIG_H

#include "BSP_TIM.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "LK_Motor.h"
#include "VT13.h"
#include "DualBoard_Frame.h"


typedef struct __attribute__((aligned(4))){

} Chassis_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DM_MOTOR_DATA_Typedef DM4310_Yaw;
    DM_MOTOR_DATA_Typedef DM4310_Pitch;
} Gimbal_Motor_Group_t;

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_2006_bo;
    DJI_MOTOR_DATA_Typedef DJI_3508_R;
    DJI_MOTOR_DATA_Typedef DJI_3508_L;
} Shoot_Motor_Group_t;


extern Chassis_Motor_Group_t chassis_motors;
extern Gimbal_Motor_Group_t  gimbal_motors;
extern Shoot_Motor_Group_t   shoot_motors;

extern VT13_Typedef VT13;
extern C2G_t        C2G;

extern BSP_PWM_t imu_heater_pwm;


#endif //F4_FRAMEWORK_ROBOT_CONFIG_H
