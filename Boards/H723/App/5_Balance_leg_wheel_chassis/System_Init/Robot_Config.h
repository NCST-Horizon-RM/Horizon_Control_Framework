//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef H7_FRAMEWORK_ROBOT_CONFIG_H
#define H7_FRAMEWORK_ROBOT_CONFIG_H

#include "BM_Motor.h"
#include "BSP_TIM.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "Referee.h"
#include "DBUS.h"
#include "DualBoard_Frame.h"
#include "VT13.h"
#include "Power_CAP.h"
#include "Power_Meter.h"

typedef struct __attribute__((aligned(4))){
    DJI_MOTOR_DATA_Typedef DJI_3508_Chassis[2];
} Chassis_Motor_Group_t;
typedef struct __attribute__((aligned(4))){
    BM_MOTOR_DATA_Typedef BM_P1010B_Leg[4];
} Leg_Motor_Group_t;
typedef struct __attribute__((aligned(4))){
    DM_MOTOR_DATA_Typedef DM4310_Yaw;
    DM_MOTOR_DATA_Typedef DM4310_Pitch;
} Gimbal_Motor_Group_t;

extern Chassis_Motor_Group_t chassis_motors;
extern Leg_Motor_Group_t     leg_motors;
extern Gimbal_Motor_Group_t  gimbal_motors;

extern Referee_Data_t Referee;
extern DBUS_Typedef   DBUS;
extern VT13_Typedef   VT13;
extern Cap_t          cap;
extern Power_Meter_t Meter;
extern G2C_t g2c;

extern BSP_PWM_t imu_heater_pwm;
extern BSP_PWM_t trigger_pwm;

void Robot_Config_Init(void);

#endif //H7_FRAMEWORK_ROBOT_CONFIG_H
