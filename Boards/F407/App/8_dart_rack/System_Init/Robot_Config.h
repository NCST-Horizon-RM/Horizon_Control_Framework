//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef F4_FRAMEWORK_ROBOT_CONFIG_H
#define F4_FRAMEWORK_ROBOT_CONFIG_H

#include "BSP_TIM.h"
#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "LK_Motor.h"
#include "Referee.h"
#include "DBUS.h"
#include "VT13.h"
#include "Power_CAP.h"
#include "DualBoard_Frame.h"

typedef struct __attribute__((aligned(4))){
        DJI_MOTOR_DATA_Typedef DJI_3508_SPRING;
        DJI_MOTOR_DATA_Typedef DJI_2006_Yaw;
        DJI_MOTOR_DATA_Typedef DJI_6020_Turn;
} Dart_Rack_Motor_Group_t;

extern Referee_Data_t Referee;
extern DBUS_Typedef   DBUS;
extern VT13_Typedef   VT13;
extern Cap_t          cap;
extern G2C_t          g2c;
extern Dart_Rack_Motor_Group_t dart_rack_motors;

extern BSP_PWM_t imu_heater_pwm;

#endif //F4_FRAMEWORK_ROBOT_CONFIG_H
