//
// Created by CaoKangqi on 2026/6/20.
//

#ifndef H7_FRAMEWORK_CHASSIS_CTRL_H
#define H7_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "Chassis_Kinematics.h"
#include "IMU_Task.h"
#include "VMC_Control.h"
#include "Kalman_Observer.h"
#include "LQR_Control.h"
#include "LESO_Control.h"
#include "Classic_Control.h"

typedef struct {
    VMC_Control_t vmc;
    Kalman_Observer_t odometry;
    float wheel_speed_mps;
    float body_position_m;
    float body_velocity_mps;
    LQR_Control_t lqr;
    LESO_Control_t leso;
    PID_t left_length_pos;
    PID_t left_length_vel;
    PID_t right_length_pos;
    PID_t right_length_vel;
    PID_t roll;
    PID_t joint_pos[4];
    PID_t joint_vel[4];
    float left_leg_force_n;
    float right_leg_force_n;
    bool stand_initialized;
} Chassis_Ctrl_Block_t;

typedef enum {
    CTRL_SAVE = 0,
    CTRL_STAND,
    CTRL_JUMP
} Chassis_Control_Mode_t;
uint8_t Chassis_Control_Init();
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, const Leg_Motor_Group_t *l_motor,const IMU_Data_t *imu, float dt);

#endif //H7_FRAMEWORK_CHASSIS_CTRL_H
