#ifndef BALANCE_LEG_WHEEL_LQR_CONTROL_H
#define BALANCE_LEG_WHEEL_LQR_CONTROL_H

#include "VMC_Control.h"
#include "IMU_Task.h"
#include "Kalman_Observer.h"
#include "LESO_Control.h"

#define LQR_STATE_SIZE 10
#define LQR_OUTPUT_SIZE 4
#define LESO_COMPENSATION_SCALE 1.0f

typedef struct {
    float position_m;
    float velocity_mps;
    float yaw_rad;
    float yaw_rate_radps;
    float pitch_rad;
    float left_leg_theta_rad;
    float right_leg_theta_rad;
} LQR_Target_t;

typedef struct {
    float x[LQR_STATE_SIZE];
    float x_ref[LQR_STATE_SIZE];
    float K[LQR_OUTPUT_SIZE * LQR_STATE_SIZE];
    float u[LQR_OUTPUT_SIZE];
    float u_eq[LQR_OUTPUT_SIZE];
    LQR_Target_t target;
} LQR_Control_t;

void LQR_Init(LQR_Control_t *lqr);
void LQR_SetTarget(LQR_Control_t *lqr, float velocity_mps,
                   float yaw_rate_radps, float pitch_rad,
                   float left_leg_theta_rad, float right_leg_theta_rad,
                   float dt);
void LQR_Update(LQR_Control_t *lqr, const VMC_Control_t *vmc,
                const Kalman_Observer_t *observer,
                const IMU_Data_t *imu, LESO_Control_t *leso,
                bool leso_learning_enabled);

#endif
