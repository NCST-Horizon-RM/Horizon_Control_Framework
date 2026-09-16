//
// Created by qza on 2026/7/17.
//

#ifndef H7_FRAMEWORK_KALMAN_OBSERVER_H
#define H7_FRAMEWORK_KALMAN_OBSERVER_H

#include "IMU_Task.h"

typedef  struct {
    float s;       // 融合后的底盘位移
    float dot_s;   // 融合后的底盘线速度
}Estimator_OutState_t;


typedef struct {
    /* 固定三状态里程计: [位移, 速度, x 轴加速度零偏] */
    float state[3];
    float covariance[9];
    float accel_noise;
    float accel_bias_noise;
    float wheel_speed_noise;
    float slip_threshold;
    float slip_gain;
    float slip_score;
    unsigned char initialized;
    float last_wheel_distance;
    float last_wheel_speed;
    float wheel_speed_input;
    Estimator_OutState_t outstate;
} Kalman_Observer_t;

void Estimator_Leg_Init(Kalman_Observer_t *est);
void Estimator_Leg_Update(Kalman_Observer_t *est, float wheel_speed,
                          float imu_accel_x, float dt);
void Estimator_Task(Kalman_Observer_t *est, IMU_Data_t imu_data, float dt);
void Estimator_Set_WheelSpeed(Kalman_Observer_t *est, float wheel_speed);
void Estimator_QR_Change(Kalman_Observer_t *est,
                         const float Q_data[4], const float R_data[1]);

#endif //H7_FRAMEWORK_KALMAN_OBSERVER_H
