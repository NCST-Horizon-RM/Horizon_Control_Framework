//
// Created by qza on 2026/7/17.
//

#ifndef H7_FRAMEWORK_KALMAN_OBSERVER_H
#define H7_FRAMEWORK_KALMAN_OBSERVER_H

#include "kalman_filter.h"
#include "IMU_Task.h"
// #include "Chassis_Calc_Leg.h"

typedef  struct {
    float s;       // 融合后的底盘位移
    float dot_s;   // 融合后的底盘线速度
}Estimator_OutState_t;


typedef struct {
    KalmanFilter_t kf; // 嵌入卡尔曼滤波器实例
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

#endif //H7_FRAMEWORK_KALMAN_OBSERVER_H
