//
// Created by CaoKangqi on 2026/6/24.
//

#ifndef HORIZON_MODEL_CONTROL_H
#define HORIZON_MODEL_CONTROL_H

#include <stdint.h>
#include <math.h>
#include "arm_math.h"
#include "kalman_filter.h" // 引入卡尔曼滤波头文件

typedef struct {
    // 物理结构参数 (连杆长度)
    float l1, l2, l3, l4, l5;
    // 正运动学输出状态
    float L0;       // 虚拟腿长 (m)
    float phi0;     // 虚拟腿角 (rad)
    // 雅可比矩阵及其逆矩阵
    float JRM[2][2];     // 雅可比矩阵
    float JRM_inv[2][2]; // 雅可比逆矩阵

} VMC_t;

void VMC_Init(VMC_t *vmc, float l1, float l2, float l3, float l4, float l5);
void VMC_Update_Kinematics(VMC_t *vmc, float theta_front, float theta_back);
void VMC_Inverse_Dynamics(VMC_t *vmc, float F0_target, float Tp_target, float *tau_front, float *tau_back);

typedef struct {
    // 增益调度多项式系数矩阵 [12个参数][三次多项式的4个系数(a,b,c,d)]
    float K_coeffs[12][4];
    // 当前周期插值计算出的 K 矩阵 [2行(Tp, Tw) x 6列状态]
    float K_matrix[2][6];
    // 状态向量 x = [theta, dtheta, s, dot_s, phi, dphi]^T
    float x_measure[6];
    float x_ref[6];
    // 输出控制量
    float Tp_out; // 期望虚拟髋关节力矩
    float Tw_out; // 期望驱动轮力矩

} LQR_t;

void LQR_Init(LQR_t *lqr, float coeffs[12][4]);
void LQR_Update_K_Matrix(LQR_t *lqr, float current_L0);
void LQR_Calculate(LQR_t *lqr, float *measure, float *ref);


typedef struct {
    float s;       // 融合后的底盘位移
    float dot_s;   // 融合后的底盘线速度
    KalmanFilter_t kf; // 嵌入卡尔曼滤波器实例
    float last_wheel_speed;
} Estimator_t;

void Estimator_Init(Estimator_t *est);
void Estimator_Update(Estimator_t *est, float wheel_rpm, float imu_accel_x, float dt);

#endif //HORIZON_MODEL_CONTROL_H