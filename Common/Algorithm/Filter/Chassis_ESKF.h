//
// Created by qza on 2026/9/8.
//

#ifndef HORIZON_CHASSIS_ESKF_H
#define HORIZON_CHASSIS_ESKF_H

#include <stdint.h>
#include "All_define.h"

typedef struct
{
    float dt;               // 采样周期 (s)

    /* 轮速正解 / 轮式里程计 */
    float wheel_vx;         // 底盘 x 速度 (m/s)
    float wheel_vy;         // 底盘 y 速度 (m/s)
    float wheel_vw;         // 底盘 z 角速度 (rad/s)
    uint8_t wheel_valid;    // 轮速是否有效

    /* IMU 原始数据 */
    float imu_gx;           // rad/s
    float imu_gy;           // rad/s
    float imu_gz;           // rad/s
    float imu_ax;           // m/s^2
    float imu_ay;           // m/s^2
    float imu_az;           // m/s^2

    /* 姿态输入，用于重力补偿；单位：rad */
    float roll;
    float pitch;
    uint8_t attitude_valid;
} Chassis_ESKF_Input_t;

typedef struct
{
    float vx;               // 融合后的底盘 x 速度 (m/s)
    float vy;               // 融合后的底盘 y 速度 (m/s)
    float vw;               // 融合后的底盘 z 角速度 (rad/s)

    float bax;              // 估计的 x 轴加速度零偏
    float bay;              // 估计的 y 轴加速度零偏
    float bgz;              // 估计的 z 轴陀螺零偏

    float slip_score;       // 0~1，越大表示越可能打滑
    float confidence;       // 0~1，越大表示越可信

    float vx_var;           // vx 方差估计
    float vy_var;           // vy 方差估计
    float vw_var;           // vw 方差估计
} Chassis_ESKF_Output_t;

typedef struct
{
    float x[5];             // [vx, vy, bax, bay, bgz]
    float P[25];            // 5x5 协方差矩阵

    float gravity;          // 重力加速度
    float sigma_acc;        // 过程噪声：线加速度
    float sigma_bias_acc;   // 过程噪声：加速度零偏随机游走
    float sigma_bias_gyr;   // 过程噪声：陀螺零偏随机游走
    float wheel_v_meas_std; // 轮速平移观测标准差
    float wheel_w_meas_std; // 轮速角速度观测标准差
    float slip_gain;        // 打滑时的观测降权倍数
    float slip_threshold_v; // 平移残差阈值
    float slip_threshold_w; // 转动残差阈值
    float static_gyro_th;   // 静止判据：陀螺阈值
    float static_wheel_v_th;// 静止判据：轮速阈值
    float static_wheel_w_th;// 静止判据：角速度阈值

    float slip_score;       // 低通后的打滑评分
    uint8_t initialized;
} Chassis_ESKF_t;

void Chassis_ESKF_Init(Chassis_ESKF_t *f);
void Chassis_ESKF_Reset(Chassis_ESKF_t *f);
void Chassis_ESKF_Update(Chassis_ESKF_t *f,
                         const Chassis_ESKF_Input_t *in,
                         Chassis_ESKF_Output_t *out);

#endif //HORIZON_CHASSIS_ESKF_H
