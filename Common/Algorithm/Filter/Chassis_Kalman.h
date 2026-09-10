//
// Created by qza on 2026/9/8.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_CHASSIS_KALMAN_H
#define HORIZON_CONTROL_FRAMEWORK_CHASSIS_KALMAN_H

#include <stdint.h>
#include "Chassis_Kinematics.h"
#include "kalman_filter.h"

/* =====================================================================
 * 底盘里程计 Kalman 融合
 * ---------------------------------------------------------------------
 * 坐标系沿用 Chassis_Kinematics.h：前 +X、左 +Y、绕 +Z 逆时针为正。
 *
 * 状态向量 x（下标见 Chassis_Kalman_StateIndex_e）：
 *   x = [ vx, vy, bax, bay, bgz ]^T
 *     vx/vy   : 车体系线速度 (m/s)
 *     bax/bay : IMU X / Y 加速度零偏 (m/s²)
 *     bgz     : IMU Z 轴陀螺零偏 (rad/s)
 *
 * 预测模型（含转动坐标耦合，角速度取轮式 vw）：
 *   vx' = vx + (ax + vw·vy - bax) * dt
 *   vy' = vy + (ay - vw·vx - bay) * dt
 *   bax' = bax, bay' = bay, bgz' = bgz
 *   vw·vy / -vw·vx 即转动坐标耦合（ω×v），转弯且带平移时才生效。
 * ax/ay 由 Chassis_Kalman_Input_t 传入，是已扣除重力分量的水平加速度。
 * IMU 安装偏心（杠杆臂）引起的向心/切向伪加速度不在本滤波器内补偿，
 * 需要时由 IMU_Task 的 IMU_Accel_LeverArm_Compensate 统一处理，避免重复。
 *
 * 预测是非线性的且在本文件内完成，故 SkipEq1 = 1，不使用通用滤波器的
 * B·u 通道，只借用 kalman_filter.c 的协方差传播与量测更新。
 *
 * 量测向量 z（下标见 Chassis_Kalman_MeasIndex_e）：
 *   z = [ wheel_vx, wheel_vy, wz_imu - wheel_vw ]^T
 *     轮式里程计 vx/vy 直接量测线速度，轮式 vw 与 IMU wz 之差量测 bgz。
 *
 * 变 Q / 变 R（唯一一套自适应机制）：
 *   - 残差 weight = |残差| / (|残差| + 特征尺度)，低通后落在 0~1：
 *       R = R0·[1 + (R_scale-1)·weight]，Q = Q0 / [1 + (Q_div-1)·weight]；
 *   - 低速加权：线速度越低越信任轮式 vx/vy，R 再乘一个小于 1 的系数；
 *     该系数乘 (1 - 残差权重)，残差变大（疑似打滑）时自动淡出；
 *   - 角速度通道默认不参与低速加权（wheel_w_low_r_scale = 1）。
 * ===================================================================== */

#define CHASSIS_KALMAN_STATE_NUM    5u
#define CHASSIS_KALMAN_MEAS_NUM     3u

/* 状态下标 */
typedef enum {
    CK_STATE_VX  = 0,
    CK_STATE_VY,
    CK_STATE_BAX,
    CK_STATE_BAY,
    CK_STATE_BGZ,
} Chassis_Kalman_StateIndex_e;

/* 量测下标 */
typedef enum {
    CK_MEAS_WHEEL_VX = 0,
    CK_MEAS_WHEEL_VY,
    CK_MEAS_WZ_MINUS_WHEEL_VW,
} Chassis_Kalman_MeasIndex_e;

/**
 * @brief 底盘 Kalman 里程计输入
 */
typedef struct {
    float dt;                    // 采样周期 (s)

    /* 轮速正解 / 轮式里程计（需先由 Chassis_Forward 得到） */
    const Chassis_Feedback_t *fb; // 非空，取 vx / vy / vw

    /* IMU 原始量测 */
    float imu_ax;                // 车体系 X 向加速度 (m/s²，含重力)
    float imu_ay;                // 车体系 Y 向加速度 (m/s²，含重力)
    float imu_wz;                // IMU 偏航角速度 (rad/s)

    /* 姿态，用于扣除重力分量；单位 rad */
    float roll;
    float pitch;
    uint8_t attitude_valid;      // 姿态是否有效；无效时不扣重力
} Chassis_Kalman_Input_t;

/**
 * @brief 底盘 Kalman 里程计输出
 */
typedef struct {
    float vx;                    // 融合后底盘 X 速度 (m/s)
    float vy;                    // 融合后底盘 Y 速度 (m/s)
    float vw;                    // 融合后底盘 Z 角速度 (rad/s)

    float bax;                   // 估计 X 轴加速度零偏
    float bay;                   // 估计 Y 轴加速度零偏
    float bgz;                   // 估计 Z 轴陀螺零偏

    float confidence;            // 0~1，越大表示滤波结果越可信
    float residual_vx;           // 整车估计 vx - 轮式里程计 vx (m/s)
    float residual_vy;           // 整车估计 vy - 轮式里程计 vy (m/s)
    float residual_vw;           // 整车估计 vw - 轮式里程计 vw (rad/s)
    float adapt_weight_x;        // x 通道残差权重 0~1（越大越不信任轮式）
    float adapt_weight_y;        // y 通道残差权重 0~1
    float adapt_weight_w;        // 角速度通道残差权重 0~1
    float adapt_score;           // 0~1，三通道残差权重的综合评分
    float low_speed_gain_v;      // 线速度通道低速信任增益 0~1（1=最信任轮式）
    float low_speed_gain_w;      // 角速度通道低速信任增益 0~1

    float vx_var;                // vx 方差估计
    float vy_var;                // vy 方差估计
    float vw_var;                // vw 方差估计
} Chassis_Kalman_Output_t;

/**
 * @brief 底盘 Kalman 里程计实例
 */
typedef struct {
    KalmanFilter_t kf;           // 通用 Kalman 滤波工作区（kalman_filter.c）

    /* 可调参数（Init 后由调用方按实车标定） */
    float gravity;               // 重力加速度 (m/s²)
    float sigma_acc;             // 加速度计过程噪声标准差 (m/s²)
    float sigma_bias_acc;        // 加速度零偏随机游走标准差
    float sigma_bias_gyr;        // 陀螺零偏随机游走标准差
    float wheel_v_meas_std;      // 轮式里程计 vx/vy 量测标准差 (m/s)
    float wheel_w_meas_std;      // 轮式里程计 vw 量测标准差 (rad/s)

    /* 变 Q / 变 R 自适应：最大缩放与残差特征尺度 */
    float wheel_v_adapt_r_scale; // vx/vy 残差大时 R 最大放大倍数（>1）
    float wheel_v_adapt_q_div;   // vx/vy 残差大时 Q 最大除数（>1）
    float wheel_w_adapt_r_scale; // 角速度通道 R 最大放大倍数（>1）
    float wheel_w_adapt_q_div;   // 角速度通道 Q 最大除数（>1）
    float adapt_res_scale_v;     // vx/vy 残差特征尺度 (m/s)，残差≈该值时权重≈0.5
    float adapt_res_scale_w;     // 角速度残差特征尺度 (rad/s)

    /* 低速加权：低速时提高轮式里程计信任度（R 缩小） */
    float low_speed_v_lo;        // 线速度低于该值 (m/s) 时给满低速信任
    float low_speed_v_hi;        // 线速度高于该值 (m/s) 时低速信任为 0
    float low_speed_w_lo;        // |轮式 vw| 低于该值 (rad/s) 时给满低速信任
    float low_speed_w_hi;        // |轮式 vw| 高于该值 (rad/s) 时低速信任为 0
    float wheel_v_low_r_scale;   // 满低速信任时 vx/vy 的 R 缩放（0<scale<=1，越小越信任）
    float wheel_w_low_r_scale;   // 满低速信任时角速度通道 R 缩放（默认 1.0=不额外信任）

    /* 内部状态 */
    float residual_vx_smooth;    // x 残差低通状态 (m/s)
    float residual_vy_smooth;    // y 残差低通状态 (m/s)
    float residual_vw_smooth;    // 角速度残差低通状态 (rad/s)
    float adapt_weight_x;        // x 通道残差权重（0~1）
    float adapt_weight_y;        // y 通道残差权重（0~1）
    float adapt_weight_w;        // 角速度通道残差权重（0~1）
    float adapt_score;           // 综合残差评分（0~1）
    float low_speed_gain_v;      // 低速信任增益（0~1）
    float low_speed_gain_w;      // 低速信任增益（0~1）
    uint8_t initialized;
} Chassis_Kalman_t;

/** 初始化底盘 Kalman 里程计（内部状态清零，参数取默认值，之后可自行标定） */
void Chassis_Kalman_Init(Chassis_Kalman_t *kalman);

/** 执行一次底盘 Kalman 里程计更新 */
void Chassis_Kalman_Update(Chassis_Kalman_t *kalman,
                           const Chassis_Kalman_Input_t *in,
                           Chassis_Kalman_Output_t *out);

#endif //HORIZON_CONTROL_FRAMEWORK_CHASSIS_KALMAN_H
