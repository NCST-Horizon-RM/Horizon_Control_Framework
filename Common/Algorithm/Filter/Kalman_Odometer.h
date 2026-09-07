//
// Created by qza on 2026/9/7.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_KALMAN_ODOMETER_H
#define HORIZON_CONTROL_FRAMEWORK_KALMAN_ODOMETER_H

#include <stdbool.h>
#include "Chassis_Kinematics.h"
#include "kalman_filter.h"

/* =====================================================================
 * Kalman 8 状态里程计
 * ---------------------------------------------------------------------
 * 坐标系与现有底盘定义一致（右手系）：
 *   车体系：X 前、Y 左、Z 上；
 *   世界系：初始 yaw=0 时与车体系重合，x 前 / y 左 / yaw 逆时针为正。
 *
 * 状态向量（内部 xhat 顺序）：
 *   x = [ pos_x, pos_y, yaw, vx, vy, vw, ax, ay ]^T
 *     pos_x/pos_y : 世界系位移 (m)
 *     yaw         : 世界系偏航角，累计不折叠 (rad)
 *     vx/vy       : 车体系线速度 (m/s)
 *     vw          : 车体系偏航角速度 (rad/s)
 *     ax/ay       : 车体系线加速度 (m/s²)
 *
 * 预测（无控制输入，状态自转移，围绕当前 yaw 线性化）：
 *   dx   = ( vx*cos(yaw) - vy*sin(yaw) ) * dt
 *   dy   = ( vx*sin(yaw) + vy*cos(yaw) ) * dt
 *   dyaw = vw * dt
 *   dvx  = ax * dt
 *   dvy  = ay * dt
 *
 * 观测：
 *   z = [ fb.vx, fb.vy, fb.vw, ax_imu, ay_imu, yaw_rate_imu ]^T
 *   即轮式里程计速度 + IMU 线加速度 / 偏航角速度的冗余融合。
 *
 * Q/R 均按对角阵使用；每周期先恢复 q_base/r_base，再按运动状态
 * （大加速度 / 高转速 / 轮速与 IMU 角速度不一致）条件缩放。
 * 可通过 Kalman_Odometer_Set_Noise_Adapter 挂自定义策略。
 * ================================================================== */

#define KALMAN_ODOMETER_STATE_NUM 8
#define KALMAN_ODOMETER_MEAS_NUM  6

/* 内部状态下标（与 xhat_data/FilteredValue 一致） */
typedef enum {
    KO_STATE_POS_X = 0,
    KO_STATE_POS_Y,
    KO_STATE_YAW,
    KO_STATE_VX,
    KO_STATE_VY,
    KO_STATE_VW,
    KO_STATE_AX,
    KO_STATE_AY,
} Kalman_Odometer_StateIndex_e;

/* 观测下标（与 MeasuredVector 一致） */
typedef enum {
    KO_MEAS_WHEEL_VX = 0,
    KO_MEAS_WHEEL_VY,
    KO_MEAS_WHEEL_VW,
    KO_MEAS_IMU_AX,
    KO_MEAS_IMU_AY,
    KO_MEAS_IMU_YAW_RATE,
} Kalman_Odometer_MeasIndex_e;

/* 滤波后便于读取的输出结构 */
typedef struct {
    float pos_x;
    float pos_y;
    float yaw;
    float vx;
    float vy;
    float vw;
    float ax;
    float ay;
} Kalman_Odometer_State_t;

typedef struct KalmanOdometer KalmanOdometer_t;

/* Q/R 自适应回调；更新前 q/r 已恢复为 q_base/r_base */
typedef void (*Kalman_Odometer_Noise_Adapter_t)(
        KalmanOdometer_t *odo,
        const Chassis_Feedback_t *fb,
        float ax, float ay, float yaw_rate, float dt);

typedef struct {
    float q_base[KALMAN_ODOMETER_STATE_NUM];  /* 基线过程噪声（对角） */
    float r_base[KALMAN_ODOMETER_MEAS_NUM];   /* 基量测噪声（对角） */

    float q[KALMAN_ODOMETER_STATE_NUM];       /* 当次有效 Q */
    float r[KALMAN_ODOMETER_MEAS_NUM];        /* 当次有效 R */

    bool adaptive_enabled;
    Kalman_Odometer_Noise_Adapter_t adapter;  /* NULL 使用内置默认策略 */

    /* 内置策略条件 */
    float accel_high_th;
    float spin_high_th;
    float yaw_mismatch_th;

    /* 内置策略缩放系数 */
    float q_scale_high_accel;
    float q_scale_high_spin;
    float r_wheel_scale_high_accel;
    float r_imu_scale_high_accel;
    float r_wheel_scale_high_spin;
    float r_imu_scale_high_spin;
} Kalman_Odometer_Noise_t;

struct KalmanOdometer {
    KalmanFilter_t kf;                /* 8x6 标准 Kalman 矩阵工作区（CMSIS-DSP） */

    Kalman_Odometer_State_t state;    /* 滤波输出 */
    Kalman_Odometer_Noise_t noise;    /* Q/R 配置 */

    bool initialized;
    float dt;
};

void Kalman_Odometer_Init(KalmanOdometer_t *odo);
void Kalman_Odometer_Reset(KalmanOdometer_t *odo);

/* 设置初始位姿/速度，在首次 Update 前调用可选 */
void Kalman_Odometer_Set_State(KalmanOdometer_t *odo,
                               const Kalman_Odometer_State_t *init);

/* 轮式里程计正解速度 + IMU 原始量测 + 更新周期 */
void Kalman_Odometer_Update(KalmanOdometer_t *odo,
                            const Chassis_Feedback_t *fb,
                            float ax, float ay, float yaw_rate, float dt);

void Kalman_Odometer_Set_Noise(KalmanOdometer_t *odo,
                               const float q[KALMAN_ODOMETER_STATE_NUM],
                               const float r[KALMAN_ODOMETER_MEAS_NUM]);

void Kalman_Odometer_Set_Noise_Adapter(KalmanOdometer_t *odo,
                                       Kalman_Odometer_Noise_Adapter_t adapter);

void Kalman_Odometer_Default_Adaptive_Noise(
        KalmanOdometer_t *odo,
        const Chassis_Feedback_t *fb,
        float ax, float ay, float yaw_rate, float dt);

#endif //HORIZON_CONTROL_FRAMEWORK_KALMAN_ODOMETER_H
