//
// Created by qza on 2026/9/7.
//

#include "Kalman_Odometer.h"
#include <math.h>
#include <string.h>

#define KO_N   KALMAN_ODOMETER_STATE_NUM
#define KO_M   KALMAN_ODOMETER_MEAS_NUM

/* ---------------- 状态 / 输出互转 ---------------- */

static void Odometer_State_From_Output(const Kalman_Odometer_State_t *src, float dst[KO_N])
{
    dst[KO_STATE_POS_X] = src->pos_x;
    dst[KO_STATE_POS_Y] = src->pos_y;
    dst[KO_STATE_YAW]   = src->yaw;
    dst[KO_STATE_VX]    = src->vx;
    dst[KO_STATE_VY]    = src->vy;
    dst[KO_STATE_VW]    = src->vw;
    dst[KO_STATE_AX]    = src->ax;
    dst[KO_STATE_AY]    = src->ay;
}

static void Odometer_State_To_Output(const float src[KO_N], Kalman_Odometer_State_t *dst)
{
    dst->pos_x = src[KO_STATE_POS_X];
    dst->pos_y = src[KO_STATE_POS_Y];
    dst->yaw   = src[KO_STATE_YAW];
    dst->vx    = src[KO_STATE_VX];
    dst->vy    = src[KO_STATE_VY];
    dst->vw    = src[KO_STATE_VW];
    dst->ax    = src[KO_STATE_AX];
    dst->ay    = src[KO_STATE_AY];
}

/* ---------------- Kalman 矩阵模型配置 ---------------- */

/**
 * @brief 观测矩阵 H（6x8），下标用头文件枚举表达，避免魔法数字
 */
static void Odometer_Set_Measurement_Matrix(KalmanOdometer_t *odo)
{
    KalmanFilter_t *kf = &odo->kf;

    memset(kf->H_data, 0, sizeof(float) * KO_M * KO_N);
    kf->H_data[KO_MEAS_WHEEL_VX * KO_N + KO_STATE_VX] = 1.0f;
    kf->H_data[KO_MEAS_WHEEL_VY * KO_N + KO_STATE_VY] = 1.0f;
    kf->H_data[KO_MEAS_WHEEL_VW * KO_N + KO_STATE_VW] = 1.0f;
    kf->H_data[KO_MEAS_IMU_AX    * KO_N + KO_STATE_AX] = 1.0f;
    kf->H_data[KO_MEAS_IMU_AY    * KO_N + KO_STATE_AY] = 1.0f;
    kf->H_data[KO_MEAS_IMU_YAW_RATE * KO_N + KO_STATE_VW] = 1.0f;
}

/**
 * @brief 预测雅可比 F（8x8），围绕当前 yaw 线性化
 *
 *  行 0 (pos_x): 1, 0, -dt(vx*sin+vy*cos), dt*cos, -dt*sin, 0, 0, 0
 *  行 1 (pos_y): 0, 1,  dt(vx*cos-vy*sin), dt*sin,  dt*cos, 0, 0, 0
 *  行 2 (yaw):   0, 0,  1, 0, 0, dt, 0, 0
 *  行 3 (vx):    0, 0,  0, 1, 0, 0, dt, 0
 *  行 4 (vy):    0, 0,  0, 0, 1, 0, 0, dt
 */
static void Odometer_Set_State_Transition(KalmanOdometer_t *odo, float dt)
{
    KalmanFilter_t *kf = &odo->kf;
    const float *x     = kf->xhat_data;
    const float yaw    = x[KO_STATE_YAW];
    const float vx     = x[KO_STATE_VX];
    const float vy     = x[KO_STATE_VY];

    const float c = arm_cos_f32(yaw);
    const float s = arm_sin_f32(yaw);

    float *F = kf->F_data;
    memset(F, 0, sizeof(float) * KO_N * KO_N);
    for (int i = 0; i < KO_N; i++) {
        F[i * KO_N + i] = 1.0f;
    }

    /* pos_x */
    F[0 * KO_N + KO_STATE_YAW] = -dt * (vx * s + vy * c);
    F[0 * KO_N + KO_STATE_VX]  =  dt * c;
    F[0 * KO_N + KO_STATE_VY]  = -dt * s;

    /* pos_y */
    F[1 * KO_N + KO_STATE_YAW] = dt * (vx * c - vy * s);
    F[1 * KO_N + KO_STATE_VX]  = dt * s;
    F[1 * KO_N + KO_STATE_VY]  = dt * c;

    /* yaw / vx / vy */
    F[2 * KO_N + KO_STATE_VW] = dt;
    F[3 * KO_N + KO_STATE_AX] = dt;
    F[4 * KO_N + KO_STATE_AY] = dt;
}

/**
 * @brief 非线性状态预测：xhatminus = f(xhat, dt)
 *
 * F 只负责协方差传播；状态本身不用 F*x 近似，避免线性化误差累积。
 */
static void Odometer_Nonlinear_Predict(KalmanOdometer_t *odo, float dt)
{
    KalmanFilter_t *kf = &odo->kf;
    const float *x     = kf->xhat_data;
    const float c      = arm_cos_f32(x[KO_STATE_YAW]);
    const float s      = arm_sin_f32(x[KO_STATE_YAW]);

    float *xminus = kf->xhatminus_data;

    xminus[KO_STATE_POS_X] = x[KO_STATE_POS_X]
                           + (x[KO_STATE_VX] * c - x[KO_STATE_VY] * s) * dt;
    xminus[KO_STATE_POS_Y] = x[KO_STATE_POS_Y]
                           + (x[KO_STATE_VX] * s + x[KO_STATE_VY] * c) * dt;
    xminus[KO_STATE_YAW]   = x[KO_STATE_YAW] + x[KO_STATE_VW] * dt;
    xminus[KO_STATE_VX]    = x[KO_STATE_VX] + x[KO_STATE_AX] * dt;
    xminus[KO_STATE_VY]    = x[KO_STATE_VY] + x[KO_STATE_AY] * dt;
    xminus[KO_STATE_VW]    = x[KO_STATE_VW];
    xminus[KO_STATE_AX]    = x[KO_STATE_AX];
    xminus[KO_STATE_AY]    = x[KO_STATE_AY];
}

/**
 * @brief 初始协方差与方差下限
 */
static void Odometer_Set_Covariance_Config(KalmanOdometer_t *odo)
{
    KalmanFilter_t *kf = &odo->kf;

    memset(kf->P_data, 0, sizeof(float) * KO_N * KO_N);
    for (int i = 0; i < KO_N; i++) {
        kf->P_data[i * KO_N + i] = 1.0f;
        kf->StateMinVariance[i]  = 1e-6f;
    }
}

/* ---------------- Q/R 配置 ---------------- */

static void Load_Base_Noise(KalmanOdometer_t *odo)
{
    memcpy(odo->noise.q, odo->noise.q_base, sizeof(odo->noise.q));
    memcpy(odo->noise.r, odo->noise.r_base, sizeof(odo->noise.r));
}

static void Set_Default_Noise(KalmanOdometer_t *odo)
{
    Kalman_Odometer_Noise_t *n = &odo->noise;

    /* 状态顺序: x, y, yaw, vx, vy, vw, ax, ay */
    const float q_def[KO_N] = {
        0.01f, 0.01f, 0.01f,
        0.02f, 0.02f, 0.02f,
        0.50f, 0.50f
    };
    /* 量测顺序: vx轮, vy轮, vw轮, axIMU, ayIMU, yawIMU */
    const float r_def[KO_M] = {
        0.04f, 0.04f, 0.01f,
        0.25f, 0.25f, 0.01f
    };

    memcpy(n->q_base, q_def, sizeof(q_def));
    memcpy(n->r_base, r_def, sizeof(r_def));

    n->adaptive_enabled = true;
    n->adapter          = NULL;

    n->accel_high_th   = 3.0f;   /* m/s² */
    n->spin_high_th    = 2.0f;   /* rad/s */
    n->yaw_mismatch_th = 0.8f;   /* rad/s */

    n->q_scale_high_accel       = 4.0f;
    n->q_scale_high_spin        = 4.0f;
    n->r_wheel_scale_high_accel = 5.0f;
    n->r_imu_scale_high_accel   = 0.25f;
    n->r_wheel_scale_high_spin  = 5.0f;
    n->r_imu_scale_high_spin    = 0.25f;

    Load_Base_Noise(odo);
}

/**
 * @brief 把当次有效 q/r 对角写入 kf->Q_data / kf->R_data
 */
static void Write_Noise_To_Kalman(KalmanOdometer_t *odo)
{
    KalmanFilter_t *kf = &odo->kf;
    if (kf->Q_data == NULL || kf->R_data == NULL) return;

    memset(kf->Q_data, 0, sizeof(float) * KO_N * KO_N);
    memset(kf->R_data, 0, sizeof(float) * KO_M * KO_M);

    for (int i = 0; i < KO_N; i++) {
        kf->Q_data[i * KO_N + i] = odo->noise.q[i];
    }
    for (int m = 0; m < KO_M; m++) {
        kf->R_data[m * KO_M + m] = odo->noise.r[m];
    }
}

static void Odometer_Apply_Noise(KalmanOdometer_t *odo,
                                 const Chassis_Feedback_t *fb,
                                 float ax, float ay, float yaw_rate, float dt)
{
    if (odo->noise.adaptive_enabled) {
        if (odo->noise.adapter != NULL) {
            Load_Base_Noise(odo);
            odo->noise.adapter(odo, fb, ax, ay, yaw_rate, dt);
        } else {
            Kalman_Odometer_Default_Adaptive_Noise(odo, fb, ax, ay, yaw_rate, dt);
        }
    } else {
        Load_Base_Noise(odo);
    }

    Write_Noise_To_Kalman(odo);
}

/* ---------------- 公共接口 ---------------- */

void Kalman_Odometer_Init(KalmanOdometer_t *odo)
{
    if (odo == NULL) return;

    memset(odo, 0, sizeof(*odo));

    Kalman_Filter_Init(&odo->kf, KO_N, 0, KO_M);
    odo->kf.UseAutoAdjustment = 0;
    odo->kf.SkipEq1           = 1;  /* 状态预测由 Odometer_Nonlinear_Predict 完成 */

    Odometer_Set_Measurement_Matrix(odo);
    Odometer_Set_Covariance_Config(odo);
    Set_Default_Noise(odo);
    Write_Noise_To_Kalman(odo);

    odo->initialized = false;
    odo->dt          = 0.0f;
}

void Kalman_Odometer_Reset(KalmanOdometer_t *odo)
{
    if (odo == NULL) return;

    Kalman_Odometer_Noise_t noise = odo->noise; /* 保留 Q/R 调参 */

    Kalman_Filter_Reset(&odo->kf, KO_N, 0, KO_M);
    odo->kf.UseAutoAdjustment = 0;
    odo->kf.SkipEq1           = 1;

    Odometer_Set_Measurement_Matrix(odo);
    Odometer_Set_Covariance_Config(odo);

    odo->noise      = noise;
    Load_Base_Noise(odo);
    Write_Noise_To_Kalman(odo);

    memset(&odo->state, 0, sizeof(odo->state));
    odo->initialized = false;
    odo->dt          = 0.0f;
}

void Kalman_Odometer_Set_State(KalmanOdometer_t *odo,
                               const Kalman_Odometer_State_t *init)
{
    if (odo == NULL || init == NULL) return;

    float x[KO_N];
    Odometer_State_From_Output(init, x);
    memcpy(odo->kf.xhat_data, x, sizeof(x));

    odo->state      = *init;
    odo->initialized = true;
}

void Kalman_Odometer_Set_Noise(KalmanOdometer_t *odo,
                               const float q[KALMAN_ODOMETER_STATE_NUM],
                               const float r[KALMAN_ODOMETER_MEAS_NUM])
{
    if (odo == NULL || q == NULL || r == NULL) return;

    memcpy(odo->noise.q_base, q, sizeof(odo->noise.q_base));
    memcpy(odo->noise.r_base, r, sizeof(odo->noise.r_base));
    Load_Base_Noise(odo);
    Write_Noise_To_Kalman(odo);
}

void Kalman_Odometer_Set_Noise_Adapter(KalmanOdometer_t *odo,
                                       Kalman_Odometer_Noise_Adapter_t adapter)
{
    if (odo == NULL) return;
    odo->noise.adapter = adapter;
}

void Kalman_Odometer_Default_Adaptive_Noise(
        KalmanOdometer_t *odo,
        const Chassis_Feedback_t *fb,
        float ax, float ay, float yaw_rate, float dt)
{
    if (odo == NULL || fb == NULL) return;
    (void)dt;

    Kalman_Odometer_Noise_t *n = &odo->noise;
    Load_Base_Noise(odo);

    const float accel_norm = sqrtf(ax * ax + ay * ay);
    const float spin       = fabsf(yaw_rate);

    const bool high_accel   = (accel_norm >= n->accel_high_th);
    const bool high_spin    = (spin >= n->spin_high_th);
    const bool yaw_mismatch = (fabsf(yaw_rate - fb->vw) >= n->yaw_mismatch_th);

    /* 大加减速：轮子易打滑，放大速度/加速度过程噪声，
     * 同时降低对轮速的信任、相对提高对 IMU 加速度的信任。 */
    if (high_accel) {
        n->q[KO_STATE_VX] *= n->q_scale_high_accel;
        n->q[KO_STATE_VY] *= n->q_scale_high_accel;
        n->q[KO_STATE_AX] *= n->q_scale_high_accel;
        n->q[KO_STATE_AY] *= n->q_scale_high_accel;

        n->r[KO_MEAS_WHEEL_VX] *= n->r_wheel_scale_high_accel;
        n->r[KO_MEAS_WHEEL_VY] *= n->r_wheel_scale_high_accel;
        n->r[KO_MEAS_IMU_AX]   *= n->r_imu_scale_high_accel;
        n->r[KO_MEAS_IMU_AY]   *= n->r_imu_scale_high_accel;
    }

    /* 高转速或轮速 vw 与 IMU 角速度不一致：yaw/vw 过程噪声放大，
     * 轮速 vw 的可信度降低、IMU 角速度可信度提高。 */
    if (high_spin || yaw_mismatch) {
        n->q[KO_STATE_YAW] *= n->q_scale_high_spin;
        n->q[KO_STATE_VW]  *= n->q_scale_high_spin;

        n->r[KO_MEAS_WHEEL_VW] *= n->r_wheel_scale_high_spin;
        n->r[KO_MEAS_IMU_YAW_RATE] *= n->r_imu_scale_high_spin;
    }
}

void Kalman_Odometer_Update(KalmanOdometer_t *odo,
                            const Chassis_Feedback_t *fb,
                            float ax, float ay, float yaw_rate, float dt)
{
    if (odo == NULL || fb == NULL || dt <= 0.0f) return;

    KalmanFilter_t *kf = &odo->kf;

    odo->dt = dt;

    /* 1. 根据运动条件自适应 Q/R */
    Odometer_Apply_Noise(odo, fb, ax, ay, yaw_rate, dt);

    /* 2. 非线性状态预测 + 雅可比 F（F 仅用于协方差传播） */
    Odometer_Set_State_Transition(odo, dt);
    Odometer_Nonlinear_Predict(odo, dt);

    /* 3. 装载本轮观测 */
    const float z[KO_M] = {
        fb->vx,
        fb->vy,
        fb->vw,
        ax,
        ay,
        yaw_rate
    };
    memcpy(kf->MeasuredVector, z, sizeof(z));

    /* 4. 标准 Kalman 五式：预测 + 量测更新 */
    const float *filtered = Kalman_Filter_Update(kf);
    Odometer_State_To_Output(filtered, &odo->state);

    odo->initialized = true;
}
