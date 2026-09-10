//
// Created by qza on 2026/9/8.
//

/* 底盘里程计 Kalman 融合。模型与自适应说明见 Chassis_Kalman.h。
 *
 * 每拍流程：
 *   重力补偿 → 基值 Q/R + 自适应缩放 → 非线性预测（含转动坐标耦合）
 *   → 量测更新 → 输出 → 用本拍残差更新下一拍的自适应权重
 *
 * IMU 安装偏心（杠杆臂）的伪加速度不在这里补偿，需要时统一在 IMU_Task
 * 的 IMU_Accel_LeverArm_Compensate 里做，避免两边重复扣同一项。
 */

#include "Chassis_Kalman.h"

#include <math.h>
#include <string.h>

/* 预测由 Chassis_Kalman_Nonlinear_Predict 完成且 SkipEq1 = 1，
 * 通用滤波器的 B·u 通道用不到，故控制维数取 0（省掉 B/u 分配）。 */
#define CK_U 0u
#define CK_N CHASSIS_KALMAN_STATE_NUM
#define CK_M CHASSIS_KALMAN_MEAS_NUM

/**
 * @brief 数值限幅
 */
static float Chassis_Kalman_Clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

/**
 * @brief 扣除 IMU 加速度中的重力分量
 *
 * 车体系下重力加速度近似为：
 *   gx = -g * sin(pitch)
 *   gy =  g * sin(roll) * cos(pitch)
 *
 * @param kalman 滤波器实例（读取 gravity）
 * @param in     输入（roll / pitch / imu_ax / imu_ay）
 * @param ax     [输出] 扣除重力后的 X 向加速度
 * @param ay     [输出] 扣除重力后的 Y 向加速度
 */
static void Chassis_Kalman_Gravity_Compensate(const Chassis_Kalman_t *kalman,
                                              const Chassis_Kalman_Input_t *in,
                                              float *ax, float *ay)
{
    *ax = in->imu_ax;
    *ay = in->imu_ay;

    if (!in->attitude_valid) {
        return;
    }

    const float sr = sinf(in->roll);
    const float sp = sinf(in->pitch);
    const float cp = cosf(in->pitch);

    const float gx = -kalman->gravity * sp;
    const float gy =  kalman->gravity * sr * cp;

    *ax -= gx;
    *ay -= gy;
}

/**
 * @brief 计算基线过程噪声 Q / 量测噪声 R（对角阵）
 *
 * 状态顺序：vx, vy, bax, bay, bgz
 * 量测顺序：wheel_vx, wheel_vy, wz - wheel_vw
 *
 * @param kalman 滤波器实例
 * @param dt     采样周期
 * @param q      [输出] Q 对角元素
 * @param r      [输出] R 对角元素
 */
static void Chassis_Kalman_Base_Noise(const Chassis_Kalman_t *kalman,
                                      float dt,
                                      float q[CK_N], float r[CK_M])
{
    /* 加速度输入噪声累积到 vx/vy 的过程噪声 */
    const float q_vel = kalman->sigma_acc * kalman->sigma_acc * dt * dt;
    const float q_ba  = kalman->sigma_bias_acc * kalman->sigma_bias_acc * dt;
    const float q_bgz = kalman->sigma_bias_gyr * kalman->sigma_bias_gyr * dt;

    q[CK_STATE_VX]  = q_vel;
    q[CK_STATE_VY]  = q_vel;
    q[CK_STATE_BAX] = q_ba;
    q[CK_STATE_BAY] = q_ba;
    q[CK_STATE_BGZ] = q_bgz;

    const float r_v = kalman->wheel_v_meas_std * kalman->wheel_v_meas_std;
    const float r_w = kalman->wheel_w_meas_std * kalman->wheel_w_meas_std;

    r[CK_MEAS_WHEEL_VX] = r_v;
    r[CK_MEAS_WHEEL_VY] = r_v;
    r[CK_MEAS_WZ_MINUS_WHEEL_VW] = r_w;
}

/**
 * @brief 低速信任增益：速度越低越接近 1（0~1，线性过渡）
 *
 * speed <= lo → 1（满低速信任）；speed >= hi → 0（不额外信任）；
 * 中间线性过渡。hi <= lo 时退化为硬阈值。
 */
static float Chassis_Kalman_LowSpeed_Gain(float speed, float lo, float hi)
{
    if (speed <= lo) {
        return 1.0f;
    }
    if (speed >= hi || hi <= lo) {
        return 0.0f;
    }
    return (hi - speed) / (hi - lo);
}

/**
 * @brief 变 Q / 变 R 自适应
 *
 * 残差权重来自上一拍（见 Chassis_Kalman_Update_Residual），残差大→权重
 * 大→更不信任轮式量测：
 *   R = R0 · [1 + (R_scale - 1) · weight]
 *   Q = Q0 / [1 + (Q_div   - 1) · weight]
 *
 * 其上再叠加低速加权：线速度越低越信任轮式 vx/vy，把 R 再乘一个小于 1
 * 的系数；该系数乘 (1 - 该通道残差权重)，残差变大时淡出，避免把残差
 * 自适应已经放大的 R 又压回去。角速度通道默认不参与（scale = 1）。
 *
 * @param kalman 滤波器实例（读取残差权重与自适应/低速参数，回写低速增益）
 * @param in     输入（读取轮式 vx/vy/vw 作为速度调度量）
 * @param q      [输入/输出] Q 对角元素
 * @param r      [输入/输出] R 对角元素
 */
static void Chassis_Kalman_Adaptive_Noise(Chassis_Kalman_t *kalman,
                                          const Chassis_Kalman_Input_t *in,
                                          float q[CK_N], float r[CK_M])
{
    const float gx = kalman->adapt_weight_x;
    const float gy = kalman->adapt_weight_y;
    const float gw = kalman->adapt_weight_w;

    /* ---- 低速加权：速度越低，越信任轮式里程计（R 收缩） ---- */
    const float low_v = Chassis_Kalman_LowSpeed_Gain(
            sqrtf(in->fb->vx * in->fb->vx + in->fb->vy * in->fb->vy),
            kalman->low_speed_v_lo, kalman->low_speed_v_hi);
    const float low_w = Chassis_Kalman_LowSpeed_Gain(
            fabsf(in->fb->vw),
            kalman->low_speed_w_lo, kalman->low_speed_w_hi);

    kalman->low_speed_gain_v = low_v;
    kalman->low_speed_gain_w = low_w;

    /* 残差大时该通道低速信任淡出；scale 限幅避免 R 被压到 0 */
    const float trust_x = low_v * (1.0f - gx);
    const float trust_y = low_v * (1.0f - gy);
    const float trust_w = low_w * (1.0f - gw);
    const float v_low_scale = Chassis_Kalman_Clampf(
            kalman->wheel_v_low_r_scale, 0.01f, 1.0f);
    const float w_low_scale = Chassis_Kalman_Clampf(
            kalman->wheel_w_low_r_scale, 0.01f, 1.0f);

    /* R = R0 * [1 + (R_scale-1) * weight]，再叠加低速收缩 */
    r[CK_MEAS_WHEEL_VX] *= 1.0f +
            (kalman->wheel_v_adapt_r_scale - 1.0f) * gx;
    r[CK_MEAS_WHEEL_VY] *= 1.0f +
            (kalman->wheel_v_adapt_r_scale - 1.0f) * gy;
    r[CK_MEAS_WZ_MINUS_WHEEL_VW] *= 1.0f +
            (kalman->wheel_w_adapt_r_scale - 1.0f) * gw;

    r[CK_MEAS_WHEEL_VX] *= 1.0f - (1.0f - v_low_scale) * trust_x;
    r[CK_MEAS_WHEEL_VY] *= 1.0f - (1.0f - v_low_scale) * trust_y;
    r[CK_MEAS_WZ_MINUS_WHEEL_VW] *= 1.0f - (1.0f - w_low_scale) * trust_w;

    /* Q = Q0 / [1 + (Q_div-1) * weight] */
    q[CK_STATE_VX] /= 1.0f +
            (kalman->wheel_v_adapt_q_div - 1.0f) * gx;
    q[CK_STATE_VY] /= 1.0f +
            (kalman->wheel_v_adapt_q_div - 1.0f) * gy;
    q[CK_STATE_BGZ] /= 1.0f +
            (kalman->wheel_w_adapt_q_div - 1.0f) * gw;
}

/**
 * @brief 将 Q / R 对角写入 Kalman 矩阵工作区（非对角清零）
 */
static void Chassis_Kalman_Write_Noise(KalmanFilter_t *kf,
                                       const float q[CK_N],
                                       const float r[CK_M])
{
    memset(kf->Q_data, 0, sizeof(float) * CK_N * CK_N);
    memset(kf->R_data, 0, sizeof(float) * CK_M * CK_M);

    for (uint8_t i = 0; i < CK_N; i++) {
        kf->Q_data[i * CK_N + i] = q[i];
    }
    for (uint8_t i = 0; i < CK_M; i++) {
        kf->R_data[i * CK_M + i] = r[i];
    }
}

/**
 * @brief 配置量测矩阵 H（常矩阵，一次配置即可）
 */
static void Chassis_Kalman_Set_Measurement_Matrix(KalmanFilter_t *kf)
{
    memset(kf->H_data, 0, sizeof(float) * CK_M * CK_N);
    kf->H_data[CK_MEAS_WHEEL_VX * CK_N + CK_STATE_VX]  = 1.0f;
    kf->H_data[CK_MEAS_WHEEL_VY * CK_N + CK_STATE_VY]  = 1.0f;
    kf->H_data[CK_MEAS_WZ_MINUS_WHEEL_VW * CK_N + CK_STATE_BGZ] = 1.0f;
}

/**
 * @brief 初始化/复位误差协方差 P 与方差下限
 */
static void Chassis_Kalman_Set_Covariance(KalmanFilter_t *kf)
{
#define CK_P(idx) ((idx) * CK_N + (idx))
    memset(kf->P_data, 0, sizeof(float) * CK_N * CK_N);

    kf->P_data[CK_P(CK_STATE_VX)]  = 1.0f;
    kf->P_data[CK_P(CK_STATE_VY)]  = 1.0f;
    kf->P_data[CK_P(CK_STATE_BAX)] = 0.5f;
    kf->P_data[CK_P(CK_STATE_BAY)] = 0.5f;
    kf->P_data[CK_P(CK_STATE_BGZ)] = 0.2f;
#undef CK_P

    for (uint8_t i = 0; i < CK_N; i++) {
        kf->StateMinVariance[i] = 1e-6f;
    }
}

/**
 * @brief 非线性状态预测 + 雅可比 F（SkipEq1 = 1 时由 kalman_filter.c 调用）
 *
 * 状态预测（含转动坐标耦合 ω×v，角速度取轮式 vw）：
 *   vx' = vx + (ax + vw·vy_wheel - bax) * dt
 *   vy' = vy + (ay - vw·vx_wheel - bay) * dt
 *   bax' = bax, bay' = bay, bgz' = bgz
 * vw·vy / -vw·vx 为转动坐标耦合项，只在同时转弯和平移时生效。
 * IMU 安装偏心的向心/切向伪加速度不在这里补偿（需要时交给 IMU_Task）。
 *
 * F 为上述非线性函数相对状态向量 [vx, vy, bax, bay, bgz] 的雅可比，
 * 供 kalman_filter.c 做协方差传播。
 *
 * @param dt          采样周期
 * @param ax          扣除重力后的 X 向加速度
 * @param ay          扣除重力后的 Y 向加速度
 * @param wheel_vw    轮式里程计正解 vw (rad/s)
 * @param wheel_vx    轮式里程计正解 vx，用于转动坐标耦合
 * @param wheel_vy    轮式里程计正解 vy，用于转动坐标耦合
 */
static void Chassis_Kalman_Nonlinear_Predict(KalmanFilter_t *kf,
                                             float dt,
                                             float ax, float ay,
                                             float wheel_vw,
                                             float wheel_vx, float wheel_vy)
{
    const float *x = kf->xhat_data;

    /* ---- 雅可比 F ---- */
    float *F = kf->F_data;
    memset(F, 0, sizeof(float) * CK_N * CK_N);
    for (uint8_t i = 0; i < CK_N; i++) {
        F[i * CK_N + i] = 1.0f;
    }

    F[0 * CK_N + CK_STATE_BAX] = -dt;
    F[1 * CK_N + CK_STATE_BAY] = -dt;

    /* ---- 非线性状态预测 ---- */
    float *xminus = kf->xhatminus_data;
    xminus[CK_STATE_VX] = x[CK_STATE_VX]
                        + (ax + wheel_vw * wheel_vy
                           - x[CK_STATE_BAX]) * dt;
    xminus[CK_STATE_VY] = x[CK_STATE_VY]
                        + (ay - wheel_vw * wheel_vx
                           - x[CK_STATE_BAY]) * dt;
    xminus[CK_STATE_BAX] = x[CK_STATE_BAX];
    xminus[CK_STATE_BAY] = x[CK_STATE_BAY];
    xminus[CK_STATE_BGZ] = x[CK_STATE_BGZ];
}

/**
 * @brief 残差 → 0~1 连续权重
 *
 * 残差为 0 → 0；残差 ≈ 特征尺度 → 0.5；残差越大 → 趋近 1。
 */
static float Chassis_Kalman_Residual_Weight(float residual, float scale)
{
    const float a = fabsf(residual);
    if (scale <= 1e-6f) {
        scale = 1e-6f;
    }
    return a / (a + scale);
}

/**
 * @brief 用整车速度估计与轮式里程计残差更新连续权重
 *
 * 残差定义：
 *   residual_vx = xhat.vx - wheel.vx
 *   residual_vy = xhat.vy - wheel.vy
 *   residual_vw = (imu_wz - bgz) - wheel.vw
 *
 * 残差先按时间常数 ~0.15 s 平滑，再映射成 0~1 权重；下一拍由
 * Chassis_Kalman_Adaptive_Noise 用该权重调节 R（放大）/ Q（减小）。
 */
static void Chassis_Kalman_Update_Residual(Chassis_Kalman_t *kalman,
                                           const Chassis_Kalman_Input_t *in,
                                           Chassis_Kalman_Output_t *out)
{
    const float alpha = Chassis_Kalman_Clampf(in->dt / 0.15f, 0.0f, 1.0f);

    const float res_vx = out->vx - in->fb->vx;
    const float res_vy = out->vy - in->fb->vy;
    const float res_vw = out->vw - in->fb->vw;

    kalman->residual_vx_smooth += alpha * (res_vx - kalman->residual_vx_smooth);
    kalman->residual_vy_smooth += alpha * (res_vy - kalman->residual_vy_smooth);
    kalman->residual_vw_smooth += alpha * (res_vw - kalman->residual_vw_smooth);

    kalman->adapt_weight_x = Chassis_Kalman_Residual_Weight(
            kalman->residual_vx_smooth, kalman->adapt_res_scale_v);
    kalman->adapt_weight_y = Chassis_Kalman_Residual_Weight(
            kalman->residual_vy_smooth, kalman->adapt_res_scale_v);
    kalman->adapt_weight_w = Chassis_Kalman_Residual_Weight(
            kalman->residual_vw_smooth, kalman->adapt_res_scale_w);

    kalman->adapt_score = fmaxf(kalman->adapt_weight_x,
                         fmaxf(kalman->adapt_weight_y,
                               kalman->adapt_weight_w));

    out->residual_vx      = res_vx;
    out->residual_vy      = res_vy;
    out->residual_vw      = res_vw;
    out->adapt_weight_x   = kalman->adapt_weight_x;
    out->adapt_weight_y   = kalman->adapt_weight_y;
    out->adapt_weight_w   = kalman->adapt_weight_w;
    out->adapt_score      = kalman->adapt_score;
    out->low_speed_gain_v = kalman->low_speed_gain_v;
    out->low_speed_gain_w = kalman->low_speed_gain_w;
}

/**
 * @brief 初始化底盘 Kalman 里程计
 * @param kalman 滤波器实例
 */
void Chassis_Kalman_Init(Chassis_Kalman_t *kalman)
{
    if (kalman == NULL) {
        return;
    }

    memset(kalman, 0, sizeof(*kalman));

    Kalman_Filter_Init(&kalman->kf, CK_N, CK_U, CK_M);
    kalman->kf.UseAutoAdjustment = 0;
    /* 状态预测非线性，由 Chassis_Kalman_Nonlinear_Predict 完成 */
    kalman->kf.SkipEq1 = 1;

    Chassis_Kalman_Set_Measurement_Matrix(&kalman->kf);
    Chassis_Kalman_Set_Covariance(&kalman->kf);

    kalman->gravity          = 9.80665f;
    kalman->sigma_acc        = 20.0f;
    kalman->sigma_bias_acc   = 0.12f;
    kalman->sigma_bias_gyr   = 0.02f;
    kalman->wheel_v_meas_std = 0.006f;
    kalman->wheel_w_meas_std = 0.003f;

    kalman->wheel_v_adapt_r_scale = 500.0f;
    kalman->wheel_v_adapt_q_div   = 10.0f;
    kalman->wheel_w_adapt_r_scale = 50.0f;
    kalman->wheel_w_adapt_q_div   = 10.0f;
    kalman->adapt_res_scale_v     = 0.01f;
    kalman->adapt_res_scale_w     = 0.01f;

    /* 低速加权：5 cm/s 以下给满低速信任，20 cm/s 以上不再额外信任 */
    kalman->low_speed_v_lo      = 0.05f;
    kalman->low_speed_v_hi      = 0.20f;
    kalman->low_speed_w_lo      = 1.0f;
    kalman->low_speed_w_hi      = 1.5f;
    kalman->wheel_v_low_r_scale = 1.0f;
    kalman->wheel_w_low_r_scale = 0.01f;
}

/**
 * @brief 执行一次底盘 Kalman 里程计更新
 *
 * 调用前需通过 Chassis_Forward 得到 fb->vx / vy / vw。
 * Q/R 自适应完全由本滤波器内部残差与速度调度驱动。
 *
 * @param kalman 滤波器实例
 * @param in     输入
 * @param out    输出
 */
void Chassis_Kalman_Update(Chassis_Kalman_t *kalman,
                           const Chassis_Kalman_Input_t *in,
                           Chassis_Kalman_Output_t *out)
{
    if (kalman == NULL || in == NULL || out == NULL || in->fb == NULL) {
        return;
    }

    float dt = in->dt;
    if (dt <= 1e-5f) {
        dt = 1e-3f;
    }
    if (dt > 0.05f) {
        dt = 0.05f;
    }

    KalmanFilter_t *kf = &kalman->kf;

    /* ---- 1. 扣除重力分量 ---- */
    float ax = 0.0f;
    float ay = 0.0f;
    Chassis_Kalman_Gravity_Compensate(kalman, in, &ax, &ay);

    /* ---- 2. 基值噪声 + 变 Q/R 自适应，并写入 Q / R ---- */
    float q[CK_N];
    float r[CK_M];
    Chassis_Kalman_Base_Noise(kalman, dt, q, r);
    Chassis_Kalman_Adaptive_Noise(kalman, in, q, r);
    Chassis_Kalman_Write_Noise(kf, q, r);

    /* ---- 3. 首帧初始化 + 非线性预测 + 雅可比 F ---- */
    if (!kalman->initialized) {
        /* 首帧用轮式里程计与 IMU 之差初始化，加快收敛 */
        float *xhat = kf->xhat_data;
        xhat[CK_STATE_VX]  = in->fb->vx;
        xhat[CK_STATE_VY]  = in->fb->vy;
        xhat[CK_STATE_BAX] = 0.0f;
        xhat[CK_STATE_BAY] = 0.0f;
        xhat[CK_STATE_BGZ] = in->imu_wz - in->fb->vw;
        kalman->initialized = 1;
    }

    Chassis_Kalman_Nonlinear_Predict(kf, dt, ax, ay,
                                     in->fb->vw,
                                     in->fb->vx, in->fb->vy);

    /* ---- 4. 装载量测：轮式里程计 vx / vy / vw ---- */
    kf->MeasuredVector[CK_MEAS_WHEEL_VX] = in->fb->vx;
    kf->MeasuredVector[CK_MEAS_WHEEL_VY] = in->fb->vy;
    kf->MeasuredVector[CK_MEAS_WZ_MINUS_WHEEL_VW] =
            in->imu_wz - in->fb->vw;

    /* ---- 5. Kalman 协方差传播 + 量测更新（kalman_filter.c） ---- */
    float *x = Kalman_Filter_Update(kf);

    /* ---- 6. 输出 ---- */
    out->vx  = x[CK_STATE_VX];
    out->vy  = x[CK_STATE_VY];
    out->vw  = in->imu_wz - x[CK_STATE_BGZ];
    out->bax = x[CK_STATE_BAX];
    out->bay = x[CK_STATE_BAY];
    out->bgz = x[CK_STATE_BGZ];

    out->vx_var = kf->P_data[CK_STATE_VX * CK_N + CK_STATE_VX];
    out->vy_var = kf->P_data[CK_STATE_VY * CK_N + CK_STATE_VY];
    out->vw_var = kf->P_data[CK_STATE_BGZ * CK_N + CK_STATE_BGZ];

    const float cov_xy = 0.5f * (out->vx_var + out->vy_var);
    float conf = 1.0f / (1.0f +
                         2.0f * sqrtf(fmaxf(cov_xy, 0.0f)) +
                         1.5f * sqrtf(fmaxf(out->vw_var, 0.0f)));
    conf *= (1.0f - 0.8f * kalman->adapt_score);
    out->confidence = Chassis_Kalman_Clampf(conf, 0.0f, 1.0f);

    /* ---- 7. 用本拍残差更新下一拍的自适应权重 ---- */
    Chassis_Kalman_Update_Residual(kalman, in, out);
}
