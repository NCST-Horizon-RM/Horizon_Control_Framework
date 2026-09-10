//
// Created by qza on 2026/9/5.
//

#include "Chassis_Kinematics.h"
#include <math.h>
#include <stddef.h>
#include "All_define.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

/* ==================== 力矩 → 电流 系数 ==================== */
#define CHASSIS_M3508_TORQUE_TO_RAW ((1.0f / (15.7647f * 0.0157f * 0.85f)) * (16384.0f / 20.0f))

/* 轮位置符号：RF(+Lx,-Ly) LF(+Lx,+Ly) LB(-Lx,+Ly) RB(-Lx,-Ly) */
static const float POS_X_SIGN[4] = { 1.0f,  1.0f, -1.0f, -1.0f };
static const float POS_Y_SIGN[4] = { -1.0f, 1.0f,  1.0f, -1.0f };

/* 轮面线速度（m/s，电机正方向）↔ 电机 rpm */
static inline float Wheel_Surface_Speed(const Chassis_Cfg_t *c, float rpm)
{
    return rpm * RPM_TO_RADS / c->gear_ratio * c->wheel_r;
}

static inline float Surface_Speed_To_Rpm(const Chassis_Cfg_t *c, float v)
{
    return v / c->wheel_r * c->gear_ratio * RADS_TO_RPM;
}

/* 舵轮实际驱动方向（右手系）。
 *
 * Chassis_Calc.c 里那套已通过实机验证的舵轮模型本身不是右手系：
 * 它把 +Y 取为底盘右方、yaw 正方向与右手系相反，等价于把 Y / yaw
 * 同时镜像。为了在保留已验证机械方向的前提下把该模型换算到
 * “前 +X、左 +Y、yaw 逆时针为正”的右手系：
 *
 *   th = steer_angle - steer_offset   （沿用 Chassis_Calc 的电机读数差）
 *   轮面速度方向 = (cos th, -sin th)  （即实际轮向角 = -th）
 *
 * 因此逆解的目标舵角取 -atan2(uy, ux)，正解与力控的投影也统一用
 * (cos th, -sin th)，而不是上一版错误的 (cos th, sin th)。
 */
static inline void Swerve_Drive_Dir(float th, float *dx, float *dy)
{
    *dx =  cosf(th);
    *dy = -sinf(th);
}

/* 角度归一化到 [-π, π] */
static inline float Wrap_Pi(float a)
{
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

/* 几何参数合法性（防止 App 覆写错误导致除零） */
static inline int Cfg_Valid(const Chassis_Cfg_t *c)
{
    return c && c->wheel_r > 1e-6f && c->gear_ratio > 1e-6f &&
           c->Lx > 1e-6f && c->Ly > 1e-6f;
}

/* ==================== 前向解算 ==================== */

static void Mecanum_Forward(const Chassis_Cfg_t *c, const float *v,
                            float *vx, float *vy, float *vw)
{
    float L = c->Lx + c->Ly;
    *vx = (-v[0] + v[1] + v[2] - v[3]) / 4.0f;
    *vy = (-v[0] - v[1] + v[2] + v[3]) / 4.0f;
    *vw = -(v[0] + v[1] + v[2] + v[3]) / (4.0f * L);
}

static void Omni_Forward(const Chassis_Cfg_t *c, const float *v,
                         float *vx, float *vy, float *vw)
{
    /* R = 轮心到质心距离，四轮等距。
     * 全向底盘的 X / Y 平移已按右手系对齐，yaw 也应满足右手系；
     * 这里直接使用统一的右手系旋转符号。 */
    float Lx = c->Lx, Ly = c->Ly;
    float R  = sqrtf(Lx * Lx + Ly * Ly);
    float k_vx = -R / (4.0f * Ly * Ly);
    float k_vy =  R / (4.0f * Lx * Lx);
    float k_vw =  1.0f / (4.0f * R);

    float s_py = 0.0f, s_px = 0.0f, s_v = 0.0f;
    for (int i = 0; i < 4; i++) {
        float px = POS_X_SIGN[i] * Lx;
        float py = POS_Y_SIGN[i] * Ly;
        s_py += py * v[i];
        s_px += px * v[i];
        s_v  += v[i];
    }
    *vx = k_vx * s_py;
    *vy = k_vy * s_px;
    *vw = k_vw * s_v;
}

static void Swerve_Forward(const Chassis_Cfg_t *c, const Chassis_Feedback_t *fb,
                           float *vx, float *vy, float *vw)
{
    /* 对称底盘下：平移用均值精确还原；旋转剥离平移贡献后对 ω 做
     * 标量最小二乘，避免四轮同向时 3x3 矩阵奇异。 */
    float v[4], ux[4], uy[4], dx, dy;
    float sx = 0.0f, sy = 0.0f;

    for (int i = 0; i < 4; i++) {
        float th = fb->steer_angle[i] - c->steer_offset[i];
        v[i] = Wheel_Surface_Speed(c, fb->wheel_rpm[i]);
        Swerve_Drive_Dir(th, &dx, &dy);

        /* 右手系底盘速度分量：u = v·(dx, dy) */
        ux[i] = v[i] * dx;
        uy[i] = v[i] * dy;
        sx += ux[i];
        sy += uy[i];
    }
    *vx = sx / 4.0f;
    *vy = sy / 4.0f;

    /* 去掉平移后的残差应等于 ω×p = (-py·ω, px·ω)。
     * 对每个轮同时用 x/y 两个分量做最小二乘：
     * Σ [px·(uy-vy) - py·(ux-vx)] / Σ(px²+py²) */
    float num = 0.0f, den = 0.0f;
    for (int i = 0; i < 4; i++) {
        float px = POS_X_SIGN[i] * c->Lx;
        float py = POS_Y_SIGN[i] * c->Ly;
        num += px * (uy[i] - *vy) - py * (ux[i] - *vx);
        den += px * px + py * py;
    }
    *vw = (den > 1e-6f) ? num / den : 0.0f;
}

/* ==================== 运动学逆解 ==================== */

static void Mecanum_Inverse(const Chassis_Cfg_t *c, float vx, float vy, float vw,
                            Chassis_Command_t *cmd)
{
    float L = c->Lx + c->Ly;
    float v[4] = {
        -vx - vy - vw * L,   // RF
         vx - vy - vw * L,   // LF
         vx + vy - vw * L,   // LB
        -vx + vy - vw * L,   // RB
    };
    for (int i = 0; i < 4; i++) {
        cmd->wheel_rpm_target[i] = Surface_Speed_To_Rpm(c, v[i]);
    }
}

static void Omni_Inverse(const Chassis_Cfg_t *c, float vx, float vy, float vw,
                         Chassis_Command_t *cmd)
{
    /* v_i = (-py·vx + px·vy)/R + vw·R
     * 平移项沿用右手系，yaw 也按右手系正方向输出。 */
    float Lx = c->Lx, Ly = c->Ly;
    float R  = sqrtf(Lx * Lx + Ly * Ly);
    for (int i = 0; i < 4; i++) {
        float px = POS_X_SIGN[i] * Lx;
        float py = POS_Y_SIGN[i] * Ly;
        float v = (-py * vx + px * vy) / R + vw * R;
        cmd->wheel_rpm_target[i] = Surface_Speed_To_Rpm(c, v);
    }
}

static void Swerve_Inverse(const Chassis_Cfg_t *c, float vx, float vy, float vw,
                           const Chassis_Feedback_t *fb, Chassis_Command_t *cmd)
{
    for (int i = 0; i < 4; i++) {
        float px = POS_X_SIGN[i] * c->Lx;
        float py = POS_Y_SIGN[i] * c->Ly;

        /* 该轮所需轮心速度 V + ω×p（右手系） */
        float ux   = vx - vw * py;
        float uy   = vy + vw * px;
        float umag = sqrtf(ux * ux + uy * uy);

        float theta_motor   = fb->steer_angle[i];
        float theta_chassis = theta_motor - c->steer_offset[i];

        float delta, v_wheel;
        if (umag < 0.005f) {
            delta   = 0.0f;       // 低速保持当前舵向
            v_wheel = 0.0f;
        } else {
            /* 实际轮向角 = -theta_chassis，所以目标舵角在电机读数差
             * 坐标系下为 -atan2(uy, ux)。 */
            float theta_des = -atan2f(uy, ux);
            delta = Wrap_Pi(theta_des - Wrap_Pi(theta_chassis));
            if (fabsf(delta) > PI / 2.0f) {
                delta   = (delta > 0.0f) ? delta - PI : delta + PI;
                v_wheel = -umag;
            } else {
                v_wheel = umag;
            }
        }

        cmd->steer_angle_target[i] = theta_motor + delta;
        cmd->wheel_rpm_target[i]   = Surface_Speed_To_Rpm(c, v_wheel);
    }
}

/* ==================== 力控分配 ==================== */

static void Mecanum_Force(const Chassis_Cfg_t *c, float fx, float fy, float mz,
                          float *torque_raw)
{
    float L = c->Lx + c->Ly;
    float f[4] = {
        -(fx + fy) / 4.0f - mz / (4.0f * L),   // RF
         (fx - fy) / 4.0f - mz / (4.0f * L),   // LF
         (fx + fy) / 4.0f - mz / (4.0f * L),   // LB
        (-fx + fy) / 4.0f - mz / (4.0f * L),   // RB
    };
    for (int i = 0; i < 4; i++) {
        torque_raw[i] = f[i] * c->wheel_r * c->torque_to_raw;
    }
}

static void Omni_Force(const Chassis_Cfg_t *c, float fx, float fy, float mz,
                       float *torque_raw)
{
    /* 力分配 = 速度雅可比的对偶，平移项保持右手系，yaw 力矩同向分配：
     * f_i = -py·R·fx/(4Ly²) + px·R·fy/(4Lx²) + mz/(4R) */
    float Lx = c->Lx, Ly = c->Ly;
    float R  = sqrtf(Lx * Lx + Ly * Ly);
    float k_fx = -R / (4.0f * Ly * Ly);
    float k_fy =  R / (4.0f * Lx * Lx);
    float k_mz =  1.0f / (4.0f * R);

    for (int i = 0; i < 4; i++) {
        float px = POS_X_SIGN[i] * Lx;
        float py = POS_Y_SIGN[i] * Ly;
        float f = py * k_fx * fx + px * k_fy * fy + k_mz * mz;
        torque_raw[i] = f * c->wheel_r * c->torque_to_raw;
    }
}

/* 期望轮端力：平移均分 + 绕心 yaw 力矩项。
 * F_yaw = mz·(-py, px)/Σ|p|²，满足 Σ p×F_yaw = mz */
static inline void Swerve_Force_Vec(float fx, float fy, float mz,
                                    float px, float py, float sum_p2,
                                    float *F_ix, float *F_iy)
{
    *F_ix = fx / 4.0f - mz * py / sum_p2;
    *F_iy = fy / 4.0f + mz * px / sum_p2;
}

/* 只做力矩分配（供力速混控使用，避免覆盖速度逆解给出的舵向目标）：
 * 将每轮期望轮端力投影到“当前实际轮向” (cos th, -sin th)。 */
static void Swerve_Force_Torque(const Chassis_Cfg_t *c, float fx, float fy, float mz,
                                const Chassis_Feedback_t *fb, float *torque_raw)
{
    /* sum_p2 = Σ|p_i|² = 4(Lx²+Ly²) */
    float sum_p2 = 4.0f * (c->Lx * c->Lx + c->Ly * c->Ly);
    if (sum_p2 < 1e-6f) {
        for (int i = 0; i < 4; i++) torque_raw[i] = 0.0f;
        return;
    }

    for (int i = 0; i < 4; i++) {
        float px = POS_X_SIGN[i] * c->Lx;
        float py = POS_Y_SIGN[i] * c->Ly;

        float F_ix, F_iy;
        Swerve_Force_Vec(fx, fy, mz, px, py, sum_p2, &F_ix, &F_iy);

        /* 沿当前实际轮向 (cos th, -sin th) 的投影 */
        float th = fb->steer_angle[i] - c->steer_offset[i];
        float F_drive = F_ix * cosf(th) - F_iy * sinf(th);

        torque_raw[i] = F_drive * c->wheel_r * c->torque_to_raw;
    }
}

/* 完整力控：力矩分配 + 舵向位置目标解算。
 * 单独使用 Chassis_Force 做舵轮力控时，舵向目标由期望轮端力矢量决定；
 * 力速混控（速度逆解已给出舵向）只调用 Swerve_Force_Torque。 */
static void Swerve_Force(const Chassis_Cfg_t *c, float fx, float fy, float mz,
                         const Chassis_Feedback_t *fb, Chassis_Command_t *cmd)
{
    /* 力矩仍按当前实际轮向分配，保证任意瞬时施力方向都与轮向一致 */
    Swerve_Force_Torque(c, fx, fy, mz, fb, cmd->wheel_torque_raw);

    float sum_p2 = 4.0f * (c->Lx * c->Lx + c->Ly * c->Ly);
    if (sum_p2 < 1e-6f) return;

    for (int i = 0; i < 4; i++) {
        float px = POS_X_SIGN[i] * c->Lx;
        float py = POS_Y_SIGN[i] * c->Ly;

        float F_ix, F_iy;
        Swerve_Force_Vec(fx, fy, mz, px, py, sum_p2, &F_ix, &F_iy);
        float F_mag = sqrtf(F_ix * F_ix + F_iy * F_iy);

        float theta_motor   = fb->steer_angle[i];
        float theta_chassis = theta_motor - c->steer_offset[i];

        if (F_mag < 0.01f) {
            cmd->steer_angle_target[i] = theta_motor;   // 无力需求时保持当前舵向
            continue;
        }

        /* 轮向对齐到 F_i；与速度逆解相同的 -atan2 / 就近换向规则 */
        float theta_des = -atan2f(F_iy, F_ix);
        float delta = Wrap_Pi(theta_des - Wrap_Pi(theta_chassis));
        if (fabsf(delta) > PI / 2.0f) {
            delta = (delta > 0.0f) ? delta - PI : delta + PI;
        }
        cmd->steer_angle_target[i] = theta_motor + delta;
    }
}

/* ==================== 公共入口 ==================== */

/* 弱符号默认实现：真实物理参数（轮径/轴距/减速比/质量/惯量/力矩常数/舵偏置）
 * 应由 App 层定义同名强符号 Chassis_Init 覆写，算法层不改动。 */
__weak uint8_t Chassis_Init(Chassis_Cfg_t *cfg, Chassis_Type_e type)
{
    if (cfg == NULL) return 1;

    cfg->type = type;
    cfg->mass = 10.5f;
    cfg->inertia = 1.0f;
    cfg->torque_to_raw = CHASSIS_M3508_TORQUE_TO_RAW;
    for (int i = 0; i < 4; i++) cfg->steer_offset[i] = 0.0f;

    switch (type) {
    case MECANUM:
        cfg->wheel_r = 0.075f;
        cfg->Lx = 0.20f;
        cfg->Ly = 0.20f;
        cfg->gear_ratio = 3591.0f / 187.0f;
        break;
    case OMNI:
        cfg->wheel_r = 0.075f;
        cfg->Lx = 0.25f;
        cfg->Ly = 0.25f;
        cfg->gear_ratio = 3591.0f / 187.0f;
        break;
    case SWERVE:
        cfg->wheel_r = 0.06f;
        cfg->Lx = 0.24f;
        cfg->Ly = 0.24f;
        cfg->gear_ratio = 15.76f;
        break;
    default:
        return 1;
    }
    return 0;
}

void Chassis_Forward(const Chassis_Cfg_t *cfg, Chassis_Feedback_t *fb)
{
    if (!Cfg_Valid(cfg) || fb == NULL) return;

    switch (cfg->type) {
    case MECANUM: {
        float v[4];
        for (int i = 0; i < 4; i++) v[i] = Wheel_Surface_Speed(cfg, fb->wheel_rpm[i]);
        Mecanum_Forward(cfg, v, &fb->vx, &fb->vy, &fb->vw);
        break;
    }
    case OMNI: {
        float v[4];
        for (int i = 0; i < 4; i++) v[i] = Wheel_Surface_Speed(cfg, fb->wheel_rpm[i]);
        Omni_Forward(cfg, v, &fb->vx, &fb->vy, &fb->vw);
        break;
    }
    case SWERVE:
        Swerve_Forward(cfg, fb, &fb->vx, &fb->vy, &fb->vw);
        break;
    default:
        fb->vx = fb->vy = fb->vw = 0.0f;
        break;
    }
}

void Chassis_Inverse(const Chassis_Cfg_t *cfg, float vx, float vy, float vw,
                     const Chassis_Feedback_t *fb, Chassis_Command_t *cmd)
{
    if (!Cfg_Valid(cfg) || cmd == NULL) return;

    for (int i = 0; i < 4; i++) {
        cmd->wheel_torque_raw[i] = 0.0f;
        cmd->steer_angle_target[i] = 0.0f;
        cmd->wheel_rpm_target[i] = 0.0f;
    }

    switch (cfg->type) {
    case MECANUM:
        Mecanum_Inverse(cfg, vx, vy, vw, cmd);
        break;
    case OMNI:
        Omni_Inverse(cfg, vx, vy, vw, cmd);
        break;
    case SWERVE:
        if (fb == NULL) break;
        Swerve_Inverse(cfg, vx, vy, vw, fb, cmd);
        break;
    default:
        break;
    }
}

void Chassis_Force(const Chassis_Cfg_t *cfg, float ax, float ay, float aw,
                   const Chassis_Feedback_t *fb, Chassis_Command_t *cmd)
{
    if (!Cfg_Valid(cfg) || cmd == NULL) return;

    float fx = cfg->mass * ax;
    float fy = cfg->mass * ay;
    float mz = cfg->inertia * aw;

    switch (cfg->type) {
    case MECANUM:
        Mecanum_Force(cfg, fx, fy, mz, cmd->wheel_torque_raw);
        break;
    case OMNI:
        Omni_Force(cfg, fx, fy, mz, cmd->wheel_torque_raw);
        break;
    case SWERVE:
        if (fb == NULL) {
            for (int i = 0; i < 4; i++) cmd->wheel_torque_raw[i] = 0.0f;
            break;
        }
        Swerve_Force(cfg, fx, fy, mz, fb, cmd);
        break;
    default:
        for (int i = 0; i < 4; i++) cmd->wheel_torque_raw[i] = 0.0f;
        break;
    }
}

void Chassis_Mixed_Control(const Chassis_Cfg_t *cfg,
                           float vx, float vy, float vw,
                           float ax, float ay, float aw,
                           const Chassis_Feedback_t *fb, Chassis_Command_t *cmd)
{
    if (!Cfg_Valid(cfg) || cmd == NULL) return;

    Chassis_Inverse(cfg, vx, vy, vw, fb, cmd);   // 轮速 + 舵角

    float fx = cfg->mass * ax;
    float fy = cfg->mass * ay;
    float mz = cfg->inertia * aw;

    if (cfg->type == SWERVE && fb != NULL) {
        /* 舵向已由速度逆解给出，这里只补力矩，避免被加速度舵向覆盖 */
        Swerve_Force_Torque(cfg, fx, fy, mz, fb, cmd->wheel_torque_raw);
    } else {
        Chassis_Force(cfg, ax, ay, aw, fb, cmd);
    }
}
