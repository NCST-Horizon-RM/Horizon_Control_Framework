//
// Created by CaoKangqi on 2026/2/23.
//
#include "Chassis_Calc.h"
#include <math.h>
#include "All_define.h"
#include "Horizon_MATH.h"
#include "main.h"

__weak uint8_t Mecanum_Init(mecanumInit_typdef *mecanumInitT)
{
    mecanumInitT->wheel_r = 0.075f;
    mecanumInitT->half_wheelbase = 0.20f;   // 前后轮中心距的一半 (Lx)
    mecanumInitT->half_track_width = 0.20f; // 左右轮中心距的一半 (Ly)
    mecanumInitT->deceleration_ratio = 3591.0f / 187.0f;
    return 0;
}

/**
 * @brief 麦轮底盘运动学逆解计算
 * @param wheel_rpm 计算输出的各轮转速数组 (RPM)
 * @param vx X轴线速度 (前进为正, m/s)
 * @param vy Y轴线速度 (向左为正, m/s) - 注意符合右手坐标系
 * @param vw Z轴角速度 (逆时针旋转为正, rad/s)
 * @param mecanumInit_t 底盘参数结构体
 *
 * 轮子序号映射 (标准 O 型安装，俯视滚子呈 X 型):
 * [0] : FR (右前)
 * [1] : FL (左前)
 * [2] : BL (左后)
 * [3] : BR (右后)
 */
void Mecanum_Calc(float *wheel_rpm, float vx, float vy, float vw, mecanumInit_typdef *mecanumInit_t)
{
    // 计算力臂之和 (Lx + Ly)
    float lx_ly = mecanumInit_t->half_wheelbase + mecanumInit_t->half_track_width;
    float factor = (mecanumInit_t->deceleration_ratio * RADS_TO_RPM) / mecanumInit_t->wheel_r;
    wheel_rpm[0] = (-vx - vy - vw * lx_ly) * factor; // FR 右前
    wheel_rpm[1] = (vx - vy - vw * lx_ly) * factor; // FL 左前
    wheel_rpm[2] = (vx + vy - vw * lx_ly) * factor; // BL 左后
    wheel_rpm[3] = (-vx + vy - vw * lx_ly) * factor; // BR 右后
}

__weak uint8_t Omni_Init(OmniInit_typdef *OmniInit_t)
{
    OmniInit_t->wheel_r    = 0.075f;
    OmniInit_t->chassis_r= 0.25f;
    OmniInit_t->phi[0]= 45 * DEG2RAD;
    OmniInit_t->phi[1]= 135 * DEG2RAD;
    OmniInit_t->phi[2]= -135 * DEG2RAD;
    OmniInit_t->phi[3]= -45 * DEG2RAD;
    OmniInit_t->deceleration_ratio = 3591/187;
    return 0;
}

void Omni_Calc(float *wheel_rpm, float vx, float vy, float vw, OmniInit_typdef *OmniInit_t)
{
    for (int i = 0; i < 4; i++) {
        wheel_rpm[i] = ( -vx * cosf(OmniInit_t->phi[0]) + vy * sinf(OmniInit_t->phi[0])
                - vw * OmniInit_t->chassis_r) * OmniInit_t->deceleration_ratio / OmniInit_t->wheel_r * RADS_TO_RPM;
    }
}

#define M3508_NM_TO_RAW ( (1.0f / (15.7647f * 0.0157f * 0.85f)) * (16384.0f / 20.0f) )


__weak uint8_t Swerve_Init(Swerve_State_t *state) {
    if (state == NULL) return 1;
    __builtin_memset(state, 0, sizeof(Swerve_State_t));

    state->cfg.m = 10.5f;
    state->cfg.J = 1.0f;
    state->cfg.R = 0.24f;
    state->cfg.r = 0.06f;
    state->cfg.gear_d = 15.76f;

    state->cfg.Swerve_offset[0] = -120.0f * DEG2RAD;
    state->cfg.Swerve_offset[1] = -120.0f * DEG2RAD;
    state->cfg.Swerve_offset[2] = 60.0f * DEG2RAD;
    state->cfg.Swerve_offset[3] = 60.0f * DEG2RAD;

    state->cfg.phi[0] = 0.262f * PI;  state->cfg.phi[1] = 0.738f * PI;
    state->cfg.phi[2] = 1.262f * PI;  state->cfg.phi[3] = 1.738f * PI;

    return 0;
}

// 舵轮正解算
void Swerve_Forward_Calc(Swerve_State_t *now, const Swerve_Feedback_t *fb) {
    float b_x = 0, b_y = 0, b_w = 0;

    for (int i = 0; i < 4; i++) {
        now->wheel[i].v_wheel_now = (fb->wheel_rpm[i] * RPM_TO_RADS / now->cfg.gear_d) * now->cfg.r;

        float steer_chassis = fb->steer_angle_rad[i] - now->cfg.Swerve_offset[i];
        now->wheel[i].theta_now = normalize_to_pi(steer_chassis);

        float vix = now->wheel[i].v_wheel_now * cosf(steer_chassis);
        float viy = now->wheel[i].v_wheel_now * sinf(steer_chassis);

        b_x += now->wheel[i].v_wheel_now * cosf(now->wheel[i].theta_now);
        b_y += now->wheel[i].v_wheel_now * sinf(now->wheel[i].theta_now);
        b_w += (viy * sinf(now->cfg.phi[i]) - vix * cosf(now->cfg.phi[i])) / now->cfg.R;
    }

    now->vx = b_x / 4.0f;
    now->vy = b_y / 4.0f;
    now->vw = b_w / 4.0f;
}

// 舵轮逆解算
void Swerve_Inverse_Calc(Swerve_Command_t *cmd, Swerve_State_t *state,
                         float ax, float ay, float aw,
                         float vx, float vy, float vw,
                         const Swerve_Feedback_t *fb)
{
    state->ax_target = ax;
    state->ay_target = ay;
    state->aw_target = aw;
    state->vx_target = vx;
    state->vy_target = vy;
    state->vw_target = vw;

    for (int i = 0; i < 4; i++) {
        float vix = vx - state->cfg.R * vw * cosf(state->cfg.phi[i]);
        float viy = vy + state->cfg.R * vw * sinf(state->cfg.phi[i]);
        float v_mag = sqrtf(vix * vix + viy * viy);

        float current_theta_motor = fb->steer_angle_rad[i];
        float current_theta_chassis = current_theta_motor - state->cfg.Swerve_offset[i];

        float target_theta_raw;
        if (fabsf(v_mag) < 0.005f) {
            target_theta_raw = current_theta_chassis;
        } else {
            target_theta_raw = atan2f(viy, vix);
        }

        float diff = target_theta_raw - fmodf(current_theta_chassis, 2.0f * PI);
        while (diff >  PI) diff -= 2.0f * PI;
        while (diff < -PI) diff += 2.0f * PI;

        float speed_dir = 1.0f;
        if (fabsf(diff) > PI / 2.0f) {
            diff = (diff > 0) ? diff - PI : diff + PI;
            speed_dir = -1.0f;
        }

        state->wheel[i].theta_target = normalize_to_pi(current_theta_chassis + diff);
        state->wheel[i].v_wheel_target = speed_dir * v_mag * cosf(diff);

        cmd->target_steer_angle_rad[i] = current_theta_motor + diff;
        cmd->target_wheel_rpm[i] = (state->wheel[i].v_wheel_target / state->cfg.r) * state->cfg.gear_d / RPM_TO_RADS;

        // 动力学前馈解算
        float F_ix = (state->cfg.m * ax - state->cfg.J * aw / state->cfg.R * cosf(state->cfg.phi[i])) / 4.0f;
        float F_iy = (state->cfg.m * ay + state->cfg.J * aw / state->cfg.R * sinf(state->cfg.phi[i])) / 4.0f;
        float F_drive = F_ix * cosf(current_theta_chassis) + F_iy * sinf(current_theta_chassis);

        state->wheel[i].ff_out = (F_drive * state->cfg.r) * M3508_NM_TO_RAW;

        cmd->ff_torque_raw[i] = state->wheel[i].ff_out;
    }
}

__weak uint8_t Swerve_Init2(Swerve_State_t2*state)
{
    if (state == NULL) return 1;
    __builtin_memset(state, 0, sizeof(Swerve_State_t));

    state->cfg.phi[0] = 3*PI/4;
    state->cfg.phi[1] = PI/4;
    state->cfg.phi[2]= -3*PI/4;
    state->cfg.phi[3] = -PI/4;

    state->cfg.Swerve_offset[0] = 150.0f * DEG2RAD;
    state->cfg.Swerve_offset[1] = 150.0f * DEG2RAD;
    state->cfg.Swerve_offset[2] = 150.0f * DEG2RAD;
    state->cfg.Swerve_offset[3] = 150.0f * DEG2RAD;

    state->cfg.direction=-1;

    state->cfg.R = 0.29f;
    state->cfg.r = 0.06f;
    state->cfg.m = 24.0f;
    state->cfg.J = 3.0f;
    return 0;
}

void Swerve_Receive_Transform(Swerve_State_t2 *state, const Swerve_Feedback_t2 *fb) {
    for (int i = 0; i < 4; i++) {
        state->wheel[i].v_wheel_now = (fb->wheel_rpm[i] * RPM_TO_RADS/ state->cfg.gear_d);
        state->wheel[i].v_theta_now = (fb->steer_rpm[i] * RPM_TO_RADS);

        float steer_chassis = state->cfg.direction * (fb->steer_angle_encoder[i] * ENCODER_TO_RAD - state->cfg.Swerve_offset[i]);
        state->wheel[i].theta_now = normalize_to_pi(steer_chassis);
    }
}

void Swerve_Forward_Calc2(Swerve_State_t2 *now) {
    float b_x = 0, b_y = 0, b_w = 0;

    for (int i = 0; i < 4; i++) {
        float vix = now->wheel[i].v_wheel_now * cosf(now->wheel[i].theta_now );
        float viy = now->wheel[i].v_wheel_now * sinf(now->wheel[i].theta_now );

        b_x += now->wheel[i].v_wheel_now * cosf(now->wheel[i].theta_now);
        b_y += now->wheel[i].v_wheel_now * sinf(now->wheel[i].theta_now);
        b_w += (viy * sinf(now->cfg.phi[i]) - vix * cosf(now->cfg.phi[i])) / now->cfg.R;
    }

    now->vx = b_x / 4.0f;
    now->vy = b_y / 4.0f;
    now->vw = b_w / 4.0f;
}

void Swerve_Inverse_Calc2(Swerve_State_t2*state) {
    for (uint8_t i = 0; i < 4; i++) {
        state->wheel[i].v_wheel_target=(state->vx*cosf(state->wheel[i].theta_now)
                                        + state->vy*sinf(state->wheel[i].theta_now)
                                        + state->vw*state->cfg.R*sinf(state->wheel[i].theta_now-state->cfg.phi[i]))/state->cfg.r;
    }

}
void Wheel_Calculation(Swerve_State_t2*state){
    for (uint8_t i = 0; i < 4; i++) {
        state->wheel[i].wheel_torque_feedforward=
                    state->cfg.r * (state->cfg.m * state->ax_target *cosf(state->wheel[i].theta_now)

                    +state->cfg.m * state->ax_target * sinf(state->wheel[i].theta_now)

                    +state->cfg.J * state->ax_target * sinf(state->wheel[i].theta_now - state->cfg.phi[i]))/(state->cfg.R*4) ;

    }
}

void Yaw_Calculation(Swerve_State_t2*state)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (fabsf(state->vx_target-state->cfg.R*state->vw_target*sinf(state->cfg.phi[i])) <= 0.0001 && fabsf(state->vy_target+state->cfg.R*state->vw_target*cosf(state->cfg.phi[i])) <= 0.0001) {
            state->wheel[i].theta_target=state->wheel[i].theta_now;
        }
        float angle_rad = atan2f(state->vy_target+state->cfg.R*state->vw_target*cosf(state->cfg.phi[i]) , state->vx_target-state->cfg.R*state->vw_target*sinf(state->cfg.phi[i]));
        state->wheel[i].theta_target=state->wheel[i].theta_now+normalize_to_pi(angle_rad-state->wheel[i].theta_now);
    }
}

#define NM_ENCODER_6020 (0.741f*16384.0f/3.0f)
#define NM_ENCODER_3508 (0.3f*16384.0f*19.0f/20.0f/15.7f)

void Swerve_Send_Transform(Swerve_State_t2*state, Swerve_Command_t2 *cmd) {
    for (uint8_t i = 0; i < 4; i++) {
        cmd->steer_I[i]=state->wheel[i].steer_torque_pid*NM_ENCODER_6020;
        cmd->wheel_I[i]=(state->wheel[i].wheel_torque_feedforward+state->wheel[i].wheel_torque_pid)*NM_ENCODER_3508;
    }
}