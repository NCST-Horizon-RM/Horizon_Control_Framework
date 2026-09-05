//
// Created by qza on 2026/9/5.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_CHASSIS_KINEMATICS_H
#define HORIZON_CONTROL_FRAMEWORK_CHASSIS_KINEMATICS_H

/*底盘运动学（麦轮 / 全向轮 / 舵轮）
速控 + 力控 + 力速混控

============================ 统一约定 ============================

坐标系（右手系）：
  前为 +X，左为 +Y，上为 +Z。
  线速度 vx 前进为正 (m/s)，vy 左移为正 (m/s)。
  角速度 vw 绕 +Z 逆时针(俯视)为正 (rad/s)。
  加速度 ax/ay (m/s²)、aw (rad/s²) 方向同 vx/vy/vw。

轮序（俯视逆时针，右前为 1 号轮 = 下标 0）：
  0 = 右前 RF    1 = 左前 LF    2 = 左后 LB    3 = 右后 RB

轮位置（右手系，Lx = 前后半轴距，Ly = 左右半轮距）：
  RF(+Lx,-Ly)    LF(+Lx,+Ly)    LB(-Lx,+Ly)    RB(-Lx,-Ly)

电机方向约定：
  「电机正转」在左轮(LF/LB)对应底盘 +X 前进，在右轮(RF/RB)对应 -X 后退。
  故同一 +X 前进下，左轮目标为正、右轮目标为负。本模块所有轮速/力矩输出
  均已包含该镜像符号，可直接作为电机 rpm / 电流原始值下发，无需再乘符号。

输出单位：
  wheel_rpm_target   —— 电机转速 (rpm)，已含左右镜像符号
  steer_angle_target —— 舵向目标角 (rad)，舵电机坐标系（含 offset）
  wheel_torque_raw   —— 电机电流原始值 (raw)，已含左右镜像符号与力矩→电流系数

舵向角：
  fb.steer_angle 为舵电机绝对角 (rad)。模块内部减 steer_offset 换算到
  底盘坐标系；cmd.steer_angle_target 返回舵电机坐标系角，供舵向环直接跟踪。
  steer_offset[i] = 舵电机角在「轮指向底盘 +X」时的读数。
====================================================================*/

#include <stdint.h>

typedef enum {
    MECANUM = 0,   // 麦轮
    OMNI,          // 全向轮
    SWERVE,        // 舵轮
} Chassis_Type_e;

/* 底盘几何 / 动力学参数 */
typedef struct {
    Chassis_Type_e type;
    float wheel_r;         // 轮半径 (m)
    float Lx;              // 前后半轴距 (m)
    float Ly;              // 左右半轮距 (m)
    float gear_ratio;      // 驱动减速比（轮转速 = 电机转速 / gear_ratio）
    float mass;            // 底盘质量 (kg)
    float inertia;         // 绕 Z 轴转动惯量 (kg·m²)
    float torque_to_raw;   // 轮端力矩 (N·m) → 电机电流原始值 系数
    float steer_offset[4]; // 舵向电机零点偏置 (rad)
} Chassis_Cfg_t;

/* 解算输入：电机反馈 */
typedef struct {
    float vx;
    float vy;
    float vw;
    float wheel_rpm[4];    // 驱动轮转速 (rpm)
    float steer_angle[4];  // 舵向角 (rad，舵电机坐标系，仅舵轮)
} Chassis_Feedback_t;

/* 解算输出 */
typedef struct {
    float wheel_rpm_target[4];   // 目标电机转速 (rpm)
    float steer_angle_target[4]; // 目标舵向角 (rad，舵电机坐标系)
    float wheel_torque_raw[4];   // 目标电机电流原始值 (raw，力控前馈)
} Chassis_Command_t;

/* 初始化：算法层提供 __weak 默认值，实际物理参数应在 App 层定义同名强符号覆写 */
uint8_t Chassis_Init(Chassis_Cfg_t *cfg, Chassis_Type_e type);

/* 正解：轮反馈 → 底盘实际速度 (vx, vy, vw) */
void Chassis_Forward(const Chassis_Cfg_t *cfg, Chassis_Feedback_t *fb);

/* 逆解 / 速控：目标速度 → 目标轮速（+ 舵轮舵向角） */
void Chassis_Inverse(const Chassis_Cfg_t *cfg, float vx, float vy, float vw,
                     const Chassis_Feedback_t *fb, Chassis_Command_t *cmd);

/* 力控：目标加速度 (ax, ay, aw) → 各轮力矩电流 */
void Chassis_Force(const Chassis_Cfg_t *cfg, float ax, float ay, float aw,
                   const Chassis_Feedback_t *fb, Chassis_Command_t *cmd);

/* 力速混控：目标速度 + 加速度 → 轮速/舵角 + 力矩电流（= Inverse + Force） */
void Chassis_Mixed_Control(const Chassis_Cfg_t *cfg,
                           float vx, float vy, float vw,
                           float ax, float ay, float aw,
                           const Chassis_Feedback_t *fb, Chassis_Command_t *cmd);

#endif //HORIZON_CONTROL_FRAMEWORK_CHASSIS_KINEMATICS_H
