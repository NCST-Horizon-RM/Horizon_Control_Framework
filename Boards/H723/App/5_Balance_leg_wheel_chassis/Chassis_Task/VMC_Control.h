#ifndef BALANCE_LEG_WHEEL_VMC_CONTROL_H
#define BALANCE_LEG_WHEEL_VMC_CONTROL_H

#include <stdbool.h>

#define VMC_L1_LENGTH 0.215f
#define VMC_L2_LENGTH 0.258f
#define VMC_L3_LENGTH 0.258f
#define VMC_L4_LENGTH 0.215f
#define VMC_THETA_RATE_LPF_HZ 25.0f
#define VMC_LEG_MIN_LENGTH_M 0.14f
#define VMC_LEG_MAX_LENGTH_M 0.32f

/** 目标腿角的绕行方向；正方向为逆时针，负方向为顺时针。 */
typedef enum {
    VMC_ANGLE_PATH_NEGATIVE = -1,
    VMC_ANGLE_PATH_SHORTEST = 0,
    VMC_ANGLE_PATH_POSITIVE = 1
} VMC_Angle_Path_t;

typedef struct {
    float length[3];
    float phi;
    float theta[3];          /**< [0] 世界系腿角 [-pi, pi]，向下为 0；[1] 滤波角速度 rad/s。 */
    float theta_unwrapped;   /**< 连续世界系腿角 rad，可超出 +/-pi，供恢复轨迹规划使用。 */
    float length_last[2];
    float theta_last[2];
    float Tp_front;
    float Tp_back;
    float support_force;
    float support_torque;
} VMC_LegSite_t;

typedef struct {
    float l1, l2, l3, l4;
    float JRM_l[2][2];
    float JRM_r[2][2];
    VMC_LegSite_t left;
    VMC_LegSite_t right;
    bool derivative_initialized;
} VMC_Control_t;

void VMC_Init(VMC_Control_t *vmc, float l1, float l2, float l3, float l4);

/**
 * @brief 反馈中断或重新标定后清除差分历史。
 * 下一次有效更新以当前姿态重新初始化连续角和速度，不保留此前圈数。
 */
void VMC_ResetFeedbackHistory(VMC_Control_t *vmc);

/**
 * @brief 更新正运动学、连续腿角和速度。
 * 连续跟踪要求相邻有效采样的真实腿角变化小于 pi；长时间中断后须重置历史。
 * 右侧输入保持电机反馈方向，内部取反后进入统一的 VMC 关节坐标系。
 * 非有限姿态/关节反馈将丢弃本帧并重置历史；无效 dt 仅更新位置，速度置零。
 */
void VMC_Update(VMC_Control_t *vmc, float dt, float body_pitch_rad,
                float pos_front_L, float pos_back_L,
                float pos_front_R, float pos_back_R);
void VMC_ForceToTorque(const float JRM[2][2], float force, float Tp_target,VMC_LegSite_t *leg);
bool VMC_TorqueToForce(const float JRM[2][2], float torque_front,
                       float torque_back, VMC_LegSite_t *leg);

/** 将角度归一化到 [-pi, pi]。 */
float VMC_WrapAngle(float angle);

/** 将 angle 展开到 reference 附近，返回相差不超过 pi 的等价角。 */
float VMC_UnwrapAngleNear(float angle, float reference);

/**
 * @brief 按指定绕行方向选择目标角的连续表示。
 * 例如 (-1.5, 1.2, NEGATIVE) 返回约 -5.083185，SHORTEST 返回约 1.2。
 * 同向目标返回起点，不额外转一圈；需要整圈运动时直接指定展开目标。
 * 非有限输入或无效 direction 返回 NaN；不判断碰撞和关节限位。
 */
float VMC_SelectAngleTarget(float start_unwrapped, float target_theta,
                            VMC_Angle_Path_t direction);

/**
 * @brief 固定装配分支的逆解，输出统一 VMC 坐标系下的关节角 [-pi, pi]。
 * target_theta 为世界系腿角，允许连续角；内部减去 body_pitch_rad。
 * 腿长须在 [VMC_LEG_MIN_LENGTH_M, VMC_LEG_MAX_LENGTH_M] 内且几何可达。
 * 失败时保持输出不变；不检查实际关节限位或碰撞。
 */
bool VMC_InverseKinematics(const VMC_Control_t *vmc,
                           float target_length,
                           float target_theta,
                           float body_pitch_rad,
                           float *pos_front,
                           float *pos_back);

/**
 * @brief 逆解后将前后关节角展开到参考角附近，避免目标跨 +/-pi 跳变。
 * 首帧参考传实际关节位置，后续逐点传上一周期目标，关节目标单步变化须小于 pi。
 * 参考和输出均为统一 VMC 坐标：右侧电机反馈作参考前须取反，输出到右电机时再取反。
 * 本函数逐点保持连续性，不负责选择整条路径，也不检查实际关节限位或碰撞。
 * 失败时保持输出不变。
 */
bool VMC_InverseKinematicsContinuous(const VMC_Control_t *vmc,
                                     float target_length,
                                     float target_theta,
                                     float body_pitch_rad,
                                     float reference_front,
                                     float reference_back,
                                     float *pos_front,
                                     float *pos_back);

#endif
