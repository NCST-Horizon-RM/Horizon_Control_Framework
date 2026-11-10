#ifndef BALANCE_LEG_WHEEL_STEP_CLIMB_H
#define BALANCE_LEG_WHEEL_STEP_CLIMB_H

#include "Self_Righting.h"

typedef enum {
    STEP_IDLE,     /* 等待碰阶触发。 */
    STEP_LIFT,     /* 先抬轮越过台阶边缘。 */
    STEP_SWING,    /* 轮心向平衡位置回摆。 */
    STEP_PLANT,    /* 放下驱动轮并确认承重。 */
    STEP_CAPTURE,  /* 平衡控制接管，轮力矩渐入。 */
} StepClimb_Phase_t;

typedef enum {
    STEP_NONE, STEP_COMPLETE, STEP_TIMEOUT, STEP_UNSTABLE, STEP_BAD_FEEDBACK,
} StepClimb_Result_t;

typedef struct {
    const VMC_Control_t *vmc;      /* 本拍更新后的腿部运动学。 */
    const float *quat_wxyz;        /* 机体到世界的姿态四元数。 */
    const float *gyro_radps;       /* 机体系角速度，单位弧度每秒。 */
    const float *accel_mps2;       /* 机体系比力，静止时包含重力对应的分量。 */
    float hip_torque_nm[4];     /* 已转换为 VMC 方向，顺序 LF/RF/LB/RB */
    float speed_mps, pitch_rad, roll_rad;
    bool armed;                  /* 只决定是否允许从空闲状态启动。 */
    bool feedback_valid;         /* 反馈失效时中止当前动作。 */
} StepClimb_Input_t;

typedef struct {
    StepClimb_Phase_t phase;
    StepClimb_Result_t result;
    float direction, elapsed_s, cooldown_s;
    float approach_s, impact_s, unload_s, confirm_s, landed_s, capture_s;
    float support, wheel_load[2];
    float lift_x[2], lift_travel_m;
    bool wheel_odometry_valid;
    SelfRighting_Torque_t torque;
} StepClimb_t;

/* 清除历史和轨迹。底盘停机或自起接管时调用。 */
void StepClimb_Reset(StepClimb_t *step);

/* 根据本拍反馈更新台阶状态。捕获阶段由调用方恢复平衡控制。 */
void StepClimb_Update(StepClimb_t *step, const StepClimb_Input_t *input,
                      const SelfRighting_Config_t *geometry, float dt);

#endif
