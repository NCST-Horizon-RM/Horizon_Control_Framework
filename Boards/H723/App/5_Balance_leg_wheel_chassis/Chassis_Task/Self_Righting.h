#ifndef BALANCE_LEG_WHEEL_SELF_RIGHTING_H
#define BALANCE_LEG_WHEEL_SELF_RIGHTING_H

#include <stddef.h>
#include "VMC_Control.h"

typedef enum {
    SR_WATCH,        /* 正常监测。 */
    SR_SETTLE,       /* 等机身角速度降低，再选恢复路线。 */
    SR_FLIP_PLACE,   /* 倒扣时将轮摆到可撑地的位置。 */
    SR_FLIP_PUSH,    /* 驱动双腿撑地翻身。 */
    SR_FLIP_BRAKE,   /* 翻转后缩腿制动。 */
    SR_REPOSITION,   /* 规划双腿绕行到收轮起点。 */
    SR_ROLL_IN,      /* 沿地收轮，或以机身为支撑对正。 */
    SR_RISE,         /* 交接给平衡控制。 */
    SR_BLOCKED,      /* 重试耗尽，等待姿态改变。 */
    SR_FAULT         /* 反馈或恢复时间异常，锁定输出。 */
} SelfRighting_Phase_t;

typedef enum { SR_GROUND_SHORT, SR_REAR_LOWER, SR_UPPER_ARC } SelfRighting_Route_t;

typedef enum {
    SR_OK, SR_NO_PROGRESS, SR_LEFT_ROLL_PATH, SR_OVERTURNED,
    SR_UNREACHABLE, SR_BAD_INPUT, SR_TIMEOUT
} SelfRighting_Reason_t;

/* 统一 VMC 方向。髋顺序 LF/RF/LB/RB，轮顺序 L/R，单位 N*m。 */
typedef struct {
    float hip[4];
    float wheel[2];                  /* 正力矩驱动车轮向车头方向滚动 */
} SelfRighting_Torque_t;

typedef struct {
    /* 机体系 X 前、Y 左、Z 上。下列坐标须相对同一原点。 */
    const float (*support_points_m)[3]; /* 会撑地的机身外轮廓，不含活动腿 */
    size_t support_count;              /* 点表须在控制器生命周期内有效 */
    float hip_origin_m[2][3];          /* 左/右虚拟髋原点 */
    float wheel_lateral_m[2];          /* 左/右轮中心相对各自髋平面的 Y 偏移 */
    float hip_min_rad[4], hip_max_rad[4]; /* 统一 VMC 方向、连续编码器角 */
    float hip_limit_nm, wheel_limit_nm;
    float min_length_m, swing_length_m, capture_length_m, wheel_radius_m;
    float wheel_effective_mass_kg;
    float timeout_s;
} SelfRighting_Config_t;

typedef struct {
    const VMC_Control_t *vmc;        /* 当前周期 VMC_Update 的结果 */
    float quat_wxyz[4];              /* IMU_Data.q，机体 -> 世界，WXYZ */
    float gyro_radps[3];
    float accel_mps2[3];             /* 比力，静止正放 Z 约 +g */
    float motor_pos_rad[4];          /* BM 原方向，不在调用侧反右髋 */
    float motor_vel_radps[4];        /* 同上，顺序 LF/RF/LB/RB */
    float wheel_vel_radps[2];        /* 已换算至轮侧，前进为正 */
    SelfRighting_Torque_t applied;   /* 上拍实际下发，含限幅/功率限制，统一方向 */
    float leg_gravity_nm[4];         /* 可选腿重力补偿，统一方向，未知时填 0 */
    bool armed;                     /* 安全锁和使能通过后才为 true */
    bool feedback_valid;            /* 本拍 IMU、髋和轮反馈均可用 */
    bool jump_protected;            /* 跳跃时暂缓新一轮自起触发。 */
    bool step_protected;            /* 磕台阶轨迹可能主动大幅摆腿；翻倒判定仍生效。 */
} SelfRighting_Input_t;

typedef struct {
    SelfRighting_Phase_t phase;
    SelfRighting_Reason_t reason;
    bool active;                    /* 抑制用户指令 */
    bool handoff;                   /* 单周期事件，重置正常平衡控制器状态 */
    bool use_balance;               /* 计算现有 LQR + VMC，调用 MixBalance */
    bool released;                  /* 单周期事件，自起已成功 */
    float length_ref_m[2];           /* SR_RISE 或 released 时的左右腿长参考 */
    float normal_force_n[2];
    SelfRighting_Torque_t direct;    /* active && !use_balance 时的指令 */
} SelfRighting_Output_t;

/* 实例由调用方静态分配，字段供调试读取。 */
typedef struct {
    SelfRighting_Config_t cfg;
    SelfRighting_Phase_t phase;
    SelfRighting_Reason_t reason;
    SelfRighting_Route_t route[2];
    bool configured, angle_valid, support, support_tried, tilted, damp_pitch;
    bool done[2], load_valid, contact[2], offset_valid;
    unsigned retries;
    float t, total;                  /* 阶段和整次恢复的计时，单位秒。 */
    float quiet, cooldown;          /* 触发确认和重复触发屏蔽，单位秒。 */
    float best, no_progress;        /* 当前阶段的最小误差和停滞时间。 */
    float forward[2], flip_sign, previous_theta[2], theta[2];
    float starts[2][2], park[2][2], start_angle[2], goal_angle[2], duration[2];
    float start_length[2], grow[2], route_length, roll_target[2], roll_duration;
    float ref_q[4], current_q[4];   /* 统一髋方向的连续参考角和反馈角。 */
    float last_vz[2], fn[2];        /* 上拍轮心竖直速度与滤波后的轮载。 */
    float contact_time[2];          /* 轮载连续越过接地或离地阈值的时间。 */
    float blocked_up[3], blocked_q[4];
    float capture_length[2];
    SelfRighting_Torque_t capture_prev, offset;
} SelfRighting_t;

/* 填入 H723 腿长与时间默认值。支撑点、髋偏移、角度/力矩上限须实车配置。 */
void SelfRighting_DefaultConfig(SelfRighting_Config_t *cfg);
bool SelfRighting_Init(SelfRighting_t *recovery, const SelfRighting_Config_t *cfg);
void SelfRighting_Reset(SelfRighting_t *recovery);
/* 每拍先更新 VMC，再更新自起。返回值指明直接输出还是交给平衡控制。 */
SelfRighting_Output_t SelfRighting_Update(SelfRighting_t *recovery,
                                         const SelfRighting_Input_t *input, float dt);
/* 更新自起状态后调用。平衡输入已扣除气弹簧力矩，符号与实发力矩一致。 */
SelfRighting_Torque_t SelfRighting_MixBalance(SelfRighting_t *recovery,
                                             const SelfRighting_Torque_t *balance,
                                             const SelfRighting_Torque_t *applied, float dt);

#endif
