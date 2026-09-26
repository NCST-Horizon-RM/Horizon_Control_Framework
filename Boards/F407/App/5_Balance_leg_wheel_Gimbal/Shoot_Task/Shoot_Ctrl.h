//
// Created by R0602 on 26-7-10.
//

#ifndef SHOOT_CTRL_H
#define SHOOT_CTRL_H
#include <stdint.h>
#include "Robot_Config.h"
#include "Classic_Control.h"
#include "Referee.h"

typedef struct {
    float base;              // 动态基准线
    float last_val;          // 记录上一次的转速，用于算斜率
    float max_drop_in_round; // 记录单次触发过程中的最大跌落深度
    bool  armed;             // 触发状态
    uint32_t cnt;            // 计弹总数
    uint32_t last_cnt;
    float t_out_timer;       // 超时倒计时（秒）
    float cool_down_timer;   // 冷却倒计时（秒）
    bool  init;              // 初始化标志
} ShootDet_t;

typedef enum {
    FEEDER_STOP = 0,
    FEEDER_SINGLE,
    FEEDER_BURST
} FeederMode_e;
typedef struct {
    FeederMode_e mode;
    int32_t target_pos_cnt;      // 目标弹槽索引 (绝对位置)
    uint8_t last_trigger_state;  // 记录上一次触发状态 (用于单发边缘检测)
    float   target_freq;         //目标弹频
} Feeder_t;

typedef struct {
    float motor_ratio;//电机减速比
    float feed_ratio;//齿轮减速比
    float slot_num;//弹槽数
    bool use_smoothing ;//是否平滑模式的标志位
    float Counts_Shoot;//电机减速比*齿轮减速比/弹槽数
    float Lfire_speed;
    float Rfire_speed;
    int8_t dir_sign ;

    PID_t Bmotor_P;
    PID_t Bmotor_S;
    PID_t Lfire_S;
    PID_t Rfire_S;

    ShootDet_t Det_Count;
    Feeder_t Feeder_Count;
} Shoot_Ctrl_Block_t;

uint8_t Shoot_Control_Init(void);
void Shoot_Control_Task(const Shoot_Motor_Group_t *g_motor, float dt);
void Smooth_Shoot_Control(void);

//射击检测初始参数
#define BASE_DT          0.005f   // 200Hz 对应的基准 dt (5ms)
#define K_UP_BASE        0.673f   // 200Hz 对应的上升系数
#define K_DN_BASE        0.142f   // 200Hz 对应的下降系数
#define TH_FIRE          200.0f   // 触发阈值
#define TH_FIRE_MAX      1200.0f  // 最大触发阈值
#define TH_RST_SAFE      100.0f   // 复位阈值
#define RELATIVE_RECOVER 0.25f    // 回升比例
// 转换为时域物理量
#define MIN_SLOPE_PER_SEC (80.0f / BASE_DT)      // 最小斜率
#define TIMEOUT_DURATION  (14.0f * BASE_DT)      // 超时时间
#define COOLDOWN_DURATION (2.0f * BASE_DT)       // 冷却时间
bool Update_Shoot_Det_Dynamic(float speed1, float speed2, float dt, ShootDet_t *det);
float Heat_Freq_Ctrl(float kp, float heat_max,float heat_now,float cool, bool is_shot, float dt, float max_freq, float remain_heat);

#endif //SHOOT_CTRL_H
