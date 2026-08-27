//
// Created by CaoKangqi on 2026/6/25.
//

#ifndef H7_FRAMEWORK_CATAPULT_CTRL_H
#define H7_FRAMEWORK_CATAPULT_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#include "Classic_Control.h"
#include "Robot_Config.h"

// --- 状态机枚举定义 ---
typedef enum {
    CALIB_START = 0,
    CALIB_MOVING,
    CALIB_DONE,
    CALIB_NORMAL
} Calib_State_e;

typedef enum {
    PULL_STATE_NORMAL = 0,
    PULL_STATE_TRIGGERED,
    PULL_STATE_RESET,
    PULL_STATE_STOPPED
} Pull_State_e;

// --- 拨盘电机控制数据 ---
typedef struct {
    bool     is_init;
    int32_t  target_pos_cnt;
    float    smooth_ref;
} DM4310_Feeder_t;

// --- 核心控制块 ---
typedef struct {
    // 内部状态机与状态变量
    Calib_State_e   calib_state;
    Pull_State_e    pull_state;
    DM4310_Feeder_t feed_motor;

    float    zero_offset_angle;
    float    mid_offset_angle;
    float    pull_timer_sec;
    uint8_t  last_switch_v;
    uint8_t  last_cmd_trigger;

    // 输出缓存 (统一发包用)
    int16_t  out_pull_curr;
    int16_t  out_yaw_curr;
    float    out_feed_torque;
    uint16_t out_trigger_pwm;

    // 专属 PID 控制器
    PID_t PID_Feed_P;
    PID_t PID_Feed_S;
    PID_t PID_Yaw_P;
    PID_t PID_Yaw_S;
    PID_t PID_Pull_P;
    PID_t PID_Pull_S;
} Shoot_Ctrl_Block_t;

uint8_t Shoot_Control_Init(void);
void Shoot_Control_Task(const Shoot_Motor_Group_t *s_motor,
                        const Gimbal_Motor_Group_t *g_motor,
                        float dt);

#endif //H7_FRAMEWORK_CATAPULT_CTRL_H
