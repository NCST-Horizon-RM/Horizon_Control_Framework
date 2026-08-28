//
// Created by CaoKangqi on 2026/6/23.
// 模块功能：总控中心逻辑（纯净版）- 指令集与接口
//
#ifndef F4_FRAMEWORK_ROBOT_CMD_H
#define F4_FRAMEWORK_ROBOT_CMD_H

#include <stdint.h>
#include <stdbool.h>

// 底盘控制指令
typedef enum {
    CHASSIS_CMD_SAFE = 0,    // 安全锁死，无输出
    CHASSIS_CMD_FOLLOW,      // 底盘跟随云台
    CHASSIS_CMD_FREE,        // 底盘与云台分离
    CHASSIS_CMD_SPIN         // 小陀螺模式
} Chassis_Mode_e;

typedef struct {
    Chassis_Mode_e mode;
    float target_vx;         // 目标 X 轴平移速度 (m/s)
    float target_vy;         // 目标 Y 轴平移速度 (m/s)
    float target_vw;         // 目标自旋角速度 (rad/s)
    float offset_angle;      // 云台与底盘的相对夹角
    bool is_cap_on;          // 是否开启超电
} Chassis_Cmd_t;

// 云台控制指令
typedef enum {
    GIMBAL_CMD_SAFE = 0,     // 安全锁死
    GIMBAL_CMD_MANUAL,       // 键鼠/遥控器控制
    GIMBAL_CMD_AUTO_AIM      // 视觉自瞄控制
} Gimbal_Mode_e;

typedef struct {
    Gimbal_Mode_e mode;
    float target_pitch;      // 目标 Pitch 角度
    float target_pitch_rate; // 目标 Pitch 角速度
    float target_yaw;        // 目标 Yaw 角度
    float target_yaw_rate;   // 目标 Yaw 角速度
} Gimbal_Cmd_t;

// 发射机构控制指令
typedef enum {
    SHOOT_CMD_SAFE = 0,      // 安全锁死，摩擦轮停转，拨弹停止
    SHOOT_CMD_READY,         // 摩擦轮停转，拨盘目标固定在当前位置
    SHOOT_CMD_RUN,           // 摩擦轮启动
    SHOOT_CMD_FIRE           // 允许开火状态
} Shoot_Mode_e;

typedef struct {
    Shoot_Mode_e mode;
    bool trigger_single;     // 单发
    bool trigger_auto;       // 连发
    float heat_max;
    float heat_now;
    float cool;
    uint8_t last_fn1;
    uint8_t last_S1 ;
    uint8_t bullet_speed;    // 目标射速
} Shoot_Cmd_t;

extern Chassis_Cmd_t chassis_cmd;
extern Gimbal_Cmd_t  gimbal_cmd;
extern Shoot_Cmd_t   shoot_cmd;

void Robot_Cmd_Init(void);
void Robot_Cmd_Update(void);

void DualBoard_CAN_Rx_Callback(void *device_ptr, uint8_t *data);

#endif //F4_FRAMEWORK_ROBOT_CMD_H