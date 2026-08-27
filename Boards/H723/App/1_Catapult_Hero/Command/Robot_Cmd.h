//
// Created by CaoKangqi on 2026/6/23.
//
#ifndef H7_FRAMEWORK_ROBOT_CMD_H
#define H7_FRAMEWORK_ROBOT_CMD_H

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
} Chassis_Cmd_t;

// 台控制指令
typedef enum {
    GIMBAL_CMD_SAFE = 0,     // 安全锁死
    GIMBAL_CMD_MANUAL,       // 键鼠/遥控器控制
    GIMBAL_CMD_AUTO_AIM      // 视觉自瞄控制
} Gimbal_Mode_e;

typedef struct {
    Gimbal_Mode_e mode;
    float target_pitch;      // 目标 Pitch 角度
    float target_yaw;        // 目标 Yaw 角度
} Gimbal_Cmd_t;

// 发射机构控制指令
typedef enum {
    SHOOT_CMD_SAFE = 0,      // 安全锁死，摩擦轮停转，拨弹停止
    SHOOT_CMD_READY,         // 摩擦轮怠速/准备状态
    SHOOT_CMD_FIRE           // 允许开火状态
} Shoot_Mode_e;

typedef struct {
    Shoot_Mode_e mode;
    float friction_rpm;      // 摩擦轮目标转速
    bool trigger_single;     // 单发
    bool trigger_auto;       // 连发
    uint8_t bullet_speed;    // 目标射速
} Shoot_Cmd_t;

void Robot_Cmd_Init(void);
void Robot_Cmd_Update(void);

#endif //H7_FRAMEWORK_ROBOT_CMD_H