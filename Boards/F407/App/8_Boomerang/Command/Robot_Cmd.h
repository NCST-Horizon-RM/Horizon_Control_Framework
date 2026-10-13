//
// Created by CaoKangqi on 2026/6/23.
// 模块功能：总控中心逻辑（纯净版）- 指令集与接口
//
#ifndef F4_FRAMEWORK_ROBOT_CMD_H
#define F4_FRAMEWORK_ROBOT_CMD_H

#include <stdint.h>
#include <stdbool.h>



// 云台控制指令
typedef enum {
    GIMBAL_CMD_SAFE = 0,     // 安全锁死
    GIMBAL_CMD_MANUAL,       // 遥控器控制
    GIMBAL_CMD_AUTO_AIM      // 裁判系统全自动控制
} Gimbal_Mode_e;

typedef struct {
    Gimbal_Mode_e mode;
    int16_t target_arm;
    int32_t target_yaw;
} Gimbal_Cmd_t;

// 发射机构控制指令
typedef enum {
    SHOOT_CMD_SAFE = 0,      // 安全锁死，拉簧电机停转
    SHOOT_CMD_WORK,         // 拉簧电机运行
    SHOOT_CMD_FIRE           // 允许开火状态
} Shoot_Mode_e;

typedef struct {
    Shoot_Mode_e mode;
    int32_t target_spring;
} Shoot_Cmd_t;

extern Gimbal_Cmd_t  gimbal_cmd;
extern Shoot_Cmd_t   shoot_cmd;

void Robot_Cmd_Init(void);
void Robot_Cmd_Update(void);

void DualBoard_CAN_Rx_Callback(void *device_ptr, uint8_t *data);

#endif //F4_FRAMEWORK_ROBOT_CMD_H