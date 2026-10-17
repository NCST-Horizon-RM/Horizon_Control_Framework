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
    CHASSIS_CMD_SAFE = 0,     // 安全锁死
    CHASSIS_CMD_WORK      // 裁判系统全自动控制
} Chassis_Mode_e;

typedef struct {
    Chassis_Mode_e mode;
    int32_t target_yaw;
} Chassis_Cmd_t;

// 云台控制指令
typedef enum {
    GIMBAL_CMD_SAFE = 0,     // 安全锁死
    GIMBAL_CMD_WORK
} Gimbal_Mode_e;

typedef struct {
    Gimbal_Mode_e mode;
    int16_t target_arm;
} Gimbal_Cmd_t;

// 发射机构控制指令
typedef enum {
    SHOOT_CMD_SAFE = 0,      // 安全锁死，拉簧电机停转
    SHOOT_CMD_REMOTE,         // 拉簧电机运行
    SHOOT_CMD_AUTO
} Shoot_Mode_e;

typedef struct {
    Shoot_Mode_e mode;
    int32_t target_spring;
} Shoot_Cmd_t;

typedef enum {
    DOOR_CMD_SAFE = 0,      // 安全锁死
    DOOR_CMD_GAMEOVER,
    DOOR_CMD_GAMEING,
    DOOR_CMD_ROUND1,         // 闸门第一次开启
    DOOR_CMD_ROUND2           // 闸门第二次开启
} Door_Mode_e;

typedef struct {
    Door_Mode_e mode;
    uint8_t open_cnt;
} Door_Cmd_t;

typedef enum {
    VISION_CMD_SAFE = 0,      // 安全锁死
    VISION_CMD_ONLINE,         // 离线
    VISION_CMD_OFFLINE          // 在线
} Vision_Mode_e;

typedef struct {
    Vision_Mode_e mode;
    uint8_t OriginData[16];
    struct Data
    {
        uint8_t Head_frame;
        uint8_t End_frame;
        int16_t x0;
        int16_t x1;
        int16_t y1;
        int16_t x2;
        int16_t y2;
        uint16_t OffCounter; // 在线检测
        uint8_t  isOnline;
    } Data;
} Vision_Rx_Cmd_t;

typedef struct {
    uint8_t data[4];
    uint8_t Head_frame;
    uint16_t VisionState;
    uint8_t End_frame;
} Vision_Tx_Cmd_t;

extern Chassis_Cmd_t chassis_cmd;
extern Gimbal_Cmd_t  gimbal_cmd;
extern Shoot_Cmd_t   shoot_cmd;
extern Door_Cmd_t    door_cmd;
extern Vision_Rx_Cmd_t vision_rx_cmd;
extern Vision_Tx_Cmd_t vision_tx_cmd;

void Robot_Cmd_Init(void);
void Robot_Cmd_Update(void);
void Vision_CAN_Rx_Callback(void *instance, uint8_t *data);

#endif //F4_FRAMEWORK_ROBOT_CMD_H