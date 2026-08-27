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

// ==================== 双板通信协议 ====================
#pragma pack(1)
// 云台 -> 底盘
typedef union {
    struct {
        int16_t vx:11;		//平移速度
        int16_t vy:11;		//前进速度
        int16_t vr:11;		//旋转速度
        uint16_t key_q:1;
        uint16_t key_e:1;
        uint16_t key_v:1;
        uint16_t key_shift:1;
        uint16_t key_ctrl:1;
        uint16_t romoteOnLine	:2;	//遥控是否在线

        uint16_t S1:2;
        uint16_t S2:2;

        int16_t pitch:5;			//陀螺仪
        uint16_t fire_wheel:1;//发射是否离线
        uint16_t gimbal_lixian:1;//云台是否离线

        uint16_t vision_look:1;//视觉是否识别到目标
        uint16_t vision:1;//是否开启视觉
        uint32_t surplus_count:9;//预计剩余弹丸数量
    } bits;
    uint8_t buf[8];
} Protocol_Rx_t;

// 底盘 -> 云台
typedef union {
    struct {
        uint16_t heat_last:10	;//热量上限//最大是1024
        uint16_t self_color:1;//只能是0/1
        uint16_t cooling:7;//冷却
        uint8_t level:4;//等级
        uint8_t initial_s;//初速上限
        uint16_t robot_HP:9;
        uint16_t heat_large:9;//裁判系统发来的当前热量
    } bits;
    uint8_t buf[8];
} Protocol_Tx_t;
#pragma pack()

extern Protocol_Rx_t b2b_rx_data;
void DualBoard_CAN_Rx_Callback(void *device_ptr, uint8_t *data);

#endif //F4_FRAMEWORK_ROBOT_CMD_H