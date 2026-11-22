//
// Created by CaoKangqi on 2026/6/19.
//

#ifndef HORIZON_AIM_VISION_H
#define HORIZON_AIM_VISION_H

#include <stdint.h>
#include <stdbool.h>
#include "Offline_Detector.h"
#include "usart.h"
// 协议帧头帧尾
#define VISION_SOF      0xCD  // 帧头
#define VISION_EOF      0xDC  // 帧尾

// 数据包长度
#define VISION_RECV_LEN 19
#define VISION_SEND_LEN 20
// 帧总长度自动计算为 34
#define SENTINEL_VISION_LEN sizeof(Sentinel_Vision_Frame_t)
// 自动计算发送包长度 (严格等于 32 字节)
#define SENTINEL_VISION_SEND_LEN sizeof(Sentinel_Vision_Tx_Frame_t)

// 离线检测超时时间 (单位: ms)
#define VISION_OFFLINE_TIME 500

// ---------------- 接收结构体 (视觉 -> 电控) ----------------
typedef struct {
    Offline_Check_t offline;
    float pitch;        // 视觉计算的Pitch绝对角度
    float yaw;          // 视觉计算的Yaw绝对角度
    float pitch_plan;   // Pitch 预测/规划角度
    float yaw_plan;     // Yaw 预测/规划角度

    bool  target_found; // 是否识别到目标
    bool  fire_command; // 是否允许开火
    uint8_t state;      // 视觉状态
} Vision_Recv_t;

// ---------------- 发送结构体 (电控 -> 视觉) ----------------
typedef struct {
    float pitch;        // 当前云台 Pitch 角度
    float yaw;          // 当前云台 Yaw 角度
    uint8_t mode;       // 当前模式 (0自瞄, 1小符, 2大符)
    uint8_t bullet_speed; // 当前弹速
    float pitch_omega;  // 当前云台 Pitch 角速度
    float yaw_omega;    // 当前云台 Yaw 角速度
} Vision_Send_t;

/* ========================================================================== */
/*                           哨兵视觉接收相关 (32 字节)                        */
/* ========================================================================== */
/* ---------- 1. 严格对齐的物理传输帧结构体 (34 字节) ---------- */
// 必须加 __attribute__((packed)) 保证不产生填充字节
typedef struct __attribute__((packed))
{
    uint8_t head;           // [0]  包头 0xCD
    float   pitch;          // [1..4] 目标 Pitch (角度/弧度)
    float   yaw;            // [5..8] 目标 Yaw (角度/弧度)

    // [9] 状态字节: 建议使用位域或单字节掩码提取 (共 1 字节)
    uint8_t ShootBool   : 1; // bit0: 发射许可
    uint8_t Target      : 1; // bit1: 识别到目标
    uint8_t mode        : 6; // bit2..7: 模式位

    float   pitch_vel;      // [10..13] 云台 Pitch 速度前馈
    float   yaw_vel;        // [14..17] 云台 Yaw 速度前馈
    float   vx;             // [18..21] 哨兵底盘 x 轴移动线速度 (m/s)
    float   vy;             // [22..25] 哨兵底盘 y 轴移动线速度 (m/s)
    float   wz;             // [26..29] 哨兵底盘旋转角速度 (rad/s)

    int8_t  form;           // [30] 哨兵形态/巡航状态
    int8_t  zhuan;          // [31] 小陀螺/旋转指令
    uint8_t vision_mode;    // [32] 视觉当前状态机模式
    uint8_t end;            // [33] 包尾 0xDC
} Sentinel_Vision_Frame_t;



/* ---------- 2. 哨兵主控业务数据结构体 ---------- */
typedef struct
{
    // 云台
    float pitch;
    float yaw;
    float pitch_vel;
    float yaw_vel;

    // 底盘
    float vx;
    float vy;
    float wz;

    // 状态与指令（名字与视觉协议 100% 对齐）
    uint8_t ShootBool;      // 直接叫 ShootBool
    uint8_t Target;         // 直接叫 Target
    uint8_t mode;           // 模式
    int8_t  form;           // 哨兵形态
    int8_t  zhuan;          // 小陀螺
    uint8_t vision_mode;    // 视觉模式

    // 掉线检测心跳
    Offline_Check_t offline;
} Sentinel_Vision_Recv_t;




/* ========================================================================== */
/*                           哨兵视觉发送相关 (36 字节)                        */
/* ========================================================================== */

/* ---------- 1. 严格对齐的物理传输帧结构体 (必须为 36 字节) ---------- */
typedef struct __attribute__((packed))
{
    uint8_t  head;               // [0]      帧头 0xCD
    float    pitch;              // [1..4]   当前云台 Pitch 角度
    float    yaw;                // [5..8]   当前底盘 Yaw 角度
    uint8_t  mode;               // [9]      高6位颜色: color, 低2位模式: vision_state
    uint8_t  bullet_speed;       // [10]     当前弹速 (单位: 0.1 m/s)
    float    pitch_vel;          // [11..14] 当前 Pitch 角速度
    float    yaw_vel;            // [15..18] 当前 Yaw 角速度
    uint8_t  game_progress;      // [19]     比赛阶段 (裁判系统)
    uint16_t stage_remain_time;  // [20..21] 当前阶段剩余时间 (s)
    uint16_t current_hp;         // [22..23] 哨兵当前血量
    uint16_t ally_outpost_hp;    // [24..25] 己方前哨站血量
    uint8_t  state;              // [26]     哨兵自身运行状态标志
    uint8_t  energy_state;       // [27]     能量机构/打符状态
    uint16_t bullets;            // [28..29] 剩余发弹量
    uint8_t  judge;              // [30]     裁判系统通信与校验状态
    float    gimbal_yaw;         // [31..34] 当前云台 Yaw 角度 (新增)
    uint8_t  end;                // [35]     帧尾 0xDC
} Sentinel_Vision_Tx_Frame_t;



/* ---------- 2. 哨兵主控发送业务数据结构体 (供各控制模块填写) ---------- */
typedef struct
{
    // 云台与射击状态
    float    pitch;
    float    yaw;                // 底盘 Yaw
    float    gimbal_yaw;         // 云台 Yaw (新增)
    float    pitch_vel;
    float    yaw_vel;
    uint8_t  bullet_speed;

    // 模式与敌我识别
    uint8_t  color;              // 0: 红, 1: 蓝
    uint8_t  vision_state;       // 视觉工作模式

    // 裁判系统赛场信息
    uint8_t  game_progress;      // 比赛阶段
    uint16_t stage_remain_time;  // 剩余时间
    uint16_t current_hp;         // 当前血量
    uint16_t ally_outpost_hp;    // 前哨站血量
    uint8_t  state;              // 自身状态
    uint8_t  energy_state;       // 能量状态
    uint16_t bullets;            // 剩余弹丸
    uint8_t  judge;              // 裁判系统在线状态
} Sentinel_Vision_Send_t;


// 函数声明
bool Vision_Decode(uint8_t *rx_buf, Vision_Recv_t *recv_data);
bool Sentinel_Vision_Decode(uint8_t *rx_buf, Sentinel_Vision_Recv_t *recv_data);
void Vision_Encode(Vision_Send_t *send_data, uint8_t *tx_buf);
void Vision_Monitor(Vision_Recv_t *recv_data, uint32_t sys_time_ms);
void Vision_Rx_Data(uint8_t* buffer);
// 哨兵发送打包函数
void Sentinel_Vision_Encode(const Sentinel_Vision_Send_t *send_data, uint8_t *tx_buf);


#endif //HORIZON_AIM_VISION_H
