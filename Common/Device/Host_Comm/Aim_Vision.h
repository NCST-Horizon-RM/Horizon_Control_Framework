//
// Created by CaoKangqi on 2026/6/19.
//

#ifndef HORIZON_AIM_VISION_H
#define HORIZON_AIM_VISION_H

#include <stdint.h>
#include <stdbool.h>
#include "Offline_Detector.h"

// 协议帧头帧尾
#define VISION_SOF      0xCD  // 帧头
#define VISION_EOF      0xDC  // 帧尾

// 数据包长度
#define VISION_RECV_LEN 19
#define VISION_SEND_LEN 20

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
    float pitch_omega;  // 当前云台 Pitch 角速度
    float yaw_omega;    // 当前云台 Yaw 角速度

    uint8_t mode;       // 当前模式 (0自瞄, 1小符, 2大符)
    uint8_t bullet_speed; // 当前弹速
} Vision_Send_t;

// 函数声明
bool Vision_Decode(uint8_t *rx_buf, Vision_Recv_t *recv_data);
void Vision_Encode(Vision_Send_t *send_data, uint8_t *tx_buf);
void Vision_Monitor(Vision_Recv_t *recv_data, uint32_t sys_time_ms);

#endif //HORIZON_AIM_VISION_H
