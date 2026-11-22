//
// Created by CaoKangqi on 2026/6/19.
//
#include "Aim_Vision.h"
#include <string.h>

#include "All_define.h"
#include "hal.h"

// 使用共用体处理浮点数与字节的转换
typedef union {
    float f;
    uint8_t buf[4];
} Float_Byte_t;

/**
 * @brief  视觉接收数据解析 (Decode) —— 支持包头包尾环形错位自动拼接
 * @param  rx_buf:    串口接收到的原始数据缓冲区（长度必须为 VISION_RECV_LEN）
 * @param  recv_data: 解析后存储的结构体指针
 * @retval true:      解析成功 / false: 校验失败或找不到合法帧边界
 */
bool Vision_Decode(uint8_t *rx_buf, Vision_Recv_t *recv_data)
{
    if (rx_buf == NULL || recv_data == NULL) return false;
    recv_data->offline.last_feed_tick = HAL_GetTick();

    const uint8_t  SOF = VISION_SOF;
    const uint8_t  EOF = VISION_EOF;
    const uint16_t LEN = VISION_RECV_LEN;

    uint8_t *p = rx_buf;                    // 最终指向用于解析的缓冲区
    uint8_t  temp_buf[VISION_RECV_LEN];     // 旋转拼接用的临时空间
    /* ---------- 1. 先检查是否已经是标准对齐帧 ---------- */
    if (rx_buf[0] == SOF && rx_buf[LEN - 1] == EOF) {
        // 已经对齐，p 继续指向 rx_buf，无需拼接
    }
    else {
        /* ---------- 2. 环形查找 "DC 后面紧跟 CD" 的边界 ---------- */
        bool frame_found = false;
        int  offset = 0;    // 真正的包头 SOF 在 rx_buf 中的索引

        for (int i = 0; i < LEN; i++) {
            int next = (i + 1) % LEN;   // 环形后一字节
            // 必须成对判断：当前是包尾 DC，下一位是包头 CD
            if (rx_buf[i] == EOF && rx_buf[next] == SOF) {
                offset = next;          // 包头所在位置
                frame_found = true;
                break;
            }
        }

        if (!frame_found) {
            return false;   // 找不到合法的包头包尾边界，直接丢弃
        }

        /* ---------- 3. 按 offset 旋转，重新拼接成标准帧 ---------- */
        for (int i = 0; i < LEN; i++) {
            temp_buf[i] = rx_buf[(offset + i) % LEN];
        }
        p = temp_buf;       // 后续统一用 p 解析
        /* 二次确认拼接后的头尾 */
        if (p[0] != SOF || p[LEN - 1] != EOF) {
            return false;
        }
    }
    /* ---------- 4. 统一解析（无论 p 指向 rx_buf 还是 temp_buf） ---------- */
    Float_Byte_t f_cvt;
    // 解析 Pitch
    memcpy(f_cvt.buf, &p[1], 4);
    recv_data->pitch = f_cvt.f;
    // 解析 Yaw
    memcpy(f_cvt.buf, &p[5], 4);
    recv_data->yaw = f_cvt.f;
    // 解析状态位
    recv_data->target_found = (p[9] & 0x10) >> 4;
    recv_data->fire_command = (p[9] & 0x08) >> 3;
    recv_data->state        = (p[9] & 0x07);
    // 解析 Pitch 速度前馈
    memcpy(f_cvt.buf, &p[10], 4);
    recv_data->pitch_plan = f_cvt.f * DEG2RAD;
    // 解析 Yaw 速度前馈
    memcpy(f_cvt.buf, &p[14], 4);
    recv_data->yaw_plan = f_cvt.f * DEG2RAD;
    return true;
}

/**
 * @brief  视觉发送数据打包 (Encode)
 * @param  send_data: 需要发送的数据结构体指针
 * @param  tx_buf:    打包后存放的发送缓冲区
 */
void Vision_Encode(Vision_Send_t *send_data, uint8_t *tx_buf)
{
    if (send_data == NULL || tx_buf == NULL) return;

    Float_Byte_t f_cvt;
    // 帧头
    tx_buf[0] = VISION_SOF;
    // Pitch
    f_cvt.f = send_data->pitch;
    memcpy(&tx_buf[1], f_cvt.buf, 4);
    // Yaw
    f_cvt.f = send_data->yaw;
    memcpy(&tx_buf[5], f_cvt.buf, 4);
    // 模式
    tx_buf[9]  = send_data->mode;
    // 弹速
    tx_buf[10] = send_data->bullet_speed;
    // Pitch 角速度
    f_cvt.f = send_data->pitch_omega;
    memcpy(&tx_buf[11], f_cvt.buf, 4);
    // Yaw 角速度
    f_cvt.f = send_data->yaw_omega;
    memcpy(&tx_buf[15], f_cvt.buf, 4);
    // 帧尾
    tx_buf[19] = VISION_EOF;
}

/**
 * @brief  哨兵视觉接收数据解析 (Decode) —— 34 字节支持包头包尾错位自动对齐
 * @param  rx_buf:    串口 DMA/中断接收到的原始缓冲区 (长度必须为 SENTINEL_VISION_LEN)
 * @param  recv_data: 解析后存储的哨兵数据结构体指针
 * @retval true:      解析成功 / false: 校验失败
 */
bool Sentinel_Vision_Decode(uint8_t *rx_buf, Sentinel_Vision_Recv_t *recv_data)
{
    if (rx_buf == NULL || recv_data == NULL) return false;

    const uint8_t  SOF = VISION_SOF;
    const uint8_t  EOF = VISION_EOF;
    const uint16_t LEN = SENTINEL_VISION_LEN; // 34 字节

    uint8_t *p = rx_buf;
    uint8_t  temp_buf[SENTINEL_VISION_LEN];

    /* ---------- 1. 检查是否已经是标准对齐帧 ---------- */
    if (rx_buf[0] == SOF && rx_buf[LEN - 1] == EOF) {
        // 已对齐，无需移动
    }
    else {
        /* ---------- 2. 环形查找 "DC 紧跟 CD" 边界 ---------- */
        bool frame_found = false;
        int  offset = 0;

        for (int i = 0; i < LEN; i++) {
            int next = (i + 1) % LEN;
            if (rx_buf[i] == EOF && rx_buf[next] == SOF) {
                offset = next;
                frame_found = true;
                break;
            }
        }

        if (!frame_found) {
            return false;
        }

        /* ---------- 3. 旋转对齐 ---------- */
        for (int i = 0; i < LEN; i++) {
            temp_buf[i] = rx_buf[(offset + i) % LEN];
        }
        p = temp_buf;

        if (p[0] != SOF || p[LEN - 1] != EOF) {
            return false;
        }
    }

    /* ---------- 4. 零拷贝统一解析 (通过 Frame 协议帧指针映射) ---------- */
    // 注意：这里必须是 Sentinel_Vision_Frame_t！
    Sentinel_Vision_Frame_t *frame = (Sentinel_Vision_Frame_t *)p;

    // 1. 云台瞄准数据
    recv_data->pitch       = frame->pitch;
    recv_data->yaw         = frame->yaw;
    recv_data->pitch_vel   = frame->pitch_vel * DEG2RAD;
    recv_data->yaw_vel     = frame->yaw_vel * DEG2RAD;

    // 2. 哨兵底盘运动导航数据
    recv_data->vx          = frame->vx;
    recv_data->vy          = frame->vy;
    recv_data->wz          = frame->wz;

    // 3. 状态与控制指令（左右字段名完全统一）
    recv_data->ShootBool   = frame->ShootBool;
    recv_data->Target      = frame->Target;
    recv_data->mode        = frame->mode;
    recv_data->form        = frame->form;
    recv_data->zhuan       = frame->zhuan;
    recv_data->vision_mode = frame->vision_mode;

    /* ---------- 5. 解析成功，刷新心跳喂狗 ---------- */
    recv_data->offline.last_feed_tick = HAL_GetTick();

    return true;
}

/**
 * @brief  哨兵视觉发送数据打包 (Encode) —— 严格对齐为 36 字节
 * @param  send_data: 上层控制与裁判系统数据指针
 * @param  tx_buf:    发送缓冲区 (长度必须 >= SENTINEL_VISION_SEND_LEN)
 */
void Sentinel_Vision_Encode(const Sentinel_Vision_Send_t *send_data, uint8_t *tx_buf)
{
    if (send_data == NULL || tx_buf == NULL) return;

    // 直接映射到紧凑帧指针
    Sentinel_Vision_Tx_Frame_t *frame = (Sentinel_Vision_Tx_Frame_t *)tx_buf;

    // 1. 包头包尾
    frame->head = VISION_SOF; // 0xCD
    frame->end  = VISION_EOF; // 0xDC

    // 2. 云台物理量
    frame->pitch      = send_data->pitch;
    frame->yaw        = send_data->yaw;          // 底盘 Yaw
    frame->gimbal_yaw = send_data->gimbal_yaw;   // 云台 Yaw (新增)
    frame->pitch_vel  = send_data->pitch_vel;
    frame->yaw_vel    = send_data->yaw_vel;

    // 3. 射击与模式（自动完成 高6位颜色 + 低2位模式 拼接）
    frame->mode         = ((send_data->color & 0x3F) << 2) | (send_data->vision_state & 0x03);
    frame->bullet_speed = send_data->bullet_speed;

    // 4. 裁判系统与战局数据
    frame->game_progress     = send_data->game_progress;
    frame->stage_remain_time = send_data->stage_remain_time;
    frame->current_hp        = send_data->current_hp;
    frame->ally_outpost_hp   = send_data->ally_outpost_hp;
    frame->state             = send_data->state;
    frame->energy_state      = send_data->energy_state;
    frame->bullets           = send_data->bullets;
    frame->judge             = send_data->judge;
}

