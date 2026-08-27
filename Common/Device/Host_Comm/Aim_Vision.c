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
 * @brief  视觉接收数据解析 (Decode)
 * @param  rx_buf:    串口接收到的原始数据缓冲区
 * @param  recv_data: 解析后存储的结构体指针
 * @retval true:      解析成功 / false: 校验失败
 */
bool Vision_Decode(uint8_t *rx_buf, Vision_Recv_t *recv_data)
{
    recv_data->offline.last_feed_tick = HAL_GetTick();
    if (rx_buf == NULL || recv_data == NULL) return false;
    // 校验帧头和帧尾
    if (rx_buf[0] != VISION_SOF || rx_buf[VISION_RECV_LEN - 1] != VISION_EOF) {
        return false;
    }
    Float_Byte_t f_cvt;
    // 解析 Pitch
    memcpy(f_cvt.buf, &rx_buf[1], 4);
    recv_data->pitch = f_cvt.f;
    // 解析 Yaw
    memcpy(f_cvt.buf, &rx_buf[5], 4);
    recv_data->yaw = f_cvt.f;
    // 解析 状态位
    recv_data->target_found = (rx_buf[9] & 0x10) >> 4;
    recv_data->fire_command = (rx_buf[9] & 0x08) >> 3;
    recv_data->state        = (rx_buf[9] & 0x07);
    // 解析 Pitch 规划值
    memcpy(f_cvt.buf, &rx_buf[10], 4);
    recv_data->pitch_plan = f_cvt.f * DEG2RAD;
    // 解析 Yaw 规划值
    memcpy(f_cvt.buf, &rx_buf[14], 4);
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
    // Yaw 角速度
    f_cvt.f = send_data->yaw_omega;
    memcpy(&tx_buf[11], f_cvt.buf, 4);
    // Pitch 角速度
    f_cvt.f = send_data->pitch_omega;
    memcpy(&tx_buf[15], f_cvt.buf, 4);
    // 帧尾
    tx_buf[19] = VISION_EOF;
}
