/**
 * @file SBUS.c
 * @brief  SBUS 协议解析源文件
 * @version 1.0
 * @date 2026-09-11
 * @author CaoKangqi
 */

#include "SBUS.h"
#include "hal.h"

static void SBUS_ParseChannels(uint8_t *raw, int16_t *ch);

/**
 * @brief SBUS 接收数据解算
 * @param Data 25 字节原始输入缓存数据指针
 * @param device_ptr 解算结果存放的应用层目标结构体指针 (SBUS_Typedef*)
 * @param size 本次接收到的字节数
 */
void SBUS_Resolved(uint8_t* Data, void *device_ptr, uint16_t size)
{
    SBUS_Typedef *SBUS = device_ptr;

    SBUS->offline.last_feed_tick = HAL_GetTick();

    // 长度校验：SBUS 固定 25 字节
    if (size < 25) {
        return;
    }
    // 帧头校验
    if (Data[0] != 0x0F) {
        return;
    }
    // 帧尾校验
    if (Data[24] != 0x00) {
        return;
    }
    // 解析 16 通道
    SBUS_ParseChannels(Data, SBUS->Remote.CH);
    // 死区处理
    for (uint8_t i = 0; i < 16; i++) {
        if (SBUS->Remote.CH[i] <= 10 && SBUS->Remote.CH[i] >= -10) {
            SBUS->Remote.CH[i] = 0;
        }
    }
    // 提取标志位
    SBUS->flags      = Data[23];
    SBUS->frame_lost = (Data[23] >> 2) & 0x01;
    SBUS->failsafe   = (Data[23] >> 3) & 0x01;
}

/**
 * @brief 解析 SBUS 25 字节帧中的 16 个 11bit 通道
 */
static void SBUS_ParseChannels(uint8_t *raw, int16_t *ch)
{
    ch[0]  = (int16_t)(((uint16_t)raw[1]        | ((uint16_t)raw[2]  << 8)) & 0x07FF) - 1024;
    ch[1]  = (int16_t)(((uint16_t)raw[2]  >> 3  | ((uint16_t)raw[3]  << 5)) & 0x07FF) - 1024;
    ch[2]  = (int16_t)(((uint16_t)raw[3]  >> 6  | ((uint16_t)raw[4]  << 2) | ((uint16_t)raw[5] << 10)) & 0x07FF) - 1024;
    ch[3]  = (int16_t)(((uint16_t)raw[5]  >> 1  | ((uint16_t)raw[6]  << 7)) & 0x07FF) - 1024;
    ch[4]  = (int16_t)(((uint16_t)raw[6]  >> 4  | ((uint16_t)raw[7]  << 4)) & 0x07FF) - 1024;
    ch[5]  = (int16_t)(((uint16_t)raw[7]  >> 7  | ((uint16_t)raw[8]  << 1) | ((uint16_t)raw[9] << 9)) & 0x07FF) - 1024;
    ch[6]  = (int16_t)(((uint16_t)raw[9]  >> 2  | ((uint16_t)raw[10] << 6)) & 0x07FF) - 1024;
    ch[7]  = (int16_t)(((uint16_t)raw[10] >> 5  | ((uint16_t)raw[11] << 3)) & 0x07FF) - 1024;

    ch[8]  = (int16_t)(((uint16_t)raw[12]       | ((uint16_t)raw[13] << 8)) & 0x07FF) - 1024;
    ch[9]  = (int16_t)(((uint16_t)raw[13] >> 3  | ((uint16_t)raw[14] << 5)) & 0x07FF) - 1024;
    ch[10] = (int16_t)(((uint16_t)raw[14] >> 6  | ((uint16_t)raw[15] << 2) | ((uint16_t)raw[16] << 10)) & 0x07FF) - 1024;
    ch[11] = (int16_t)(((uint16_t)raw[16] >> 1  | ((uint16_t)raw[17] << 7)) & 0x07FF) - 1024;
    ch[12] = (int16_t)(((uint16_t)raw[17] >> 4  | ((uint16_t)raw[18] << 4)) & 0x07FF) - 1024;
    ch[13] = (int16_t)(((uint16_t)raw[18] >> 7  | ((uint16_t)raw[19] << 1) | ((uint16_t)raw[20] << 9)) & 0x07FF) - 1024;
    ch[14] = (int16_t)(((uint16_t)raw[20] >> 2  | ((uint16_t)raw[21] << 6)) & 0x07FF) - 1024;
    ch[15] = (int16_t)(((uint16_t)raw[21] >> 5  | ((uint16_t)raw[22] << 3)) & 0x07FF) - 1024;
}

/**
 * @brief 获取指定通道的开关档位状态
 * @param sbus SBUS 结构体指针
 * @param ch 通道号 (0 ~ 15)
 * @return SBUS_SwitchState_Env 开关状态
 */
SBUS_SwitchState_Env SBUS_GetSwitchState(SBUS_Typedef *sbus, uint8_t ch)
{
    if (sbus == NULL || ch > 15) {
        return SBUS_SW_ERROR;
    }

    int16_t val = sbus->Remote.CH[ch];
    if (val > 500)  return SBUS_SW_DOWN;
    if (val < -500) return SBUS_SW_UP;
    return SBUS_SW_CEN;
}