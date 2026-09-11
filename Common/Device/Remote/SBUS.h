/**
* @file SBUS.h
 * @brief  SBUS 协议解析头文件
 * @version 1.0
 * @date 2026-09-11
 * @author CaoKangqi
 */

#ifndef HORIZON_SBUS_H
#define HORIZON_SBUS_H

#include <stdint.h>
#include <stdbool.h>

#include "Offline_Detector.h"

/**
 * @brief 开关档位状态枚举
 */
typedef enum {
  SBUS_SW_UP    = 1,  /**< 开关上档 */
  SBUS_SW_DOWN  = 2,  /**< 开关下档 */
  SBUS_SW_CEN   = 3,  /**< 开关中档 */
  SBUS_SW_ERROR = 4   /**< 通道越界或指针异常 */
} SBUS_SwitchState_Env;

/**
 * @brief 应用层使用的解析结果结构体
 */
typedef struct {
  Offline_Check_t offline;

  /**
   * @brief 遥控器通道数据解算
   */
  struct {
    int16_t CH[16];     /**< 16 通道解算值 (-1024 ~ 1024, 中点为 0) */
  } Remote;

  uint8_t flags;          /**< 原始 flags 字节 (bit0:CH17, bit1:CH18, bit2:FrameLost, bit3:FailSafe) */
  uint8_t failsafe;       /**< 失控保护标志 (1:接收机进入失控保护) */
  uint8_t frame_lost;     /**< 丢帧标志 (1:上一帧丢失) */
} SBUS_Typedef;

void SBUS_Resolved(uint8_t* Data, void *device_ptr, uint16_t size);

SBUS_SwitchState_Env SBUS_GetSwitchState(SBUS_Typedef *sbus, uint8_t ch);

#endif // HORIZON_SBUS_H