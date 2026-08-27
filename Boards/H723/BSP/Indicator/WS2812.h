//
// Created by CaoKangqi on 2026/6/19.
//

#ifndef H7_FRAMEWORK_WS2812_H
#define H7_FRAMEWORK_WS2812_H

#include <stdint.h>
#include "BSP_TIM.h"

#define WS2812_PWM_LOW      100
#define WS2812_PWM_HIGH     235

#define MAX_LED             1    // 车载 LED 灯珠总数
#define WS2812_RESET_SLOTS  20   // Reset 信号所需的低电平双缓冲周期数量

extern BSP_PWM_t ws2812_pwm;

typedef enum {
    LED_MODE_STATIC = 0,   // 常亮 / 静态底色
    LED_MODE_BREATHING,    // 自动呼吸模式
    LED_MODE_BLINK         // 自动闪烁警报模式
} LED_Mode_e;

typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} WS2812_Color_t;

/**
 * @brief 初始化灯组逻辑状态
 */
void WS2812_Init(void);

/**
 * @brief 纯设置指定灯珠的物理底色
 */
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);
void WS2812_Clear(void);

/**
 * @brief 触发双缓冲 DMA 将数据推向硬件喷射（供内部心跳或外部强制刷新使用）
 */
void WS2812_Send(void);

/**
 * @brief 【全新接口】将灯珠配置为常亮/常灭（只需触发一次，不放循环）
 */
void WS2812_SetMode_Static(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 【全新接口】将灯珠配置为后台自动呼吸（只需触发一次，不放循环）
 * @param period_s 呼吸总周期（秒）
 */
void WS2812_SetMode_Breathing(uint16_t index, uint8_t r, uint8_t g, uint8_t b, float period_s);

/**
 * @brief 【全新接口】将灯珠配置为后台自动闪烁（只需触发一次，不放循环）
 * @param interval_s 亮灭翻转的时间间隔（秒）
 */
void WS2812_SetMode_Blink(uint16_t index, uint8_t r, uint8_t g, uint8_t b, float interval_s);

/**
 * @brief 【全新接口】WS2812 状态机核心后台心跳，需要挂载在 1ms 定时器或 Task 中
 */
void WS2812_Ticks(void);

// 硬件 DMA 回调句柄保持不变
void WS2812_DMA_Handler(uint8_t half_cplt);

#endif //H7_FRAMEWORK_WS2812_H