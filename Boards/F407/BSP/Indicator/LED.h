//
// Created by CaoKangqi on 2026/7/7.
//

#ifndef C_BOARD_FRAMEWORK_LED_H
#define C_BOARD_FRAMEWORK_LED_H

#include <stdint.h>
#include "BSP_TIM.h"

typedef enum {
    LED_MODE_STATIC = 0,   // 常亮 / 静态底色
    LED_MODE_BREATHING,    // 自动呼吸模式
    LED_MODE_BLINK         // 自动闪烁警报模式
} LED_Mode_e;

void LED_Init(void);
void LED_SetPixel(uint8_t r, uint8_t g, uint8_t b);
void LED_SetMode_Static(uint8_t r, uint8_t g, uint8_t b);
void LED_SetMode_Breathing(uint8_t r, uint8_t g, uint8_t b, float period_s);
void LED_SetMode_Blink(uint8_t r, uint8_t g, uint8_t b, float interval_s);
void LED_Ticks(void);

#endif //C_BOARD_FRAMEWORK_LED_H
