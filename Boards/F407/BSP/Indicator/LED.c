//
// Created by CaoKangqi on 2026/7/7.
//
#include "LED.h"

extern TIM_HandleTypeDef htim5;

// BLUE: TIM5_CH1 | GREEN: TIM5_CH2 | RED: TIM5_CH3
static BSP_PWM_t pwm_blue  = {&htim5, TIM_CHANNEL_1, PWM_CHANNEL_NORMAL};
static BSP_PWM_t pwm_green = {&htim5, TIM_CHANNEL_2, PWM_CHANNEL_NORMAL};
static BSP_PWM_t pwm_red   = {&htim5, TIM_CHANNEL_3, PWM_CHANNEL_NORMAL};

// 0-255 的无浮点正弦波表 (0 -> 255 -> 0)，用于呼吸灯平滑过渡
static const uint8_t Sine_Table[256] = {
    0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   6,   7,   8,   9,
    11,  12,  13,  15,  17,  18,  20,  22,  24,  26,  28,  30,  32,  35,  37,  39,
    42,  44,  47,  49,  52,  55,  58,  60,  63,  66,  69,  72,  75,  78,  81,  85,
    88,  91,  94,  97,  101, 104, 107, 111, 114, 117, 121, 124, 127, 131, 134, 137,
    141, 144, 147, 150, 154, 157, 160, 163, 167, 170, 173, 176, 179, 182, 185, 188,
    191, 194, 197, 200, 202, 205, 208, 210, 213, 215, 217, 220, 222, 224, 226, 229,
    231, 232, 234, 236, 238, 239, 241, 242, 244, 245, 246, 248, 249, 250, 251, 251,
    252, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 253, 253,
    252, 251, 251, 250, 249, 248, 246, 245, 244, 242, 241, 239, 238, 236, 234, 232,
    231, 229, 226, 224, 222, 220, 217, 215, 213, 210, 208, 205, 202, 200, 197, 194,
    191, 188, 185, 182, 179, 176, 173, 170, 167, 163, 160, 157, 154, 150, 147, 144,
    141, 137, 134, 131, 127, 124, 121, 117, 114, 111, 107, 104, 101, 97,  94,  91,
    88,  85,  81,  78,  75,  72,  69,  66,  63,  60,  58,  55,  52,  49,  47,  44,
    42,  39,  37,  35,  32,  30,  28,  26,  24,  22,  20,  18,  17,  15,  13,  12,
    11,  9,   8,   7,   6,   5,   4,   3,   2,   2,   1,   1,   1,   0,   0,   0
};

// 后台控制看板
static struct {
    LED_Mode_e mode;
    uint32_t   param_ms;
    uint8_t    base_r;
    uint8_t    base_g;
    uint8_t    base_b;
} LED_Manage;

/**
 * @brief 内部函数：直接下发占空比到硬件
 * @note 如果你的硬件是低电平点亮（即 0 最亮，255 最暗），请在此处将值反转，如：255 - r
 */
static void LED_Apply_Hardware(uint8_t r, uint8_t g, uint8_t b) {
    BSP_PWM_Set_Compare(&pwm_red,   r);
    BSP_PWM_Set_Compare(&pwm_green, g);
    BSP_PWM_Set_Compare(&pwm_blue,  b);
}

void LED_Init(void)
{
    // 启动 PWM 输出
    BSP_PWM_Start(&pwm_red);
    BSP_PWM_Start(&pwm_green);
    BSP_PWM_Start(&pwm_blue);
    // 默认关闭
    LED_SetMode_Static(0, 0, 0);
}

void LED_SetPixel(uint8_t r, uint8_t g, uint8_t b)
{
    LED_Manage.base_r = r;
    LED_Manage.base_g = g;
    LED_Manage.base_b = b;
}

void LED_SetMode_Static(uint8_t r, uint8_t g, uint8_t b)
{
    LED_SetPixel(r, g, b);
    LED_Manage.mode = LED_MODE_STATIC;
    LED_Manage.param_ms = 0;
}

void LED_SetMode_Breathing(uint8_t r, uint8_t g, uint8_t b, float period_s)
{
    if (period_s <= 0.0f) period_s = 1.0f;
    LED_SetPixel(r, g, b);
    LED_Manage.mode = LED_MODE_BREATHING;
    LED_Manage.param_ms = (uint32_t)(period_s * 1000.0f);
}

void LED_SetMode_Blink(uint8_t r, uint8_t g, uint8_t b, float interval_s)
{
    if (interval_s <= 0.0f) interval_s = 0.5f;
    LED_SetPixel(r, g, b);
    LED_Manage.mode = LED_MODE_BLINK;
    LED_Manage.param_ms = (uint32_t)(interval_s * 1000.0f);
}

/**
 * @brief 根据模式实时解算占空比并下发
 */
void LED_Ticks(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tick < 5) return;
    last_tick = now;
    uint8_t out_r = 0, out_g = 0, out_b = 0;
    switch (LED_Manage.mode)
    {
        case LED_MODE_STATIC:
            out_r = LED_Manage.base_r;
            out_g = LED_Manage.base_g;
            out_b = LED_Manage.base_b;
            break;
        case LED_MODE_BREATHING: {
            uint32_t idx = (now % LED_Manage.param_ms) * 255 / LED_Manage.param_ms;
            uint32_t factor = Sine_Table[idx];

            out_r = (LED_Manage.base_r * factor) >> 8;
            out_g = (LED_Manage.base_g * factor) >> 8;
            out_b = (LED_Manage.base_b * factor) >> 8;
            break;
        }
        case LED_MODE_BLINK: {
            uint8_t is_on = (now / LED_Manage.param_ms) % 2;
            if (is_on) {
                out_r = LED_Manage.base_r;
                out_g = LED_Manage.base_g;
                out_b = LED_Manage.base_b;
            }
            break;
        }
    }
    LED_Apply_Hardware(out_r, out_g, out_b);
}