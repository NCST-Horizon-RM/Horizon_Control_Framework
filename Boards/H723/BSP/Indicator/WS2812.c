//
// Created by CaoKangqi on 2026/1/23.
//
#include "WS2812.h"
#include <string.h>

BSP_PWM_t ws2812_pwm     = {&htim3,  TIM_CHANNEL_2, PWM_CHANNEL_NORMAL};

// 0-255 的无浮点正弦波表 (0 -> 255 -> 0)
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

// 【新增控制块】管理后台灯效模式
typedef struct {
    LED_Mode_e mode;
    uint32_t   param_ms; // 存放转为毫秒的周期或间隔时间
} LED_Manage_t;

static WS2812_Color_t Base_Color[MAX_LED];
static WS2812_Color_t LED_Data[MAX_LED];
static LED_Manage_t   LED_Manage[MAX_LED]; // 后台控制看板

// 乒乓双缓冲大小 (24 bits * 2 空间 = 48 字节)
#define WS2812_DMA_BUF_LEN (24 * 2)
__attribute__((section(".RAM_D2"))) uint16_t DMA_Buffer[WS2812_DMA_BUF_LEN];

static uint8_t Global_Brightness = 255;
static volatile uint8_t isSending = 0;
static uint16_t send_pixel_idx = 0;

/**
 * @brief 填充缓冲（原函数保持不变）
 */
static void Fill_Buffer(uint16_t ledIdx, uint16_t bufferOffset, uint8_t isReset)
{
    if (isReset) {
        memset(&DMA_Buffer[bufferOffset], 0, 24 * sizeof(uint16_t));
        return;
    }

    uint8_t r = LED_Data[ledIdx].R;
    uint8_t g = LED_Data[ledIdx].G;
    uint8_t b = LED_Data[ledIdx].B;

    if (Global_Brightness < 255) {
        r = (r * Global_Brightness) >> 8;
        g = (g * Global_Brightness) >> 8;
        b = (b * Global_Brightness) >> 8;
    }

    uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;

    for (int8_t i = 23; i >= 0; i--) {
        DMA_Buffer[bufferOffset++] = (color & (1 << i)) ? WS2812_PWM_HIGH : WS2812_PWM_LOW;
    }
}

void WS2812_Init(void)
{
    isSending = 0;
    // 默认所有人都是静态常灭
    for(uint16_t i = 0; i < MAX_LED; i++) {
        LED_Manage[i].mode = LED_MODE_STATIC;
        LED_Manage[i].param_ms = 0;
    }
    WS2812_Clear();
    BSP_PWM_Register_DMA_Callback(ws2812_pwm.htim, WS2812_DMA_Handler);
}

void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= MAX_LED) return;
    Base_Color[index].R = r;
    Base_Color[index].G = g;
    Base_Color[index].B = b;
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < MAX_LED; i++) {
        WS2812_SetPixel(i, r, g, b);
    }
}

void WS2812_Clear(void)
{
    memset(Base_Color, 0, sizeof(Base_Color));
    memset(LED_Data, 0, sizeof(LED_Data));
}

void WS2812_Send(void)
{
    if (isSending) return;
    send_pixel_idx = 0;

    Fill_Buffer(0, 0, 0);
    if (MAX_LED > 1) {
        Fill_Buffer(1, 24, 0);
        send_pixel_idx = 2;
    } else {
        Fill_Buffer(0, 24, 1);
        send_pixel_idx = 1;
    }

    isSending = 1;
    HAL_TIM_PWM_Start_DMA(ws2812_pwm.htim, ws2812_pwm.channel, (uint32_t *)DMA_Buffer, WS2812_DMA_BUF_LEN);
}

void WS2812_DMA_Handler(uint8_t half_cplt)
{
    if (half_cplt == 1) {
        if (send_pixel_idx < MAX_LED) {
            Fill_Buffer(send_pixel_idx, 0, 0);
            send_pixel_idx++;
        }
        else if (send_pixel_idx < MAX_LED + WS2812_RESET_SLOTS) {
            Fill_Buffer(0, 0, 1);
            send_pixel_idx++;
        }
    }
    else {
        if (send_pixel_idx < MAX_LED) {
            Fill_Buffer(send_pixel_idx, 24, 0);
            send_pixel_idx++;
        }
        else if (send_pixel_idx < MAX_LED + WS2812_RESET_SLOTS) {
            Fill_Buffer(0, 24, 1);
            send_pixel_idx++;
        }
        else {
            HAL_TIM_PWM_Stop_DMA(ws2812_pwm.htim, ws2812_pwm.channel);
            isSending = 0;
            BSP_PWM_Set_Compare(&ws2812_pwm, 0);
        }
    }
}

/* ==================== 以下为全新模式管理层实现 ==================== */

void WS2812_SetMode_Static(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= MAX_LED) return;
    WS2812_SetPixel(index, r, g, b);
    LED_Manage[index].mode = LED_MODE_STATIC;
    LED_Manage[index].param_ms = 0;
}

void WS2812_SetMode_Breathing(uint16_t index, uint8_t r, uint8_t g, uint8_t b, float period_s)
{
    if (index >= MAX_LED) return;
    if (period_s <= 0.0f) period_s = 1.0f;

    WS2812_SetPixel(index, r, g, b);
    LED_Manage[index].mode = LED_MODE_BREATHING;
    LED_Manage[index].param_ms = (uint32_t)(period_s * 1000);
}

void WS2812_SetMode_Blink(uint16_t index, uint8_t r, uint8_t g, uint8_t b, float interval_s)
{
    if (index >= MAX_LED) return;
    if (interval_s <= 0.0f) interval_s = 0.5f;

    WS2812_SetPixel(index, r, g, b);
    LED_Manage[index].mode = LED_MODE_BLINK;
    LED_Manage[index].param_ms = (uint32_t)(interval_s * 1000);
}

/**
 * @brief  核心心跳：隐式完成颜色看板刷新与一键发送，建议 20ms 或 30ms 节拍调用
 */
void WS2812_Ticks(void)
{
    // 利用硬件内置的分频发送锁，如果上一次的 DMA 串行流还没发完，直接不计算，安全退出
    if (isSending) return;

    // 引入一个全局帧率节拍控制（比如限制物理刷新间隔为 25ms 左右，保证丝滑且不给总线增加负担）
    static uint32_t last_send_tick = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_send_tick < 25) return;
    last_send_tick = now;

    // 隐式状态机大循环：在后台根据各灯珠的模式实时解算物理色彩
    for (uint16_t i = 0; i < MAX_LED; i++)
    {
        switch (LED_Manage[i].mode)
        {
            case LED_MODE_STATIC:
                // 常亮模式：直接同步物理底色
                LED_Data[i] = Base_Color[i];
                break;

            case LED_MODE_BREATHING: {
                // 呼吸模式后台解算
                uint32_t idx = (now % LED_Manage[i].param_ms) * 255 / LED_Manage[i].param_ms;
                uint32_t factor = Sine_Table[idx];

                LED_Data[i].R = (Base_Color[i].R * factor) >> 8;
                LED_Data[i].G = (Base_Color[i].G * factor) >> 8;
                LED_Data[i].B = (Base_Color[i].B * factor) >> 8;
                break;
            }

            case LED_MODE_BLINK: {
                // 闪烁模式后台解算
                uint8_t is_on = (now / LED_Manage[i].param_ms) % 2;
                if (is_on) {
                    LED_Data[i] = Base_Color[i];
                } else {
                    LED_Data[i].R = 0;
                    LED_Data[i].G = 0;
                    LED_Data[i].B = 0;
                }
                break;
            }
        }
    }
    // 后台无感自动喷射刷新
    WS2812_Send();
}