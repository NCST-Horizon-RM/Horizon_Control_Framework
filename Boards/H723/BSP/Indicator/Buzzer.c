//
// Created by CaoKangqi on 2026/6/21.
//
#include "Buzzer.h"

BSP_PWM_t buzzer_pwm     = {&htim12, TIM_CHANNEL_2, PWM_CHANNEL_NORMAL};

typedef enum {
    BUZZER_IDLE = 0,
    BUZZER_STATE_RUNNING
} Buzzer_Ctrl_State_e;

// 每个状态音效最多包含 6 个音符片段
#define MAX_EFFECT_FRAGMENTS 6

typedef struct {
    uint16_t freq;
    uint16_t duration;
} Tone_Fragment_t;

typedef struct {
    Buzzer_Ctrl_State_e state;
    uint32_t timer_ms;
    Tone_Fragment_t effect[MAX_EFFECT_FRAGMENTS];
    uint8_t total_fragments;
    uint8_t current_fragment;
} Buzzer_Ctrl_t;

static Buzzer_Ctrl_t buzzer;

void Buzzer_Init(void)
{
    BSP_PWM_Start(&buzzer_pwm);
    buzzer.state = BUZZER_IDLE;
    buzzer.timer_ms = 0;
    buzzer.total_fragments = 0;
    buzzer.current_fragment = 0;
    Buzzer_Off();
}

void Buzzer_Off(void)
{
    BSP_PWM_Set_Compare(&buzzer_pwm, 0);
}

void Buzzer_Set_Freq(uint16_t frequency_hz)
{
    if (frequency_hz == 0) {
        Buzzer_Off();
    } else {
        // 基于 TIM12 1MHz 计数时钟
        uint32_t arr = 1000000 / frequency_hz - 1;
        uint32_t ccr = (arr + 1) / 2; // 50% 占空比
        BSP_PWM_Set_Autoreload_Immediate(&buzzer_pwm, arr, ccr);
    }
}