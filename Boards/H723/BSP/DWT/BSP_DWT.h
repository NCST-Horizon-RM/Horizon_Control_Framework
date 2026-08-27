//
// Created by CaoKangqi on 2026/1/19.
//

#ifndef G4_FRAMEWORK_BSP_DWT_H
#define G4_FRAMEWORK_BSP_DWT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

// 修改：超时定时器改用 64 位绝对时间，突破 10 秒限制
typedef struct {
    uint64_t start_time_us;
    uint64_t delay_us;
} DWT_Timeout_t;

typedef struct {
    uint32_t start_tick;
    float cost_us;
    float cost_ms;
} DWT_Profiler_t;

extern DWT_Time_t SysTime;
extern uint64_t CYCCNT64;

void DWT_Init(uint32_t CPU_Freq_MHz);
void DWT_SysTimeUpdate(void);

float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);

// 修改：绝对时间戳返回 double，防止 float 在 16 秒后精度截断
double DWT_GetTimeline_s(void);
double DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);

void DWT_Delay_us(uint32_t uSec);
void DWT_Delay_ms(float Delay);
void DWT_Delay_s(float Delay);

void DWT_Set_Timeout(DWT_Timeout_t *timeout, float ms);
bool DWT_Check_Timeout(DWT_Timeout_t *timeout);

void DWT_Profile_Start(DWT_Profiler_t *profiler);
void DWT_Profile_Stop(DWT_Profiler_t *profiler);

#endif //G4_FRAMEWORK_BSP_DWT_H