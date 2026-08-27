//
// Created by CaoKangqi on 2026/1/19.
//
#include "BSP_DWT.h"

DWT_Time_t SysTime;

static uint32_t CPU_FREQ_Hz;
static uint32_t CPU_FREQ_Hz_ms;
static uint32_t CPU_FREQ_Hz_us;

static uint32_t CYCCNT_RoundCount = 0;
static uint32_t CYCCNT_LAST = 0;
uint64_t CYCCNT64 = 0;

static inline void DWT_CNT_Update(uint32_t cnt_now)
{
    if (cnt_now < CYCCNT_LAST) {
        CYCCNT_RoundCount++;
    }
    CYCCNT_LAST = cnt_now;
}

void DWT_Init(uint32_t CPU_Freq_MHz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    #if defined(__CORTEX_M) && (__CORTEX_M == 7 || __CORTEX_M == 33)
    DWT->LAR = 0xC5ACCE55;
    #endif

    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_MHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    CYCCNT_RoundCount = 0;
    CYCCNT_LAST = DWT->CYCCNT;
    CYCCNT64 = 0;
    __set_PRIMASK(primask);
}

void DWT_SysTimeUpdate(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t cnt_now = DWT->CYCCNT;
    DWT_CNT_Update(cnt_now);

    CYCCNT64 = ((uint64_t)CYCCNT_RoundCount << 32) | cnt_now;
    uint64_t total_us = CYCCNT64 / CPU_FREQ_Hz_us;

    __set_PRIMASK(primask);

    SysTime.s  = (uint32_t)(total_us / 1000000);
    SysTime.ms = (uint16_t)((total_us % 1000000) / 1000);
    SysTime.us = (uint16_t)(total_us % 1000);
}

float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t cnt_now = DWT->CYCCNT;
    DWT_CNT_Update(cnt_now);
    __set_PRIMASK(primask);

    float dt = (float)(cnt_now - *cnt_last) / CPU_FREQ_Hz;
    *cnt_last = cnt_now;
    return dt;
}

double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t cnt_now = DWT->CYCCNT;
    DWT_CNT_Update(cnt_now);
    __set_PRIMASK(primask);

    double dt = (double)(cnt_now - *cnt_last) / CPU_FREQ_Hz;
    *cnt_last = cnt_now;
    return dt;
}

double DWT_GetTimeline_s(void) {
    DWT_SysTimeUpdate();
    return (double)SysTime.s + (double)SysTime.ms * 0.001 + (double)SysTime.us * 0.000001;
}

double DWT_GetTimeline_ms(void){
    DWT_SysTimeUpdate();
    return (double)SysTime.s * 1000.0 + (double)SysTime.ms + (double)SysTime.us * 0.001;
}

uint64_t DWT_GetTimeline_us(void) {
    DWT_SysTimeUpdate();
    return (uint64_t)SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;
}


void DWT_Delay_us(uint32_t uSec)
{
    while (uSec > 1000000) {
        uint32_t start_tick = DWT->CYCCNT;
        uint32_t chunk_ticks = 1000000 * CPU_FREQ_Hz_us;
        while ((DWT->CYCCNT - start_tick) < chunk_ticks) {
        }
        uSec -= 1000000;
    }

    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_ticks = uSec * CPU_FREQ_Hz_us;
    while ((DWT->CYCCNT - start_tick) < delay_ticks) {
    }
}

void DWT_Delay_ms(float Delay) { DWT_Delay_us((uint32_t)(Delay * 1000.0f)); }
void DWT_Delay_s(float Delay)  { DWT_Delay_us((uint32_t)(Delay * 1000000.0f)); }


/**
 * @brief  基于 64 位绝对时间线，支持任意长度的无阻塞超时
 */
void DWT_Set_Timeout(DWT_Timeout_t *timeout, float ms) {
    timeout->start_time_us = DWT_GetTimeline_us();
    timeout->delay_us = (uint64_t)(ms * 1000.0f);
}

bool DWT_Check_Timeout(DWT_Timeout_t *timeout) {
    return ((DWT_GetTimeline_us() - timeout->start_time_us) >= timeout->delay_us);
}

/* ================== 代码执行性能分析 ================== */

void DWT_Profile_Start(DWT_Profiler_t *profiler) {
    profiler->start_tick = DWT->CYCCNT;
}

void DWT_Profile_Stop(DWT_Profiler_t *profiler) {
    uint32_t delta_tick = DWT->CYCCNT - profiler->start_tick;
    profiler->cost_us = (float)delta_tick / CPU_FREQ_Hz_us;
    profiler->cost_ms = (float)delta_tick / CPU_FREQ_Hz_ms;
}