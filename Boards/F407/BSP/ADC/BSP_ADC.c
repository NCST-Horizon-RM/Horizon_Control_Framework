//
// Created by user on 2026/9/11.
//

#include "BSP_ADC.h"
#include "adc.h"

static volatile uint16_t ADC_date = 0;   /* 原始值：中断里写，外面读 */
static uint32_t s_last_ms = 0;

void BSP_ADC_Init(void) {
    ADC_date = 0;
    s_last_ms = 0;
    HAL_ADC_Start_IT(&hadc3);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC3) {
        ADC_date = HAL_ADC_GetValue(&hadc3);   /* 只存值，不重挂 */
    }
}

float BSP_ADC_Get_Battery_Voltage(float Battery_ratio) {
    uint32_t now = HAL_GetTick();

    /* 每 BSP_ADC_PERIOD_MS 才发起一次转换 —— 中断频率被这里限住 */
    if ((now - s_last_ms) >= BSP_ADC_PERIOD_MS) {
        s_last_ms = now;
        (void)HAL_ADC_Start_IT(&hadc3);
    }

    return (ADC_date / 4095.0f * 3.3f) * 10.09f * Battery_ratio + 0.6f;
}