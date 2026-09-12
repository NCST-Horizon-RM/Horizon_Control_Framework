//
// Created by user on 2026/9/11.
//

#include "BSP_ADC.h"
#include "adc.h"

static float ADC_date = 0;
static float Battery_Voltage = 0;

void BSP_ADC_Init(void) {
    HAL_ADC_Start_IT(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        ADC_date = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Start_IT(&hadc1);
    }
}

float BSP_ADC_Get_Battery_Voltage(float Battery_ratio) {
    Battery_Voltage = ((ADC_date / 65535.0f * 3.3f) * 11 * Battery_ratio) - 0.8;
    return Battery_Voltage;
}