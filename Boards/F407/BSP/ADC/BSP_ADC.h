//
// Created by user on 2026/9/11.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_BSP_ADC_H
#define HORIZON_CONTROL_FRAMEWORK_BSP_ADC_H
#include "main.h"

#define BSP_ADC_PERIOD_MS   50U

void BSP_ADC_Init(void);
float BSP_ADC_Get_Battery_Voltage(float Battery_ratio);
#endif //HORIZON_CONTROL_FRAMEWORK_BSP_ADC_H
