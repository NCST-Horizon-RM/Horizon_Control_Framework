#ifndef H7_FRAMEWORK_BSP_TIM_H
#define H7_FRAMEWORK_BSP_TIM_H

#include "tim.h"

// 通道极性类型
typedef enum {
    PWM_CHANNEL_NORMAL = 0, // 普通正向通道 (CH1, CH2, CH3, CH4)
    PWM_CHANNEL_COMP   = 1, // 高级定时器互补/反向通道 (CH1N, CH2N, CH3N)
} BSP_PWM_Type_e;

// DMA 传输回调函数指针
typedef void (*PWM_DMA_Callback_t)(uint8_t half_cplt);

/**
 * @brief PWM 对象句柄 (Device 层或 App 层持有此句柄)
 */
typedef struct {
    TIM_HandleTypeDef *htim; // HAL 库定时器句柄
    uint32_t channel;        // 定时器通道号 (如 TIM_CHANNEL_2)
    BSP_PWM_Type_e type;     // 通道极性
} BSP_PWM_t;

void BSP_PWM_Start(BSP_PWM_t *pwm);
void BSP_PWM_Stop(BSP_PWM_t *pwm);
void BSP_PWM_Set_Compare(BSP_PWM_t *pwm, uint32_t compare);
void BSP_PWM_Set_Autoreload(BSP_PWM_t *pwm, uint32_t autoreload);
void BSP_PWM_Set_Autoreload_Immediate(BSP_PWM_t *pwm, uint32_t autoreload, uint32_t compare);

void BSP_PWM_Register_DMA_Callback(TIM_HandleTypeDef *htim, PWM_DMA_Callback_t callback);

#endif //H7_FRAMEWORK_BSP_TIM_H