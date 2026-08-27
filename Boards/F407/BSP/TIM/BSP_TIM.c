//
// Created by CaoKangqi on 2026/6/19.
//
#include "BSP_TIM.h"

#define MAX_PWM_DMA_SLOTS 4
typedef struct {
    TIM_HandleTypeDef *htim;
    PWM_DMA_Callback_t callback;
} PWM_DMA_Slot_t;

static PWM_DMA_Slot_t dma_slots[MAX_PWM_DMA_SLOTS] = {0};

/**
 * @brief 注册 PWM DMA 中断回调
 */
void BSP_PWM_Register_DMA_Callback(TIM_HandleTypeDef *htim, PWM_DMA_Callback_t callback)
{
    for (int i = 0; i < MAX_PWM_DMA_SLOTS; i++) {
        // 找到空槽位或者覆盖同一个 htim 的槽位
        if (dma_slots[i].htim == NULL || dma_slots[i].htim == htim) {
            dma_slots[i].htim = htim;
            dma_slots[i].callback = callback;
            return;
        }
    }
}


void BSP_PWM_Start(BSP_PWM_t *pwm)
{
    if (pwm == NULL || pwm->htim == NULL) return;
    if (pwm->type == PWM_CHANNEL_COMP) {
        HAL_TIMEx_PWMN_Start(pwm->htim, pwm->channel);
    } else {
        HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    }
}

void BSP_PWM_Stop(BSP_PWM_t *pwm)
{
    if (pwm == NULL || pwm->htim == NULL) return;
    if (pwm->type == PWM_CHANNEL_COMP) {
        HAL_TIMEx_PWMN_Stop(pwm->htim, pwm->channel);
    } else {
        HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
    }
}

void BSP_PWM_Set_Compare(BSP_PWM_t *pwm, uint32_t compare)
{
    if (pwm == NULL || pwm->htim == NULL) return;
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, compare);
}

void BSP_PWM_Set_Autoreload(BSP_PWM_t *pwm, uint32_t autoreload)
{
    if (pwm == NULL || pwm->htim == NULL) return;
    __HAL_TIM_SET_AUTORELOAD(pwm->htim, autoreload);
}

void BSP_PWM_Set_Autoreload_Immediate(BSP_PWM_t *pwm, uint32_t autoreload, uint32_t compare)
{
    if (pwm == NULL || pwm->htim == NULL) return;
    TIM_HandleTypeDef *htim = pwm->htim;

    __HAL_TIM_SET_COMPARE(htim, pwm->channel, 0);
    __HAL_TIM_SET_AUTORELOAD(htim, autoreload);
    __HAL_TIM_SET_COMPARE(htim, pwm->channel, compare);
    __HAL_TIM_SET_COUNTER(htim, 0);
    htim->Instance->EGR = TIM_EGR_UG;
}


static void Dispatch_PWM_DMA_Callback(TIM_HandleTypeDef *htim, uint8_t is_half)
{
    for (int i = 0; i < MAX_PWM_DMA_SLOTS; i++) {
        if (dma_slots[i].htim == htim && dma_slots[i].callback != NULL) {
            dma_slots[i].callback(is_half);
            return;
        }
    }
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    Dispatch_PWM_DMA_Callback(htim, 1);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    Dispatch_PWM_DMA_Callback(htim, 0);
}