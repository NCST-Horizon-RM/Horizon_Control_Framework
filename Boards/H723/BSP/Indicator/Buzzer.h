//
// Created by CaoKangqi on 2026/6/21.
//

#ifndef H7_FRAMEWORK_BUZZER_H
#define H7_FRAMEWORK_BUZZER_H

#include "BSP_TIM.h"
#include "stdint.h"

/* 初始化蜂鸣器 */
void Buzzer_Init(void);

/* 关闭蜂鸣器 */
void Buzzer_Off(void);

/**
 * @brief 纯硬件原子接口：直接改变蜂鸣器当前的物理输出频率
 * @param frequency_hz 频率(Hz)，传入0则关闭输出
 */
void Buzzer_Set_Freq(uint16_t frequency_hz);

extern BSP_PWM_t buzzer_pwm;

#endif //H7_FRAMEWORK_BUZZER_H
