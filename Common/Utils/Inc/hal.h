//
// 板级 HAL 兼容头：让 Shared 上层代码不直接 include 具体芯片的 HAL 头
// 由 CMake 按板子注入 STM32F407xx / STM32H723xx 宏，此处据此选择。
//
#ifndef HORIZON_HAL_H
#define HORIZON_HAL_H

#if defined(STM32H723xx)
#include "stm32h7xx_hal.h"
#elif defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#else
#error "Unknown target: define STM32H723xx or STM32F407xx"
#endif

/* USB CDC 发送函数名：F4 全速(FS) / H7 高速(HS)，两板函数名不同 */
#if defined(STM32H723xx)
#define CDC_Transmit CDC_Transmit_HS
#elif defined(STM32F407xx)
#define CDC_Transmit CDC_Transmit_FS
#endif

#endif //HORIZON_HAL_H
