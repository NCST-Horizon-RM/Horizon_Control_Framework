//
// Created by CaoKangqi on 2026/2/13.
//
#include "System_Init.h"

#include "BMI088.h"
#include "BSP_DWT.h"
#include "BSP_CAN.h"
#include "WS2812.h"
#include "BSP_TIM.h"
#include "BSP_UART.h"
#include "Buzzer.h"
#include "System_State.h"
#include "Robot_Cmd.h"
#include "Robot_Config.h"
#include "System_Indicator.h"

uint32_t stm32_id[3];
void Get_UID(uint32_t *uid) {
    uid[0] = HAL_GetUIDw0();
    uid[1] = HAL_GetUIDw1();
    uid[2] = HAL_GetUIDw2();
}
void System_Init() {
    DWT_Init(550);
    Get_UID(stm32_id);

    //CAN滤波器初始化
    CAN_Config(&hfdcan1, CLASSIC_1M, FDCAN_RX_FIFO0);
    CAN_Config(&hfdcan2, CLASSIC_1M, FDCAN_RX_FIFO1);
    CAN_Config(&hfdcan3, CLASSIC_1M, FDCAN_RX_FIFO0);
    //CAN设备初始化
    BSP_CAN_Auto_Init();
    //串口设备初始化
    Auto_UART_Router_Init();
    //WS2812初始化
    WS2812_Init();
    //蜂鸣器初始化
    Buzzer_Init();
    //TODO 这里不该出现HAL库代码的，偷个懒后面再改
    //开启XT30 2+2 可控输出
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_SET);
    //开启对外5V
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);

    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    //BMI088初始化
    BMI088_Init();
    //注册设备数据到数据中心(须在订阅方之前)
    Robot_Config_Init();
    //系统状态监测初始化
    System_Indicator_Init();
    System_State_Init();
    //指令中心初始化
    Robot_Cmd_Init();
}