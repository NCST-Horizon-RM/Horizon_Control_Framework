//
// Created by CaoKangqi on 2026/6/14.
//
#include "All_Task.h"

#include "BSP_SPI.h"
#include "Robot_Config.h"
#include "Buzzer.h"
#include "Chassis_Ctrl.h"
#include "DBUS.h"
#include "LED.h"
#include "Power_CAP.h"
#include "Referee.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "System_Indicator.h"
#include "VT13.h"
#include "Vofa.h"
// 指令中心任务 200Hz
static uint32_t CMD_DWT_Count = 0;
static float cmd_period_s = 0.0f;
void Command_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(5);//绝对延时5ms

    CMD_DWT_Count = DWT->CYCCNT;
    Robot_Cmd_Init();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        cmd_period_s = DWT_GetDeltaT(&CMD_DWT_Count);
        Robot_Cmd_Update();
    }
}

// IMU任务 中断触发
static TaskHandle_t xIMUTaskHandle = NULL;
static void IMU_Interrupt_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xIMUTaskHandle != NULL) {
        xTaskNotifyFromISR(xIMUTaskHandle, 0, eIncrement, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
static uint32_t INS_DWT_Count = 0;
static float imu_period_s = 0.0f;
void IMU_Task(void *argument) {
    (void)argument;
    xIMUTaskHandle = xTaskGetCurrentTaskHandle();
    // 向 BSP 层注册中断回调
    BSP_SPI_RegisterIRQCallback(IMU_Interrupt_Handler);
    INS_DWT_Count = DWT->CYCCNT;
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_period_s = DWT_GetDeltaT(&INS_DWT_Count);
        IMU_Update_Task(&IMU_Data, imu_period_s);
    }
}

// 运动控制任务 1000Hz
static uint32_t motor_DWT_Count = 0;
static float motor_period_s = 0.0f;
void Motor_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    motor_DWT_Count = DWT->CYCCNT;
    Chassis_Control_Init();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        motor_period_s = DWT_GetDeltaT(&motor_DWT_Count);

        Chassis_Control_Task(&chassis_motors,&IMU_Data,motor_period_s);
    }
}

// 自定义任务1 1000Hz
static uint32_t TASK1_DWT_Count = 0;
static float TASK1_Period_S = 0.0f;
void StartTask01(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    TASK1_DWT_Count = DWT->CYCCNT;
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        TASK1_Period_S = DWT_GetDeltaT(&TASK1_DWT_Count);
        //在这里加代码
    }
}

// 自定义任务2 1000Hz
static uint32_t TASK2_DWT_Count = 0;
static float TASK2_Period_S = 0.0f;
void StartTask02(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms
    TASK2_DWT_Count = DWT->CYCCNT;
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        TASK2_Period_S = DWT_GetDeltaT(&TASK2_DWT_Count);
        //在这里加代码
    }
}

//定时器中断
void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    //定时器3 1000Hz
    if (htim->Instance == TIM3) {
        DWT_SysTimeUpdate();//系统时间
        Offline_Monitor();
        System_State_Update(&Referee);
        LED_Ticks();
        System_Indicator_Ticks();//蜂鸣器
    }
    //定时器6 500Hz
    if (htim->Instance == TIM6) {

    }
    //定时器7 200Hz
    if (htim->Instance == TIM7) {

    }
}