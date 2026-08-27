//
// Created by CaoKangqi on 2026/6/14.
//
#include "All_Task.h"
#include "BSP_SPI.h"
#include "Robot_Config.h"
#include "DBUS.h"
#include "LED.h"
#include "Message_Center.h"
#include "Gimbal_Ctrl.h"
#include "Shoot_Ctrl.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "System_Indicator.h"
#include "VT13.h"
#include "Vofa.h"
// 指令中心任务 200Hz
void Command_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(5);//绝对延时5ms
    PubRegister("imu_data",   &IMU_Data,  sizeof(IMU_Data));
    PubRegister("gimbal_motors",  &gimbal_motors,  sizeof(Gimbal_Motor_Group_t));
    PubRegister("shoot_motors",  &shoot_motors,  sizeof(Shoot_Motor_Group_t));
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

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
DWT_Profiler_t ins_time;
void IMU_Task(void *argument) {
    (void)argument;
    xIMUTaskHandle = xTaskGetCurrentTaskHandle();
    // 向 BSP 层注册中断回调
    BSP_SPI_RegisterIRQCallback(IMU_Interrupt_Handler);
    INS_DWT_Count = DWT->CYCCNT;
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_period_s = DWT_GetDeltaT(&INS_DWT_Count);
        DWT_Profile_Start(&ins_time);
        IMU_Update_Task(&IMU_Data, imu_period_s);
        DWT_Profile_Stop(&ins_time);
        DWT_SysTimeUpdate();
    }
}

// 运动控制任务 1000Hz
static IMU_Data_t imu ={0};
static Gimbal_Motor_Group_t gimbal_m = {0};
static Shoot_Motor_Group_t shoot_m = {0};
static uint32_t motor_DWT_Count = 0;
static float motor_period_s = 0.0f;
void Motor_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms

    Subscriber_t *imu_sub = NULL;
    Subscriber_t *g_motor_sub = NULL;
    Subscriber_t *s_motor_sub = NULL;

    imu_sub = SubRegister("imu_data", sizeof(IMU_Data_t));
    g_motor_sub = SubRegister("gimbal_motors", sizeof(Gimbal_Motor_Group_t));
    s_motor_sub = SubRegister("shoot_motors", sizeof(Shoot_Motor_Group_t));
    motor_DWT_Count = DWT->CYCCNT;
    Gimbal_Control_Init();
    Shoot_Control_Init();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);
        motor_period_s = DWT_GetDeltaT(&motor_DWT_Count);

        if (imu_sub) SubGetMessage(imu_sub, &imu);
        if (g_motor_sub) SubGetMessage(g_motor_sub, &gimbal_m);
        if (s_motor_sub)  SubGetMessage(s_motor_sub, &shoot_m);
        
        Gimbal_Control_Task(&gimbal_m,&imu);
        Shoot_Control_Task(&shoot_m,motor_period_s);
    }
}

// 自定义任务1 1000Hz
void StartTask01(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);
        //在这里加代码
    }
}

// 自定义任务2 1000Hz
void StartTask02(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(1);//绝对延时1ms
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);
        //在这里加代码
    }
}

//定时器中断
void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    //定时器3 1000Hz
    if (htim->Instance == TIM3) {
        DWT_SysTimeUpdate();
        Offline_Monitor();
        System_State_Update();
        LED_Ticks();
        System_Indicator_Ticks();
    }
    //定时器6 500Hz
    if (htim->Instance == TIM6) {

    }
    //定时器7 200Hz
    if (htim->Instance == TIM7) {

    }
}