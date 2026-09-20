//
// Created by CaoKangqi on 2026/6/25.
//
#include "Offline_Detector.h"
#include "BSP_UART.h"
#include "BSP_CAN.h"
#include "All_define.h"
#include "DBUS.h"
#include "VT13.h"
#include "Referee.h"
#include "Robot_Config.h"
#include "Comm_DualBoard.h"
#include "Power_CAP.h"
#include "System_State.h"
#include "IMU_Task.h"
#include "Robot_Cmd.h"

// 设备数据实例：ISR 写 / 任务读（裸全局 volatile）
Chassis_Motor_Group_t chassis_motors;
Leg_Motor_Group_t leg_motors;

Referee_Data_t Referee;
static uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH]__attribute__((section(".RAM_D2")));

DBUS_Typedef DBUS = {0};
static uint8_t DBUS_RX_DATA[18]__attribute__((section(".RAM_D2")));
Cap_t cap;
Power_Meter_t Meter;

/* ================= 链接器段自动注册 ================= */

UART_RX_NODE(&huart5, 100000,0,0, 18, DBUS_RX_DATA, NULL, 18, &DBUS, DBUS_Resolved);
OFFLINE_NODE(&DBUS.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart7, 921600,0,0, 21, NULL, NULL, 21, NULL, VT13_Resolved);

UART_RX_NODE(&huart1, 1152000, 0,0,0, Referee_Rx_Buf[0], Referee_Rx_Buf[1], REFEREE_RXFRAME_LENGTH, &Referee, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN1, 0x203, &chassis_motors.DJI_3508_Chassis[0], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN1, 0x201, &chassis_motors.DJI_3508_Chassis[1], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[1].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN1, 0x603, &Meter, CAN_Power_Rx);

CAN_RX_NODE(FDCAN2, 0x51, &leg_motors.BM_P1010B_Leg[0], BM_Motor_Resolve);
OFFLINE_NODE(&leg_motors.BM_P1010B_Leg[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN2, 0x52, &leg_motors.BM_P1010B_Leg[1], BM_Motor_Resolve);
OFFLINE_NODE(&leg_motors.BM_P1010B_Leg[1].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN2, 0x53, &leg_motors.BM_P1010B_Leg[2], BM_Motor_Resolve);
OFFLINE_NODE(&leg_motors.BM_P1010B_Leg[2].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN2, 0x54, &leg_motors.BM_P1010B_Leg[3], BM_Motor_Resolve);
OFFLINE_NODE(&leg_motors.BM_P1010B_Leg[3].offline, MOTOR_OFFLINE_TIME, CHASSIS);

// 设备通过链接器段自动装配，无需运行时注册
void Robot_Config_Init(void)
{
}
