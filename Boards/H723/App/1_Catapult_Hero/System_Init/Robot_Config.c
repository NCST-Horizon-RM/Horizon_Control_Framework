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
Gimbal_Motor_Group_t gimbal_motors;
Shoot_Motor_Group_t shoot_motors;

Referee_Data_t Referee;
static uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH]__attribute__((section(".RAM_D2")));
VT13_Typedef VT13 = {0};
static uint8_t VT13_RX_DATA[21]__attribute__((section(".RAM_D2")));
DBUS_Typedef DBUS = {0};
static uint8_t DBUS_RX_DATA[18]__attribute__((section(".RAM_D2")));
Cap_t cap;

/* ================= 链接器段自动注册 ================= */

UART_RX_NODE(&huart5, 100000, 18, DBUS_RX_DATA, NULL, 18, &DBUS, DBUS_Resolved);
OFFLINE_NODE(&DBUS.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart7, 921600, 21, VT13_RX_DATA, NULL, 21, &VT13, VT13_Resolved);
OFFLINE_NODE(&VT13.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart1, 115200, 0, Referee_Rx_Buf[0], Referee_Rx_Buf[1], REFEREE_RXFRAME_LENGTH, &Referee, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(FDCAN1, 0x201, &chassis_motors.DJI_3508_Chassis[0], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN1, 0x202, &chassis_motors.DJI_3508_Chassis[1], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[1].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN1, 0x203, &chassis_motors.DJI_3508_Chassis[2], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[2].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN1, 0x204, &chassis_motors.DJI_3508_Chassis[3], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[3].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN2, 0x205, &chassis_motors.DJI_6020_Steer[0], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_6020_Steer[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN2, 0x206, &chassis_motors.DJI_6020_Steer[1], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_6020_Steer[1].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN2, 0x207, &chassis_motors.DJI_6020_Steer[2], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_6020_Steer[2].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN2, 0x208, &chassis_motors.DJI_6020_Steer[3], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_6020_Steer[3].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(FDCAN3, 0x301, &shoot_motors.DM4310_Feed, DM_1to4_Resolve);
OFFLINE_NODE(&shoot_motors.DM4310_Feed.offline, MOTOR_OFFLINE_TIME, SHOOT);

CAN_RX_NODE(FDCAN3, 0x202, &shoot_motors.DJI_3508_Pull, DJI_Motor_Resolve);
OFFLINE_NODE(&shoot_motors.DJI_3508_Pull.offline, MOTOR_OFFLINE_TIME, SHOOT);

CAN_RX_NODE(FDCAN3, 0x203, &gimbal_motors.DJI_3508_Yaw, DJI_Motor_Resolve);
OFFLINE_NODE(&gimbal_motors.DJI_3508_Yaw.offline, MOTOR_OFFLINE_TIME, GIMBAL);

CAN_RX_NODE(FDCAN3, 0x288, &cap, Power_Cap_Rx);
OFFLINE_NODE(&cap.get.offline, CAP_OFFLINE_TIME, GROUP_NONE);

// 设备通过链接器段自动装配，无需运行时注册
void Robot_Config_Init(void)
{
}
