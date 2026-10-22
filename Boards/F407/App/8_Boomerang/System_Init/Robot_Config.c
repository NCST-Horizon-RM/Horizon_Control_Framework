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
#include "DualBoard_Frame.h"
#include "Power_CAP.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "IMU_Task.h"

// 设备数据实例：ISR 写 / 任务读（裸全局 volatile）
Dart_Rack_Motor_Group_t dart_rack_motors = {0};

Referee_Data_t Referee;
static uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];
DBUS_Typedef DBUS = {0};
static uint8_t DBUS_RX_DATA[18];

/* ================= 链接器段自动注册 ================= */

UART_RX_NODE(&huart3, 100000,0,0, 18, DBUS_RX_DATA, NULL, 18, &DBUS, DBUS_Resolved);
OFFLINE_NODE(&DBUS.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart1, 115200,0,0, 0, Referee_Rx_Buf[0], Referee_Rx_Buf[1], REFEREE_RXFRAME_LENGTH, &Referee, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(CAN1, 0x206, &dart_rack_motors.DJI_3508_SPRING, DJI_Motor_Resolve);
OFFLINE_NODE(&dart_rack_motors.DJI_3508_SPRING.offline, MOTOR_OFFLINE_TIME, SHOOT);

CAN_RX_NODE(CAN2, 0x201, &dart_rack_motors.DJI_2006_Yaw, DJI_Motor_Resolve);
OFFLINE_NODE(&dart_rack_motors.DJI_2006_Yaw.offline, MOTOR_OFFLINE_TIME, GIMBAL);

CAN_RX_NODE(CAN1, 0x209, &dart_rack_motors.DJI_6020_Turn, DJI_Motor_Resolve);
OFFLINE_NODE(&dart_rack_motors.DJI_6020_Turn.offline, MOTOR_OFFLINE_TIME, GIMBAL);