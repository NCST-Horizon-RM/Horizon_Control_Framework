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
#include "Robot_Cmd.h"
#include "Message_Center.h"

Chassis_Motor_Group_t chassis_motors;
Gimbal_Motor_Group_t  gimbal_motors;

// 设备数据实例:数据中心私有,不对外暴露,仅经发布订阅访问
static Referee_Data_t Referee;
static uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];
static VT13_Typedef VT13 = {0};
static uint8_t VT13_RX_DATA[21];
static DBUS_Typedef DBUS = {0};
static uint8_t DBUS_RX_DATA[18];
static Cap_t cap;

UART_RX_NODE(&huart3, 100000, 18, DBUS_RX_DATA, NULL, 18, &DBUS, DBUS_Resolved);
OFFLINE_NODE(&DBUS.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart6, 921600, 0, VT13_RX_DATA, NULL, 13, &VT13, VT13_Resolved);
OFFLINE_NODE(&VT13.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

UART_RX_NODE(&huart1, 115200, 0, Referee_Rx_Buf[0], Referee_Rx_Buf[1], REFEREE_RXFRAME_LENGTH, &Referee, Referee_System_Frame_Update);
OFFLINE_NODE(&Referee.offline, REFEREE_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(CAN1, 0x288, &cap, Power_Cap_Rx);
OFFLINE_NODE(&cap.get.offline, CAP_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(CAN1, 0x231, &b2b_rx_data, DualBoard_CAN_Rx_Callback);


CAN_RX_NODE(CAN1, 0x301,&gimbal_motors.DM4310_Yaw, DM_1to4_Resolve);
OFFLINE_NODE(&gimbal_motors.DM4310_Yaw.offline, MOTOR_OFFLINE_TIME, GIMBAL);

CAN_RX_NODE(CAN2, 0x201, &chassis_motors.DJI_3508_Chassis[0], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[0].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(CAN2, 0x202, &chassis_motors.DJI_3508_Chassis[1], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[1].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(CAN2, 0x203, &chassis_motors.DJI_3508_Chassis[2], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[2].offline, MOTOR_OFFLINE_TIME, CHASSIS);

CAN_RX_NODE(CAN2, 0x204, &chassis_motors.DJI_3508_Chassis[3], DJI_Motor_Resolve);
OFFLINE_NODE(&chassis_motors.DJI_3508_Chassis[3].offline, MOTOR_OFFLINE_TIME, CHASSIS);

// 将设备数据实例注册到数据中心,供上层订阅读取
void Robot_Config_Init(void)
{
    PubRegister("dbus_data",    &DBUS,    sizeof(DBUS_Typedef));
    PubRegister("vt13_data",    &VT13,    sizeof(VT13_Typedef));
    PubRegister("referee_data", &Referee, sizeof(Referee_Data_t));
    PubRegister("cap_data",     &cap,     sizeof(Cap_t));
}