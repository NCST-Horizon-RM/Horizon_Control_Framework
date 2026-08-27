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


Gimbal_Motor_Group_t  gimbal_motors;
Shoot_Motor_Group_t   shoot_motors;

// 设备数据实例:数据中心私有,不对外暴露,仅经发布订阅访问
static VT13_Typedef VT13 = {0};
static uint8_t VT13_RX_DATA[21];

UART_RX_NODE(&huart6, 921600, 21, VT13_RX_DATA, NULL, 21, &VT13, VT13_Resolved);
OFFLINE_NODE(&VT13.offline, DBUS_OFFLINE_TIME, GROUP_NONE);

CAN_RX_NODE(CAN1, 0x232, &b2b_rx_data, DualBoard_CAN_Rx_Callback);


CAN_RX_NODE(CAN1, 0x301,&gimbal_motors.DM4310_Yaw, DM_1to4_Resolve);
OFFLINE_NODE(&gimbal_motors.DM4310_Yaw.offline, MOTOR_OFFLINE_TIME, GIMBAL);

CAN_RX_NODE(CAN1, 0x302,&gimbal_motors.DM4310_Pitch, DM_1to4_Resolve);
OFFLINE_NODE(&gimbal_motors.DM4310_Pitch.offline, MOTOR_OFFLINE_TIME, GIMBAL);
CAN_RX_NODE(CAN1, 0x203,&shoot_motors.DJI_2006_bo , DJI_Motor_Resolve);
OFFLINE_NODE(&shoot_motors.DJI_2006_bo .offline, MOTOR_OFFLINE_TIME, SHOOT);

CAN_RX_NODE(CAN2, 0x201,&shoot_motors.DJI_3508_L, DJI_Motor_Resolve);
OFFLINE_NODE(&shoot_motors.DJI_3508_L.offline, MOTOR_OFFLINE_TIME, SHOOT);
CAN_RX_NODE(CAN2, 0x202,&shoot_motors.DJI_3508_R , DJI_Motor_Resolve);
OFFLINE_NODE(&shoot_motors.DJI_3508_R .offline, MOTOR_OFFLINE_TIME, SHOOT);

// 将设备数据实例注册到数据中心,供上层订阅读取
void Robot_Config_Init(void)
{
    PubRegister("vt13_data", &VT13, sizeof(VT13_Typedef));
}
