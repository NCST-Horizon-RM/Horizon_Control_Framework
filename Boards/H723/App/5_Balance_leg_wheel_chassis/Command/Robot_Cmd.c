//
// Created by CaoKangqi on 2026/6/23.
//
#include "Robot_Cmd.h"
#include "Robot_Config.h"
#include "System_State.h"
#include "DBUS.h"
#include "Aim_Vision.h"
#include "All_define.h"
#include "BSP_UART.h"
#include "Horizon_MATH.h"
#include "Comm_DualBoard.h"
#include "Referee.h"
#include "usart.h"
#include "VT13.h"

#define RC_ROCKER_XY_COEF      0.004f  // 摇杆控制平移的增益
#define RC_ROCKER_VW_COEF      0.005f   // 摇杆控制自旋的增益
#define RC_ROLL_COEF            0.0005f
#define RC_LENGTH_COEF          0.000005f
#define LEG_LENGTH_TARGET_M  0.340f

// --- 本地静态内存缓存 ---

Chassis_Cmd_t chassis_cmd = {0};



// --- 私有函数声明 ---
static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_DualBoard_Sync(void);


void Robot_Cmd_Init(void)
{
    chassis_cmd.target_length = LEG_LENGTH_TARGET_M;
    // topic 槽位表在 Robot_Config.c 静态装配，无需运行时注册
}

void Robot_Cmd_Update(void)
{

    System_State_Report_Remote(DBUS.offline.is_online);//向系统状态模块传入遥控器在线状态

    if (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        sys_state.global_mode == GLOBAL_MODULE_ERROR ||
        sys_state.global_mode == GLOBAL_STANDBY)
    {
        Cmd_Handle_Safe_Mode();
    }
    if (DBUS.Ctrl_Mode == 1) {
        Cmd_Update_Mouse_Key();
    }
    else {
        Cmd_Update_Remote_Ctrl();
    }


    // 双板通信
    Cmd_DualBoard_Sync();
}

/**
 * @brief 安全模式清除物理输出
 */
static void Cmd_Handle_Safe_Mode(void)
{
    chassis_cmd.mode = CHASSIS_CMD_SAFE;

    chassis_cmd.target_vx = 0.0f;
    chassis_cmd.target_vw = 0.0f;

}

/**
 * @brief 遥控器模式
 */
static void Cmd_Update_Remote_Ctrl(void)
{
    chassis_cmd.target_vx = (float)DBUS.Remote.CH1 * RC_ROCKER_XY_COEF ;//+ (float)VT13.Remote.Channel[1] * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vw = -(float)DBUS.Remote.CH2 * RC_ROCKER_VW_COEF ;//+ (float)VT13.Remote.Channel[3] * RC_ROCKER_VW_COEF;
    chassis_cmd.target_length += (float)DBUS.Remote.CH3 * RC_LENGTH_COEF;
    chassis_cmd.target_length=MATH_Limit_float(chassis_cmd.target_length,0.17f,0.35f);
    chassis_cmd.target_roll = -(float)DBUS.Remote.CH0 * RC_ROLL_COEF ;

    chassis_cmd.mode_last = chassis_cmd.mode;

    if (DBUS.Remote.S2 == 1) {
        chassis_cmd.mode = CHASSIS_CMD_SAFE;
    }else if (DBUS.Remote.S2 == 3){
        chassis_cmd.mode = CHASSIS_CMD_FREE;
    }
    else if (DBUS.Remote.S2 == 2){
        chassis_cmd.mode = CHASSIS_CMD_FOLLOW;
    }
}

/**
 * @brief 键鼠模式
 */
static void Cmd_Update_Mouse_Key(void)
{

}

/**
 * @brief 双板数据同步逻辑
 */
static void Cmd_DualBoard_Sync(void)
{

}