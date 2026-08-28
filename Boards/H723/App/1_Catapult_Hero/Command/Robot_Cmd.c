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

#define PITCH_MAX              25.0f
#define PITCH_MIN             -20.0f
#define FRICTION_MAX_RPM       6500.0f
#define FRICTION_RAMP_STEP     1.7f    //摩擦轮缓启动时长

#define RC_ROCKER_XY_COEF      0.004f  // 摇杆控制平移的增益
#define RC_ROCKER_VW_COEF      0.02f   // 摇杆控制自旋的增益
#define RC_PITCH_COEF          0.001f
#define RC_YAW_COEF            0.006f

#define KB_WASD_COEF           1.0f    // 键盘 WASD 速度增益
#define MOUSE_PITCH_COEF       0.06f
#define MOUSE_YAW_COEF         0.04f

// --- 本地静态内存缓存 ---

Chassis_Cmd_t chassis_cmd = {0};
Gimbal_Cmd_t gimbal_cmd = {0};
Shoot_Cmd_t shoot_cmd = {0};


// --- 私有函数声明 ---
static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_DualBoard_Sync(void);


void Robot_Cmd_Init(void)
{
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
    gimbal_cmd.mode  = GIMBAL_CMD_SAFE;
    shoot_cmd.mode   = SHOOT_CMD_SAFE;

    chassis_cmd.target_vx = 0.0f;
    chassis_cmd.target_vy = 0.0f;
    chassis_cmd.target_vw = 0.0f;

    shoot_cmd.friction_rpm   = 0.0f;
    shoot_cmd.trigger_single = false;
    shoot_cmd.trigger_auto   = false;
}

/**
 * @brief 遥控器模式
 */
static void Cmd_Update_Remote_Ctrl(void)
{
    chassis_cmd.target_vx = (float)DBUS.Remote.CH1 * RC_ROCKER_XY_COEF + (float)VT13.Remote.Channel[1] * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vy = (float)DBUS.Remote.CH0 * RC_ROCKER_XY_COEF + (float)VT13.Remote.Channel[0] * RC_ROCKER_XY_COEF;
    float active_vw       = (float)DBUS.Remote.CH2 * RC_ROCKER_VW_COEF + (float)VT13.Remote.Channel[3] * RC_ROCKER_VW_COEF;
    gimbal_cmd.target_yaw   += (float)DBUS.Remote.CH3 * RC_YAW_COEF + (float)VT13.Remote.Channel[2] * RC_YAW_COEF;

    if (VT13.Remote.mode_sw == 1 && VT13.Remote.fn_2 == 1) {
        shoot_cmd.mode = SHOOT_CMD_FIRE;
        shoot_cmd.trigger_single = true;
    }else {
        shoot_cmd.trigger_single = false;
    }
    chassis_cmd.mode = CHASSIS_CMD_FREE;
    shoot_cmd.mode = SHOOT_CMD_READY;
    chassis_cmd.target_vw = active_vw;

}

/**
 * @brief 键鼠模式
 */
static void Cmd_Update_Mouse_Key(void)
{
    chassis_cmd.target_vx = (DBUS.KeyBoard.W - DBUS.KeyBoard.S) * KB_WASD_COEF;
    chassis_cmd.target_vy = (DBUS.KeyBoard.D - DBUS.KeyBoard.A) * KB_WASD_COEF;
    float active_vw       = (DBUS.KeyBoard.E - DBUS.KeyBoard.Q) * 3.0f + DBUS.Mouse.X_Flt * RC_ROCKER_VW_COEF;

    if (DBUS.KeyBoard.Shift) {
        chassis_cmd.mode = CHASSIS_CMD_SPIN;
        chassis_cmd.target_vw = 5.0f;
    } else if (active_vw != 0.0f) {
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vw = active_vw;
    } else {
        chassis_cmd.mode = CHASSIS_CMD_FOLLOW;
        chassis_cmd.target_vw = 0.0f;
    }

}

/**
 * @brief 双板数据同步逻辑
 */
static void Cmd_DualBoard_Sync(void)
{

}