//
// Created by CaoKangqi on 2026/6/23.
//
#include "Robot_Cmd.h"
#include "Robot_Config.h"
#include "System_State.h"
#include "DBUS.h"
#include "Aim_Vision.h"
#include "All_define.h"
#include "Horizon_MATH.h"
#include "Comm_DualBoard.h"
#include "Referee.h"
#include "DualBoard_Frame.h"

#define RC_ROCKER_XY_COEF      0.003f  // 摇杆控制平移的增益
#define RC_ROCKER_VW_COEF      0.01f   // 摇杆控制自旋的增益
#define RC_LENGTH_COEF          0.000007f
#define LEG_LENGTH_TARGET_M  0.340f
#define YAW_ZERO               4797

// --- 本地静态内存缓存 ---

Chassis_Cmd_t chassis_cmd = {0};
C2G_t c2g;


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

    System_State_Report_Remote(g2c.remoteOnLine);//向系统状态模块传入遥控器在线状态

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
    if (g2c.S2 != 2) {
        chassis_cmd.mode = CHASSIS_CMD_SAFE;
    }
    else {
        chassis_cmd.target_length += (float)g2c.CH3 * RC_LENGTH_COEF;
        chassis_cmd.target_length=MATH_Limit_float(chassis_cmd.target_length,0.17f,0.32f);
        chassis_cmd.target_vx = (float)g2c.CH1 * RC_ROCKER_XY_COEF;
        chassis_cmd.target_roll = (float)g2c.CH0 * 0.00015f ;

        int16_t relative_angle = YAW_ZERO - gimbal_motors.DM4310_Yaw.Angle_now;
        chassis_cmd.offset_angle = normalize_to_pi((float)relative_angle * ENCODER_TO_RAD);
        chassis_cmd.mode = CHASSIS_CMD_FOLLOW;
        if (g2c.Dial != 0) {
            chassis_cmd.mode = CHASSIS_CMD_SPIN;
            chassis_cmd.target_vw = (float)g2c.Dial * RC_ROCKER_VW_COEF;
        }
    }
    chassis_cmd.mode_last = chassis_cmd.mode;
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
    c2g.heat_last  = Referee.power_heat_data.shooter_17mm_barrel_heat;
    c2g.cooling    = Referee.robot_status.shooter_barrel_cooling_value;
    c2g.level      = Referee.robot_status.robot_level;
    c2g.initial_s  = (uint8_t)roundf(Referee.shoot_data.initial_speed * 10);
    c2g.robot_HP   = Referee.robot_status.current_HP;
    c2g.heat_large = Referee.robot_status.shooter_barrel_heat_limit;
    c2g.self_color = (Referee.robot_status.robot_id == 103) ? 1 : 0;

    uint8_t buf[8];
    C2G_pack(&c2g, buf);
    CAN_Send_Msg(&hfdcan1, 0x232, buf, 8);
}

/**
 * @brief 双板通信接收回调 (解算 Protocol_Rx_t)
 * @note  必须挂载到 CAN Rx FIFO 中断的回调函数中
 * @param device_ptr CAN设备指针(hcan)
 * @param data 接收到的8字节数据指针
 */
void DualBoard_CAN_Rx_Callback(void *instance, uint8_t *data)
{
    if (instance == NULL || data == NULL) return;
    G2C_unpack(data, (G2C_t *)instance);
}
