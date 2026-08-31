//
// Created by CaoKangqi on 2026/6/23.
//
#include "Robot_Cmd.h"
#include "System_State.h"
#include "DBUS.h"
#include "All_define.h"
#include "BSP_CAN.h"
#include "Referee.h"
#include "Robot_Config.h"
#include "VT13.h"
#include "dsp/fast_math_functions.h"
#include "DualBoard_Frame.h"

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

#define YAW_ZERO               1190

// --- 本地静态内存缓存 ---

Chassis_Cmd_t chassis_cmd = {0};
Gimbal_Cmd_t gimbal_cmd = {0};
Shoot_Cmd_t shoot_cmd = {0};
//双板通讯
C2G_t c2g = {0};
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

    System_State_Report_Remote(g2c.romoteOnLine);//向系统状态模块传入遥控器在线状态

    if (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        sys_state.global_mode == GLOBAL_MODULE_ERROR ||
        sys_state.global_mode == GLOBAL_STANDBY)
    {
        Cmd_Handle_Safe_Mode();
    }
    Cmd_Update_Remote_Ctrl();

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
    int16_t relative_angle = YAW_ZERO - gimbal_motors.DM4310_Yaw.Angle_now;
    chassis_cmd.offset_angle = normalize_to_pi((float)relative_angle * ENCODER_TO_RAD);;

    chassis_cmd.target_vx = (float)g2c.vx * 0.01f;
    chassis_cmd.target_vy = (float)g2c.vy * 0.01f;
    chassis_cmd.mode = CHASSIS_CMD_FOLLOW;
    if (g2c.vr != 0) {
        chassis_cmd.mode = CHASSIS_CMD_SPIN;
        chassis_cmd.target_vw = (float)g2c.vr * 0.01f;
    }
    chassis_cmd.is_cap_on = true;
}

/**
 * @brief 键鼠模式
 */
static void Cmd_Update_Mouse_Key(void)
{
    chassis_cmd.target_vx = (float)g2c.vx * KB_WASD_COEF;
    chassis_cmd.target_vy = (float)g2c.vy * KB_WASD_COEF;
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
    chassis_cmd.is_cap_on = g2c.key_v;
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
    CAN_Send_Msg(&hcan1, 0x232, buf, 8);
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