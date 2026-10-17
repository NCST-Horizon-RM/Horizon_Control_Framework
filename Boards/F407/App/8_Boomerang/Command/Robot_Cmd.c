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
#include "usbd_cdc_if.h"

#define VISION_COMPENSATION       193      //视觉补偿
// #define PITCH_MAX              25.0f
// #define PITCH_MIN             -20.0f
// #define FRICTION_MAX_RPM       6500.0f
// #define FRICTION_RAMP_STEP     1.7f    //摩擦轮缓启动时长
//
 #define RC_ROCKER_SPRING_COEF      4  // 摇杆控制平移的增益
// #define RC_ROCKER_VW_COEF      0.02f   // 摇杆控制自旋的增益
// #define RC_PITCH_COEF          0.001f
// #define RC_YAW_COEF            0.006f
//
// #define KB_WASD_COEF           1.0f    // 键盘 WASD 速度增益
// #define MOUSE_PITCH_COEF       0.06f
// #define MOUSE_YAW_COEF         0.04f
//
// #define YAW_ZERO               1100

// --- 本地静态内存缓存 ---
Chassis_Cmd_t chassis_cmd = {0};
Gimbal_Cmd_t gimbal_cmd = {0};
Shoot_Cmd_t shoot_cmd = {0};
Door_Cmd_t door_cmd = {0};
Vision_Tx_Cmd_t vision_tx_cmd = {0};
Vision_Rx_Cmd_t vision_rx_cmd = {0};
// --- 私有函数声明 ---
static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Referee_Ctrl(void);
static void Cmd_Update_Vision_Ctrl_Yaw(void);
static void Cmd_Vision_Sync(void);
static void vision_isline();


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
    if (Referee.game_status.game_progress > 1 &&
        Referee.game_status.game_progress < 6)
    {
        door_cmd.mode = DOOR_CMD_GAMEING;
        Cmd_Update_Referee_Ctrl();
    }else
    {
        door_cmd.mode = DOOR_CMD_GAMEOVER;
        Cmd_Update_Remote_Ctrl();
    }

    // 视觉数据
    Cmd_Vision_Sync();
    //视觉离线检测
    vision_isline();
    //视觉追踪
    Cmd_Update_Vision_Ctrl_Yaw();
}

/**
 * @brief 安全模式清除物理输出
 */
static void Cmd_Handle_Safe_Mode(void)
{
    chassis_cmd.mode = CHASSIS_CMD_SAFE;
    gimbal_cmd.mode  = GIMBAL_CMD_SAFE;
    shoot_cmd.mode   = SHOOT_CMD_SAFE;
    vision_rx_cmd.mode  = VISION_CMD_SAFE;

    shoot_cmd.target_spring  = 0;
}

/**
 * @brief 遥控器模式
 */
static void Cmd_Update_Remote_Ctrl(void)
{
    chassis_cmd.mode = CHASSIS_CMD_WORK;
    gimbal_cmd.mode = GIMBAL_CMD_WORK;
    shoot_cmd.mode = SHOOT_CMD_REMOTE;

    shoot_cmd.target_spring = DBUS.Remote.CH1 * RC_ROCKER_SPRING_COEF;
}

/**
 * @brief 裁判系统自动模式
 */
static void Cmd_Update_Referee_Ctrl(void)
{
    static uint16_t dart_launch_opening_status_last = 0;
    if (Referee.game_status.game_progress == 4)
    {
        chassis_cmd.mode = CHASSIS_CMD_WORK;
        gimbal_cmd.mode = GIMBAL_CMD_WORK;
        shoot_cmd.mode = SHOOT_CMD_AUTO;
        if (Referee.dart_client_cmd.dart_launch_opening_status == 2 &&
            dart_launch_opening_status_last == 0)
        {
            ++door_cmd.open_cnt;
        }
        dart_launch_opening_status_last = Referee.dart_client_cmd.dart_launch_opening_status;
    }

    switch (door_cmd.open_cnt)
    {
        case 1:
        door_cmd.mode = DOOR_CMD_ROUND1;
        break;
        case 2:
        door_cmd.mode = DOOR_CMD_ROUND2;
        break;
        default:
        break;
    }
}

/**
 * @brief 视觉追踪独立函数
 */
static void Cmd_Update_Vision_Ctrl_Yaw(void)
{
    switch (vision_rx_cmd.mode)
    {
        case VISION_CMD_OFFLINE:
        break;
        case VISION_CMD_ONLINE:
        chassis_cmd.mode = GIMBAL_CMD_WORK;
        chassis_cmd.target_yaw = 0;
        break;
        case VISION_CMD_SAFE:
        break;
    }
}

/**
 * @brief 视觉数据同步逻辑
 */
static void Cmd_Vision_Sync(void)
{
    uint8_t i = 0;

    vision_tx_cmd.Head_frame = 0xCD;
    vision_tx_cmd.VisionState = Referee.dart_info.dart_selected_target;
    vision_tx_cmd.End_frame = 0xDC;


    vision_tx_cmd.data[i++] = vision_tx_cmd.Head_frame;

    vision_tx_cmd.data[i++] = (uint8_t)(vision_tx_cmd.VisionState & 0xFF);
    vision_tx_cmd.data[i++] = (uint8_t)((vision_tx_cmd.VisionState >> 8) & 0xFF);

    vision_tx_cmd.data[i++] = vision_tx_cmd.End_frame;

    CDC_Transmit_FS(vision_tx_cmd.data, sizeof(vision_tx_cmd.data));
}

/**
 * @brief 双板通信接收回调 (解算 Protocol_Rx_t)
 * @note  必须挂载到 CAN Rx FIFO 中断的回调函数中
 * @param device_ptr CAN设备指针(hcan)
 * @param data 接收到的8字节数据指针
 */
void Vision_CAN_Rx_Callback(void *instance, uint8_t *data)
{

}

static void vision_isline()
{
    vision_rx_cmd.Data.OffCounter++;
    if ( vision_rx_cmd.Data.OffCounter > 80)
    {
        vision_rx_cmd.Data.OffCounter = 80;
        vision_rx_cmd.mode = VISION_CMD_OFFLINE;
    }

    else
    {
            if (vision_rx_cmd.Data.x1 == 0 && vision_rx_cmd.Data.x2 == 0)
            {
                vision_rx_cmd.mode = VISION_CMD_OFFLINE;
            }
        vision_rx_cmd.mode = VISION_CMD_ONLINE;
    }

    vision_rx_cmd.mode = VISION_CMD_SAFE;
}