//
// Created by CaoKangqi on 2026/6/23.
//
#include "Robot_Cmd.h"

#include "Aim_Vision.h"
#include "System_State.h"
#include "All_define.h"
#include "BSP_CAN.h"
#include "Robot_Config.h"
#include "IMU_Task.h"
#include "DualBoard_Frame.h"

#define PITCH_MAX              25.0f
#define PITCH_MIN             -20.0f
#define FRICTION_MAX_RPM       6500.0f
#define FRICTION_RAMP_STEP     1.7f    //摩擦轮缓启动时长

#define RC_ROCKER_XY_COEF      0.004f  // 摇杆控制平移的增益
#define RC_ROCKER_VW_COEF      0.02f   // 摇杆控制自旋的增益
#define RC_PITCH_COEF          0.001f
#define RC_YAW_COEF            0.002f

#define KB_WASD_COEF           330.0f    // 键盘 WASD 速度增益
#define KB_VW_COEF             660.0f
#define MOUSE_PITCH_COEF       0.06f
#define MOUSE_YAW_COEF         0.04f
#define KB_YAW_COEF            2.0f

#define YAW_ZERO               5100

Chassis_Cmd_t chassis_cmd = {0};
Gimbal_Cmd_t gimbal_cmd = {0};
Shoot_Cmd_t shoot_cmd = {0};
//双板通讯
G2C_t g2c;
//自瞄
Vision_Send_t vision_Send;
// --- 私有函数声明 ---
static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_DualBoard_Sync(void);


void Robot_Cmd_Init(void)
{

}

void Robot_Cmd_Update(void)
{

    System_State_Report_Remote(DBUS.offline.is_online);//向系统状态模块传入遥控器在线状态

    if (sys_state.error.bit.remote_lost)
    {
        Cmd_Handle_Safe_Mode();
    }
    else if (DBUS.Ctrl_Mode == 1) {
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

    shoot_cmd.trigger_single = false;
    shoot_cmd.trigger_auto   = false;
    shoot_cmd.bullet_speed = 0.0f;
}

/**
 * @brief 遥控器模式
 */
static void Cmd_Update_Remote_Ctrl(void)
{
    // 底盘
    chassis_cmd.target_vx = (float)DBUS.Remote.CH1 * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vy = -(float)DBUS.Remote.CH0 * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vw =-(float)DBUS.Remote.Dial * RC_ROCKER_VW_COEF;
    //云台
    if (vision_Recv.target_found==1 && DBUS.Remote.S1!=1) {
        gimbal_cmd.mode = GIMBAL_CMD_AUTO_AIM;
        gimbal_cmd.target_yaw_rate = vision_Recv.yaw_plan * DEG2RAD;
        gimbal_cmd.target_yaw = -vision_Recv.yaw;
        gimbal_cmd.target_yaw = normalize_to_pi(gimbal_cmd.target_yaw * DEG2RAD) * RAD2DEG;

        gimbal_cmd.target_pitch_rate = vision_Recv.pitch_plan * DEG2RAD;
        gimbal_cmd.target_pitch = -vision_Recv.pitch;
        gimbal_cmd.target_pitch = MATH_Limit_float(gimbal_cmd.target_pitch, -13.0f, 31.0f);
    }
    else{
        gimbal_cmd.mode = GIMBAL_CMD_MANUAL;
        gimbal_cmd.target_yaw_rate = -(float)DBUS.Remote.CH2*RC_YAW_COEF;
        gimbal_cmd.target_yaw += gimbal_cmd.target_yaw_rate;
        gimbal_cmd.target_yaw = normalize_to_pi(gimbal_cmd.target_yaw * DEG2RAD) * RAD2DEG;

        gimbal_cmd.target_pitch_rate = (float)DBUS.Remote.CH3*RC_PITCH_COEF;
        gimbal_cmd.target_pitch -= gimbal_cmd.target_pitch_rate;
        gimbal_cmd.target_pitch = MATH_Limit_float(gimbal_cmd.target_pitch, -13.0f, 31.0f);

    }

    //发射
    shoot_cmd.mode = SHOOT_CMD_READY;
    /*shoot_cmd.heat_max = C2G.heat_large;
    shoot_cmd.heat_now = C2G.heat_last;
    shoot_cmd.cool = C2G.cooling;
    shoot_cmd.trigger_single = (VT13.Remote.fn_1==1 && shoot_cmd.last_fn1==0);
    shoot_cmd.trigger_auto   = (VT13.Remote.fn_2==1||VT13.Remote.trigger==1);
    if (VT13.Remote.mode_sw != 0) {
        shoot_cmd.mode = SHOOT_CMD_RUN;
        if (shoot_cmd.trigger_single || shoot_cmd.trigger_auto)
        {
            shoot_cmd.mode = SHOOT_CMD_FIRE;
        }
    }
    shoot_cmd.last_fn1 = VT13.Remote.fn_1;*/
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
uint8_t vision_buf[20];
static void Cmd_DualBoard_Sync(void)
{
    // 放大并四舍五入取整
    int32_t int_vx = (int32_t)roundf(chassis_cmd.target_vx * 100.0f);
    int32_t int_vy = (int32_t)roundf(chassis_cmd.target_vy * 100.0f);
    int32_t int_vr = (int32_t)roundf(chassis_cmd.target_vw * 100.0f);
    int32_t int_pitch = (int32_t)roundf(IMU_Data.pitch);

    int_vx = MATH_Limit_int16(int_vx, -1024, 1023);
    int_vy = MATH_Limit_int16(int_vy, -1024, 1023);
    int_vr = MATH_Limit_int16(int_vr, -1024, 1023);
    int_pitch = MATH_Limit_int16(int_pitch, -16, 15);

    g2c.vx    = (int16_t)int_vx;
    g2c.vy    = (int16_t)int_vy;
    g2c.vr    = (int16_t)int_vr;
    g2c.pitch = (int8_t)int_pitch;

    g2c.key_q         = DBUS.KeyBoard.Q;
    g2c.key_e         = DBUS.KeyBoard.E;
    g2c.key_v         = DBUS.KeyBoard.V;
    g2c.key_shift     = DBUS.KeyBoard.Shift;
    g2c.key_ctrl      = DBUS.KeyBoard.Ctrl;

    g2c.romoteOnLine  = DBUS.offline.is_online;
    g2c.S1            = DBUS.Remote.S1;
    g2c.S2            = DBUS.Remote.S2;

    g2c.fire_wheel    = Is_Group_Online(SHOOT);
    g2c.gimbal_lixian = Is_Group_Online(GIMBAL);

    uint8_t buf[8];
    G2C_pack(&g2c, buf);
    CAN_Send_Msg(&hcan1, 0x231, buf, 8);


    vision_Send.pitch = -IMU_Data.pitch;
    vision_Send.yaw = -IMU_Data.yaw;
    vision_Send.pitch_omega = -IMU_Data.gyro[1];
    vision_Send.yaw_omega = -IMU_Data.gyro[2];
    vision_Send.mode = 0;
    vision_Send.bullet_speed = 0;

    Vision_Encode(&vision_Send, vision_buf);
    HAL_UART_Transmit_DMA(&huart1, vision_buf, 20);
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
    C2G_unpack(data, (C2G_t *)instance);
}

void Vision_UART_Rx_Callback(uint8_t* Data, void *device_ptr, uint16_t size) {
    if (device_ptr == NULL) return;
    Vision_Decode( Data, &vision_Recv);
}