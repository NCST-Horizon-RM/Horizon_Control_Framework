//
// Created by CaoKangqi on 2026/6/23.
//
#include "Robot_Cmd.h"
#include "Message_Center.h"
#include "System_State.h"
#include "DBUS.h"
#include "All_define.h"
#include "BSP_CAN.h"
#include "Referee.h"
#include "Robot_Config.h"
#include "VT13.h"
#include "dsp/fast_math_functions.h"

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

// --- Pub/Sub 句柄 ---
static Subscriber_t *sys_state_sub;
static Subscriber_t *dbus_sub;
static Subscriber_t *vt13_sub;
static Subscriber_t *referee_sub;
static Subscriber_t *gimbal_motors_sub;

static Publisher_t *chassis_cmd_pub;
static Publisher_t *gimbal_cmd_pub;
static Publisher_t *shoot_cmd_pub;

// --- 本地静态内存缓存 ---
static System_State_t cmd_sys_state;
static DBUS_Typedef dbus_data;
static VT13_Typedef vt13_data;
static Referee_Data_t referee_data;
static Gimbal_Motor_Group_t gimbal_motors_data;

static Chassis_Cmd_t chassis_cmd = {0};
static Gimbal_Cmd_t gimbal_cmd = {0};
static Shoot_Cmd_t shoot_cmd = {0};
//双板通讯
static Protocol_Tx_t b2b_tx_data;
Protocol_Rx_t b2b_rx_data;
// --- 私有函数声明 ---
static void Cmd_Handle_Safe_Mode(void);
static void Cmd_Update_Remote_Ctrl(void);
static void Cmd_Update_Mouse_Key(void);
static void Cmd_DualBoard_Sync(void);


void Robot_Cmd_Init(void)
{
    sys_state_sub = SubRegister("system_state", sizeof(System_State_t));
    dbus_sub      = SubRegister("dbus_data", sizeof(DBUS_Typedef));
    vt13_sub     = SubRegister("vt13_data", sizeof(VT13_Typedef));
    referee_sub = SubRegister("referee_data",sizeof(Referee_Data_t));
    gimbal_motors_sub = SubRegister("gimbal_motors", sizeof(Gimbal_Motor_Group_t));

    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", &gimbal_cmd, sizeof(Gimbal_Cmd_t));
    shoot_cmd_pub   = PubRegister("shoot_cmd", &shoot_cmd, sizeof(Shoot_Cmd_t));
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);
    if (dbus_sub)      SubGetMessage(dbus_sub, &dbus_data);
    if (vt13_sub)     SubGetMessage(vt13_sub, &vt13_data);
    if (referee_sub)  SubGetMessage(referee_sub,&referee_data);
    if (gimbal_motors_sub) SubGetMessage(gimbal_motors_sub,&gimbal_motors_data);

    System_State_Report_Remote(b2b_rx_data.bits.romoteOnLine);//向系统状态模块传入遥控器在线状态

    if (cmd_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
        cmd_sys_state.global_mode == GLOBAL_MODULE_ERROR ||
        cmd_sys_state.global_mode == GLOBAL_STANDBY)
    {
        Cmd_Handle_Safe_Mode();
    }
    if (dbus_data.Ctrl_Mode == 1) {
        Cmd_Update_Mouse_Key();
    }
    else {
        Cmd_Update_Remote_Ctrl();
    }

    PubPushMessage(chassis_cmd_pub, &chassis_cmd);
    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd);
    PubPushMessage(shoot_cmd_pub, &shoot_cmd);

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
    int16_t relative_angle = YAW_ZERO - gimbal_motors_data.DM4310_Yaw.Angle_now;
    chassis_cmd.offset_angle = normalize_to_pi((float)relative_angle * ENCODER_TO_RAD);;

    chassis_cmd.target_vx = (float)b2b_rx_data.bits.vx * 0.01f;
    chassis_cmd.target_vy = (float)b2b_rx_data.bits.vy * 0.01f;
    chassis_cmd.mode = CHASSIS_CMD_FOLLOW;
    if (b2b_rx_data.bits.vr != 0) {
        chassis_cmd.mode = CHASSIS_CMD_SPIN;
        chassis_cmd.target_vw = (float)b2b_rx_data.bits.vr /100.0f;
    }
    chassis_cmd.is_cap_on = true;
}

/**
 * @brief 键鼠模式
 */
static void Cmd_Update_Mouse_Key(void)
{
    chassis_cmd.target_vx = (float)b2b_rx_data.bits.vx * KB_WASD_COEF;
    chassis_cmd.target_vy = (float)b2b_rx_data.bits.vy * KB_WASD_COEF;
    float active_vw       = (dbus_data.KeyBoard.E - dbus_data.KeyBoard.Q) * 3.0f + dbus_data.Mouse.X_Flt * RC_ROCKER_VW_COEF;

    if (dbus_data.KeyBoard.Shift) {
        chassis_cmd.mode = CHASSIS_CMD_SPIN;
        chassis_cmd.target_vw = 5.0f;
    } else if (active_vw != 0.0f) {
        chassis_cmd.mode = CHASSIS_CMD_FREE;
        chassis_cmd.target_vw = active_vw;
    } else {
        chassis_cmd.mode = CHASSIS_CMD_FOLLOW;
        chassis_cmd.target_vw = 0.0f;
    }
    chassis_cmd.is_cap_on = b2b_rx_data.bits.key_v;
}

/**
 * @brief 双板数据同步逻辑
 */
static void Cmd_DualBoard_Sync(void)
{
    memset(&b2b_tx_data, 0, sizeof(b2b_tx_data));

    b2b_tx_data.bits.heat_last         = referee_data.power_heat_data.shooter_17mm_barrel_heat;
    b2b_tx_data.bits.cooling           = referee_data.robot_status.shooter_barrel_cooling_value;
    b2b_tx_data.bits.level             = referee_data.robot_status.robot_level;
    b2b_tx_data.bits.initial_s         = (uint8_t)roundf (referee_data.shoot_data.initial_speed *10); // 裁判系统在线状态
    b2b_tx_data.bits.robot_HP          = referee_data.robot_status.current_HP;
    b2b_tx_data.bits.heat_large        = referee_data.robot_status.shooter_barrel_heat_limit;
    // 2. 自身颜色判断
    if(referee_data.robot_status.robot_id == 3) { // 红方
        b2b_tx_data.bits.self_color = 0;
    }
    else if(referee_data.robot_status.robot_id == 103) { // 蓝方
        b2b_tx_data.bits.self_color = 1;
    }
    CAN_Send_Msg(&hcan1, 0x232, b2b_tx_data.buf, 8);
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
    Protocol_Rx_t *rx_ptr = (Protocol_Rx_t *)instance;

    memcpy(rx_ptr->buf, data, 8);
}