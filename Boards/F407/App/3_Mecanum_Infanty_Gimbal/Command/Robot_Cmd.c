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
#include "IMU_Task.h"
#include "dsp/fast_math_functions.h"

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

// --- Pub/Sub 句柄 ---
static Subscriber_t *sys_state_sub;
static Subscriber_t *vt13_sub;
static Subscriber_t *gimbal_motors_sub;
static Subscriber_t *shoot_motors_sub;
static Subscriber_t *imu_sub;

static Publisher_t *chassis_cmd_pub;
static Publisher_t *gimbal_cmd_pub;
static Publisher_t *shoot_cmd_pub;

// --- 本地静态内存缓存 ---
static System_State_t cmd_sys_state;
static VT13_Typedef vt13_data;
static Gimbal_Motor_Group_t gimbal_motors_data;
static Shoot_Motor_Group_t shoot_motors_data;
static IMU_Data_t imu_data;

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
    vt13_sub     = SubRegister("vt13_data", sizeof(VT13_Typedef));
    gimbal_motors_sub = SubRegister("gimbal_motors", sizeof(Gimbal_Motor_Group_t));
    shoot_motors_sub = SubRegister("shoot_motors",sizeof(Shoot_Motor_Group_t));
    imu_sub = SubRegister("imu_data", sizeof(IMU_Data_t));
    chassis_cmd_pub = PubRegister("chassis_cmd", &chassis_cmd, sizeof(Chassis_Cmd_t));
    gimbal_cmd_pub  = PubRegister("gimbal_cmd", &gimbal_cmd, sizeof(Gimbal_Cmd_t));
    shoot_cmd_pub   = PubRegister("shoot_cmd", &shoot_cmd, sizeof(Shoot_Cmd_t));
}

void Robot_Cmd_Update(void)
{
    if (sys_state_sub) SubGetMessage(sys_state_sub, &cmd_sys_state);
    if (vt13_sub)     SubGetMessage(vt13_sub, &vt13_data);
    if (gimbal_motors_sub) SubGetMessage(gimbal_motors_sub,&gimbal_motors_data);
    if (shoot_motors_sub) SubGetMessage(shoot_motors_sub,&shoot_motors_data);
    if (imu_sub)      SubGetMessage(imu_sub,&imu_data);

    System_State_Report_Remote(vt13_data.offline.is_online);//向系统状态模块传入遥控器在线状态

    if (cmd_sys_state.error.bit.remote_lost)
    {
        Cmd_Handle_Safe_Mode();
    }
    else if (vt13_data.Ctrl_Mode == 1) {
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
    chassis_cmd.target_vx = (float)vt13_data.Remote.Channel [1] * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vy = -(float)vt13_data.Remote.Channel[0] * RC_ROCKER_XY_COEF;
    chassis_cmd.target_vw =-(float)vt13_data.Remote.wheel * RC_ROCKER_VW_COEF;
    //云台
    gimbal_cmd.mode = GIMBAL_CMD_MANUAL;
    gimbal_cmd.target_yaw_rate = -(float)vt13_data.Remote.Channel [3]*RC_YAW_COEF;
    gimbal_cmd.target_yaw += gimbal_cmd.target_yaw_rate;
    gimbal_cmd.target_yaw = normalize_to_pi(gimbal_cmd.target_yaw * DEG2RAD) * RAD2DEG;

    gimbal_cmd.target_pitch_rate = (float)vt13_data.Remote.Channel [2]*RC_PITCH_COEF;
    gimbal_cmd.target_pitch += gimbal_cmd.target_pitch_rate;
    gimbal_cmd.target_pitch = MATH_Limit_float(gimbal_cmd.target_pitch, -13.0f, 31.0f);
    if (vt13_data.Remote.mode_sw == 2) {
        gimbal_cmd.mode = GIMBAL_CMD_AUTO_AIM;
        gimbal_cmd.target_yaw_rate =0;
        gimbal_cmd.target_yaw +=0;
        gimbal_cmd.target_yaw = normalize_to_pi(gimbal_cmd.target_yaw * DEG2RAD) * RAD2DEG;

        gimbal_cmd.target_pitch_rate = 0;
        gimbal_cmd.target_pitch += 0;
        gimbal_cmd.target_pitch = MATH_Limit_float(gimbal_cmd.target_pitch, -13.0f, 31.0f);
    }

    //发射
    shoot_cmd.mode = SHOOT_CMD_READY;
    shoot_cmd.heat_max = b2b_rx_data.bits.heat_large;
    shoot_cmd.heat_now = b2b_rx_data.bits.heat_last;
    shoot_cmd.cool = b2b_rx_data.bits.cooling;
    shoot_cmd.trigger_single = (vt13_data.Remote.fn_1==1 && shoot_cmd.last_fn1==0);
    shoot_cmd.trigger_auto   = (vt13_data.Remote.fn_2==1||vt13_data.Remote.trigger==1);
    if (vt13_data.Remote.mode_sw != 0) {
        shoot_cmd.mode = SHOOT_CMD_RUN;
        if (shoot_cmd.trigger_single || shoot_cmd.trigger_auto)
        {
            shoot_cmd.mode = SHOOT_CMD_FIRE;
        }
    }
    shoot_cmd.last_fn1 = vt13_data.Remote.fn_1;
}

/**
 * @brief 键鼠模式
 */
static void Cmd_Update_Mouse_Key(void)
{
    chassis_cmd.target_vx = (float)(vt13_data.KeyBoard.W - vt13_data.KeyBoard.S)* KB_WASD_COEF;
    chassis_cmd.target_vy = (float)(vt13_data.KeyBoard.D - vt13_data.KeyBoard.A)* KB_WASD_COEF;
    chassis_cmd.target_vw = (float)(-vt13_data.KeyBoard.Shift *KB_VW_COEF);
    gimbal_cmd.target_yaw   -=(float)(vt13_data.KeyBoard.E- vt13_data.KeyBoard.Q ) * KB_YAW_COEF+(vt13_data.Mouse.X_Flt)*MOUSE_YAW_COEF;
    gimbal_cmd.target_pitch -=(float)(vt13_data.Mouse.Y_Flt *MOUSE_PITCH_COEF) ;

}

/**
 * @brief 双板数据同步逻辑
 */

static void Cmd_DualBoard_Sync(void)
{
    // 放大并四舍五入取整
    int32_t int_vx = (int32_t)roundf(chassis_cmd.target_vx * 100.0f);
    int32_t int_vy = (int32_t)roundf(chassis_cmd.target_vy * 100.0f);
    int32_t int_vr = (int32_t)roundf(chassis_cmd.target_vw * 100.0f);
    int32_t int_pitch = (int32_t)roundf(imu_data.pitch);

    int_vx = MATH_Limit_int16(int_vx, -1024, 1023);
    int_vy = MATH_Limit_int16(int_vy, -1024, 1023);
    int_vr = MATH_Limit_int16(int_vr, -1024, 1023);
    int_pitch = MATH_Limit_int16(int_pitch, -16, 15);

    uint32_t u_vx    = (uint32_t)int_vx    & 0x07FF;
    uint32_t u_vy    = (uint32_t)int_vy    & 0x07FF;
    uint32_t u_vr    = (uint32_t)int_vr    & 0x07FF;
    uint32_t u_pitch = (uint32_t)int_pitch & 0x001F;

    uint8_t k_q       = vt13_data.KeyBoard.Q      & 0x01;
    uint8_t k_e       = vt13_data.KeyBoard.E      & 0x01;
    uint8_t k_v       = vt13_data.KeyBoard.V      & 0x01;
    uint8_t k_shift   = vt13_data.KeyBoard.Shift  & 0x01;
    uint8_t k_ctrl    = vt13_data.KeyBoard.Ctrl   & 0x01;
    uint8_t rc_online = vt13_data.offline.is_online & 0x03;
    uint8_t rc_s1     = vt13_data.Remote.fn_1     & 0x03;
    uint8_t rc_s2     = vt13_data.Remote.fn_2     & 0x03;
    uint8_t f_wheel   = Is_Group_Online(SHOOT)    & 0x01;
    uint8_t g_lixian  = Is_Group_Online(GIMBAL)   & 0x01;
    uint8_t v_look    = 0 & 0x01;
    uint8_t vision    = 0 & 0x01;
    uint32_t surplus  = 0 & 0x01FF; // 9位

    uint8_t temp_buf[8] = {0};
    // --- Byte 0 ---
    temp_buf[0] = (uint8_t)(u_vx & 0xFF);
    // --- Byte 1 ---
    temp_buf[1] = (uint8_t)(((u_vx >> 8) & 0x07) | ((u_vy << 3) & 0xF8));
    // --- Byte 2 ---
    temp_buf[2] = (uint8_t)(((u_vy >> 5) & 0x3F) | ((u_vr << 6) & 0xC0));
    // --- Byte 3 ---
    temp_buf[3] = (uint8_t)((u_vr >> 2) & 0xFF);
    // --- Byte 4 ---
    temp_buf[4] = (uint8_t)(((u_vr >> 10) & 0x01) |
                            (k_q << 1)            |
                            (k_e << 2)            |
                            (k_v << 3)            |
                            (k_shift << 4)        |
                            (k_ctrl << 5)         |
                            (rc_online << 6));
    // --- Byte 5 ---
    temp_buf[5] = (uint8_t)((rc_s1 & 0x03) |
                            ((rc_s2 & 0x03) << 2) |
                            ((u_pitch << 4) & 0xF0));
    // --- Byte 6 ---
    temp_buf[6] = (uint8_t)(((u_pitch >> 4) & 0x01) |
                            (f_wheel << 1)          |
                            (g_lixian << 2)         |
                            (v_look << 3)           |
                            (vision << 4)           |
                            ((surplus << 5) & 0xE0));
    // --- Byte 7 ---
    temp_buf[7] = (uint8_t)((surplus >> 3) & 0x3F);
    memcpy(b2b_tx_data.buf, temp_buf, 8);
    CAN_Send_Msg(&hcan1, 0x231, b2b_tx_data.buf, 8);
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