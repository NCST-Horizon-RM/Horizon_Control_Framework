/**
 * @file DBUS.c
 * @brief  DBUS 协议解析源文件
 * @version 1.0
 * @date 2026-01-19
 * @author CaoKangqi
 */

#include "DBUS.h"
#include "All_define.h"
#include "Horizon_MATH.h"
#include "hal.h"

static float DBUS_OneFilter(float last, float now, float thresholdValue);
static uint8_t DBUS_UpdateKeyStatus(uint8_t is_pressed, uint8_t *press_time);
static void DBUS_HandleKeyToggle(uint8_t is_pressed, uint8_t *lock_flag, uint8_t *toggle_number);

/**
 * @brief DBUS 协议解析入口
 */
void DBUS_Resolved(uint8_t* Data, void *device_ptr,uint16_t size)
{
    DBUS_Typedef*DBUS = device_ptr;
    // 如果离线时间未作改动，在此更新在线生存时间
    DBUS->offline.last_feed_tick = HAL_GetTick();

    // 将输入缓冲区映射到位域结构体
    DBUS_FrameTypeDef* frame = (DBUS_FrameTypeDef*)Data;

    DBUS->Remote.S1 = frame->switch_right;
    DBUS->Remote.S2 = frame->switch_left;

    DBUS->Remote.CH0  = (int16_t)frame->channel0 - 1024;
    DBUS->Remote.CH1  = (int16_t)frame->channel1 - 1024;
    DBUS->Remote.CH2  = (int16_t)frame->channel2 - 1024;
    DBUS->Remote.CH3  = (int16_t)frame->channel3 - 1024;
    DBUS->Remote.Dial = (int16_t)frame->dial - 1024;

    if (frame->channel0 == 0)
    {
        DBUS->Remote.CH0  = 0;
        DBUS->Remote.CH1  = 0;
        DBUS->Remote.CH2  = 0;
        DBUS->Remote.CH3  = 0;
        DBUS->Remote.Dial = 0;
    }

    DBUS->Mouse.L_State = DBUS_UpdateKeyStatus(frame->mouse_press_l, &DBUS->Mouse.L_PressTime);
    DBUS->Mouse.R_State = DBUS_UpdateKeyStatus(frame->mouse_press_r, &DBUS->Mouse.R_PressTime);

    DBUS->KeyBoard.W = DBUS_UpdateKeyStatus(frame->key_w, &DBUS->KeyBoard.W_PressTime);
    DBUS->KeyBoard.A = DBUS_UpdateKeyStatus(frame->key_a, &DBUS->KeyBoard.A_PressTime);
    DBUS->KeyBoard.S = DBUS_UpdateKeyStatus(frame->key_s, &DBUS->KeyBoard.S_PressTime);
    DBUS->KeyBoard.D = DBUS_UpdateKeyStatus(frame->key_d, &DBUS->KeyBoard.D_PressTime);

    // 鼠标一阶低通滤波
    DBUS->Mouse.X_Flt = DBUS_OneFilter(DBUS->Mouse.X_Flt, (float)frame->mouse_x, 500);
    DBUS->Mouse.Y_Flt = DBUS_OneFilter(DBUS->Mouse.Y_Flt, (float)frame->mouse_y, 500);
    DBUS->Mouse.Z_Flt = DBUS_OneFilter(DBUS->Mouse.Z_Flt, (float)frame->mouse_z, 500);

    DBUS->KeyBoard.Shift = frame->key_shift;
    DBUS->KeyBoard.Ctrl  = frame->key_ctrl;
    DBUS->KeyBoard.Q     = frame->key_q;
    DBUS->KeyBoard.E     = frame->key_e;
    DBUS->KeyBoard.R     = frame->key_r;
    DBUS->KeyBoard.F     = frame->key_f;
    DBUS->KeyBoard.G     = frame->key_g;
    DBUS->KeyBoard.Z     = frame->key_z;
    DBUS->KeyBoard.X     = frame->key_x;
    DBUS->KeyBoard.C     = frame->key_c;
    DBUS->KeyBoard.V     = frame->key_v;
    DBUS->KeyBoard.B     = frame->key_b;

    static uint8_t KeyLocks[12] = {0};
    DBUS_HandleKeyToggle(DBUS->KeyBoard.Shift, &KeyLocks[0],  &DBUS->KeyBoard.Shift_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.Ctrl,  &KeyLocks[1],  &DBUS->KeyBoard.Ctrl_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.Q,     &KeyLocks[2],  &DBUS->KeyBoard.Q_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.E,     &KeyLocks[3],  &DBUS->KeyBoard.E_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.R,     &KeyLocks[4],  &DBUS->KeyBoard.R_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.F,     &KeyLocks[5],  &DBUS->KeyBoard.F_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.G,     &KeyLocks[6],  &DBUS->KeyBoard.G_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.Z,     &KeyLocks[7],  &DBUS->KeyBoard.Z_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.X,     &KeyLocks[8],  &DBUS->KeyBoard.X_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.C,     &KeyLocks[9],  &DBUS->KeyBoard.C_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.V,     &KeyLocks[10], &DBUS->KeyBoard.V_PreeNumber);
    DBUS_HandleKeyToggle(DBUS->KeyBoard.B,     &KeyLocks[11], &DBUS->KeyBoard.B_PreeNumber);

    if (memcmp(&frame->mouse_x, (const uint8_t[10]){0}, 10) != 0) {
        DBUS->Ctrl_Mode = 1;  // 键鼠有动作 -> 键鼠模式
    } else {
        DBUS->Ctrl_Mode = 0;  // 键鼠全为0 -> 遥控模式
    }
}

/**
 * @brief 按键复合状态判定
 * @param is_pressed 当前周期的物理按下状态 (1:按下, 0:松开)
 * @param press_time 计数器指针
 * @return uint8_t 返回当前状态码：DBUS_KEY_UP(未按下), DBUS_KEY_DOWN(刚按下), DBUS_KEY_CLICK(点按触发), DBUS_KEY_PRESS(持续长按)
 * @note
 */
static uint8_t DBUS_UpdateKeyStatus(uint8_t is_pressed, uint8_t *press_time)
{
    if (is_pressed)
    {
        if (*press_time < 250) (*press_time)++;
        return (*press_time <= 18) ? DBUS_KEY_CLICK : DBUS_KEY_PRESS;
    }
    else
    {
        *press_time = 0;
        return DBUS_KEY_UP;
    }
}

/**
 * @brief 处理功能型按键的一键锁定切换机制（按下瞬时取反一次目标状态位）
 */
static void DBUS_HandleKeyToggle(uint8_t is_pressed, uint8_t *lock_flag, uint8_t *toggle_number)
{
    if (is_pressed && !(*lock_flag))
    {
        *toggle_number = !(*toggle_number);
        *lock_flag = 1;
    }
    else if (!is_pressed)
    {
        *lock_flag = 0;
    }
}

/**
 * @brief 一阶低通滤波
 */
static float DBUS_OneFilter(float last, float now, float thresholdValue)
{
    const float sensitivlFilter = 0.8f;
    const float numbFilter = 0.2f;

    if (MATH_ABS_float(MATH_ABS_float(last) - MATH_ABS_float(now)) >= thresholdValue)
    {
        return (now * sensitivlFilter + last * (1.0f - sensitivlFilter));
    }
    else
    {
        return (now * numbFilter + last * (1.0f - numbFilter));
    }
}