/**
 * @file VT13.c
 * @brief RoboMaster 裁判系统相机图传模块 VT13 协议解析源文件
 * @version 1.0
 * @date 2026-03-21
 * @author CaoKangqi
 */

#include "VT13.h"
#include "Horizon_MATH.h"
#include "hal.h"

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
static float VT_OneFilter(float last, float now, float thresholdValue);
static float VT_deadband(float Value, float minValue, float maxValue);
static uint8_t VT_UpdateKeyStatus(uint8_t is_pressed, uint8_t *press_time);
static void VT_HandleKeyToggle(uint8_t is_pressed, uint8_t *lock_flag, uint8_t *toggle_number);

/**
 * @brief VT13 协议解析函数
 */
void VT13_Resolved(uint8_t* Data, void *device_ptr, uint16_t size)
{
    VT13_Typedef *VT13 = device_ptr;
    VT13_FrameTypeDef* frame = (VT13_FrameTypeDef*)Data;
    VT13->offline.last_feed_tick = HAL_GetTick();
    // 校验帧头 0xA9, 0x53 及整个数据帧的 CRC16
    if (frame->header1 == 0XA9 && frame->header2 == 0X53 && verify_crc16_check_sum(Data, 21))
    {
        VT13->CRC_flag = true;

        VT13->Remote.Channel[0] = (int16_t)frame->channel0 - 1024;
        VT13->Remote.Channel[1] = (int16_t)frame->channel1 - 1024;
        VT13->Remote.Channel[2] = (int16_t)frame->channel2 - 1024;
        VT13->Remote.Channel[3] = (int16_t)frame->channel3 - 1024;
        VT13->Remote.wheel      = (int16_t)frame->wheel - 1024;

        VT13->Remote.mode_sw    = frame->mode_sw;
        VT13->Remote.pause      = frame->pause;
        VT13->Remote.fn_1       = frame->fn_1;
        VT13->Remote.fn_2       = frame->fn_2;
        VT13->Remote.trigger    = frame->trigger;

        VT13->Mouse.X_Flt = VT_deadband(VT_OneFilter(VT13->Mouse.X_Flt, (float)frame->mouse_x, 500), -3.0e-3f, 3.0e-3f);
        VT13->Mouse.Y_Flt = VT_deadband(VT_OneFilter(VT13->Mouse.Y_Flt, (float)frame->mouse_y, 500), -3.0e-3f, 3.0e-3f);
        VT13->Mouse.Z_Flt = VT_deadband(VT_OneFilter(VT13->Mouse.Z_Flt, (float)frame->mouse_z, 500), -3.0e-3f, 3.0e-3f);

        VT13->Mouse.L_State = VT_UpdateKeyStatus(frame->mouse_l & 0x01, (uint8_t*)&VT13->Mouse.L_PressTime);
        VT13->Mouse.R_State = VT_UpdateKeyStatus(frame->mouse_r & 0x01, (uint8_t*)&VT13->Mouse.R_PressTime);
        VT13->Mouse.M_State = VT_UpdateKeyStatus(frame->mouse_m & 0x01, (uint8_t*)&VT13->Mouse.M_PressTime);

        VT13->KeyBoard.W = VT_UpdateKeyStatus(frame->key_w, &VT13->KeyBoard.W_PressTime);
        VT13->KeyBoard.S = VT_UpdateKeyStatus(frame->key_s, &VT13->KeyBoard.S_PressTime);
        VT13->KeyBoard.A = VT_UpdateKeyStatus(frame->key_a, &VT13->KeyBoard.A_PressTime);
        VT13->KeyBoard.D = VT_UpdateKeyStatus(frame->key_d, &VT13->KeyBoard.D_PressTime);

        VT13->KeyBoard.Shift = frame->key_shift;
        VT13->KeyBoard.Ctrl  = frame->key_ctrl;
        VT13->KeyBoard.Q     = frame->key_q;
        VT13->KeyBoard.E     = frame->key_e;
        VT13->KeyBoard.R     = frame->key_r;
        VT13->KeyBoard.F     = frame->key_f;
        VT13->KeyBoard.G     = frame->key_g;
        VT13->KeyBoard.Z     = frame->key_z;
        VT13->KeyBoard.X     = frame->key_x;
        VT13->KeyBoard.C     = frame->key_c;
        VT13->KeyBoard.V     = frame->key_v;
        VT13->KeyBoard.B     = frame->key_b;

        // 键盘锁存
        static uint8_t KeyLocks[12] = {0};
        VT_HandleKeyToggle(VT13->KeyBoard.Shift, &KeyLocks[0],  &VT13->KeyBoard.Shift_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.Ctrl,  &KeyLocks[1],  &VT13->KeyBoard.Ctrl_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.Q,     &KeyLocks[2],  &VT13->KeyBoard.Q_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.E,     &KeyLocks[3],  &VT13->KeyBoard.E_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.R,     &KeyLocks[4],  &VT13->KeyBoard.R_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.F,     &KeyLocks[5],  &VT13->KeyBoard.F_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.G,     &KeyLocks[6],  &VT13->KeyBoard.G_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.Z,     &KeyLocks[7],  &VT13->KeyBoard.Z_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.X,     &KeyLocks[8],  &VT13->KeyBoard.X_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.C,     &KeyLocks[9],  &VT13->KeyBoard.C_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.V,     &KeyLocks[10], &VT13->KeyBoard.V_PreeNumber);
        VT_HandleKeyToggle(VT13->KeyBoard.B,     &KeyLocks[11], &VT13->KeyBoard.B_PreeNumber);
        if (memcmp(&frame->mouse_x, (const uint8_t[9]){0}, 9) != 0) {
            VT13->Ctrl_Mode = 1;  // 键鼠有动作 -> 键鼠模式
        } else {
            VT13->Ctrl_Mode = 0;  // 键鼠全为0 -> 遥控模式
        }
    }
    else
    {
        VT13->CRC_flag = false;
    }
}

/**
 * @brief 更新按键按压时间，并判断当前复合状态（释放/点按/长按）
 * @param is_pressed 当前物理是否按下
 * @param press_time 持续按下时间计数缓存
 * @return 复合状态编码 (KEY_UP, KEY_CLICK, KEY_PRESS)
 */
static uint8_t VT_UpdateKeyStatus(uint8_t is_pressed, uint8_t *press_time)
{
    if (is_pressed)
    {
        if (*press_time < 250) (*press_time)++;
        return (*press_time <= 20) ? VT13_KEY_CLICK : VT13_KEY_PRESS;
    }
    else
    {
        *press_time = 0;
        return VT13_KEY_UP;
    }
}

/**
 * @brief 处理按键切换锁定逻辑（按下瞬时改变一次状态）
 * @param is_pressed 物理按下检测
 * @param lock_flag 触发边沿锁标志位
 * @param toggle_number 状态转换标志寄存器（目标取反状态位）
 */
static void VT_HandleKeyToggle(uint8_t is_pressed, uint8_t *lock_flag, uint8_t *toggle_number)
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
 * @brief 鼠标数据一阶低通滤波
 */
static float VT_OneFilter(float last, float now, float thresholdValue)
{
    const float sensitivlFilter = 0.8f;
    const float numbFilter = 0.2f;
    if (MATH_ABS_float(last - now) >= thresholdValue)
        return (now * sensitivlFilter + last * (1.0f - sensitivlFilter));
    else
        return (now * numbFilter + last * (1.0f - numbFilter));
}

/**
 * @brief 浮点死区滤除
 */
static float VT_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue) return 0.0f;
    return Value;
}

/**
 * @brief 计算 CRC16 校验和
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    while(len--)
    {
        crc16 = (crc16 >> 8) ^ crc16_tab[(crc16 ^ *p_msg++) & 0x00ff];
    }
    return crc16;
}

/**
 * @brief 验证数据帧 CRC16 完整性与正确性
 */
static bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    if (p_msg == NULL || len <= 2) return false;
    uint16_t w_expected = get_crc16_check_sum(p_msg, len - 2, 0xFFFF);
    return ((w_expected & 0xFF) == p_msg[len - 2] && (w_expected >> 8) == p_msg[len - 1]);
}