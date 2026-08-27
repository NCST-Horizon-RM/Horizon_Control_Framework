//
// Created by CaoKangqi on 2026/6/25.
//
#include "Catapult_Ctrl.h"
#include "Message_Center.h"
#include "System_State.h"
#include "Horizon_MATH.h"
#include "BSP_TIM.h"
#include <math.h>
#include "Robot_Cmd.h"

#define SHOOT_TOTAL_SLOTS         6.0f
#define SHOOT_FEED_ZERO_OFFSET    1325.0f
#define SHOOT_COUNTS_PER_SHOT     (8192.0f / SHOOT_TOTAL_SLOTS)
#define SHOOT_FEED_INIT_BIAS      150.0f
#define SHOOT_FEED_RAMP_SPEED     1000.0f    // 拨盘平滑斜坡速度 (Unit/s)

#define YAW_CALIB_SPEED           500.0f     // 云台寻零速度
#define YAW_CALIB_STUCK_CURR      3000.0f    // 云台寻零堵转电流阈值
#define YAW_CALIB_TIME_SEC        0.4f       // 堵转确认时间(秒)
#define YAW_CENTER_OFFSET         72550.0f   // 云台中心偏移量

#define PULL_RECOIL_ANGLE         1100000.0f // 拉绳复位行程
#define PULL_TRIGGER_DELAY_SEC    0.5f      // 触发后等待时间(秒)

#define TRIGGER_PWM_OPEN          1200        // 扳机关闭 PWM
#define TRIGGER_PWM_CLOSE         600       // 扳机开启 PWM
// 静态实例化内部控制块
static Shoot_Ctrl_Block_t shoot_ctrl = {0};

// Pub/Sub 句柄与本地缓存
static Subscriber_t *sys_state_sub;
static Subscriber_t *shoot_cmd_sub;
static Subscriber_t *gimbal_cmd_sub;

static System_State_t local_sys_state;
static Shoot_Cmd_t    local_shoot_cmd = {0};
static Gimbal_Cmd_t   local_gimbal_cmd = {0};
// 扳机舵机
BSP_PWM_t trigger_pwm = {&htim2, TIM_CHANNEL_1, PWM_CHANNEL_NORMAL};

// 私有函数声明
static bool Check_Motor_Stuck(const DJI_MOTOR_DATA_Typedef* motor, float target_speed, float stuck_current, float dt, float confirm_time_sec);
static float Math_Ramp(float current, float target, float step);

/**
 * @brief 发射与云台(Yaw)控制初始化
 */
uint8_t Shoot_Control_Init(void)
{
    BSP_PWM_Start(&trigger_pwm);
    BSP_PWM_Set_Compare(&trigger_pwm, TRIGGER_PWM_OPEN);

    shoot_ctrl.calib_state   = CALIB_START;
    shoot_ctrl.pull_state    = PULL_STATE_NORMAL;
    shoot_ctrl.last_switch_v = GPIO_PIN_SET;

    uint8_t mode = Integral_Limit | ErrorHandle;

    float PID_P_FEED[3] = {1.0f, 0.0f, 0.0f};
    float PID_S_FEED[3] = {0.4f, 0.0f, 0.0f};
    PID_Init(&shoot_ctrl.PID_Feed_P, 80, 30, PID_P_FEED, 0, 0, 0, 0, 0, mode);
    PID_Init(&shoot_ctrl.PID_Feed_S, 15, 10, PID_S_FEED, 0, 0, 0, 0, 0, mode);

    float PID_P_YAW[3]  = {1.0f, 0.0f, 0.0f};
    float PID_S_YAW[3]  = {5.0f, 0.01f, 0.0f};
    PID_Init(&shoot_ctrl.PID_Yaw_P, 1000, 150, PID_P_YAW, 0, 0, 0, 0, 0, mode);
    PID_Init(&shoot_ctrl.PID_Yaw_S, 16384, 2000, PID_S_YAW, 0, 0, 0, 0, 0, mode);

    float PID_P_PULL[3] = {2.5f, 0.01f, 0.0f};
    float PID_S_PULL[3] = {11.0f, 0.01f, 0.0f};
    PID_Init(&shoot_ctrl.PID_Pull_P, 2000, 100, PID_P_PULL, 0, 0, 0, 0, 0, mode);
    PID_Init(&shoot_ctrl.PID_Pull_S, 16000, 2000, PID_S_PULL, 0, 0, 0, 0, 0, mode);

    sys_state_sub  = SubRegister("system_state", sizeof(System_State_t));
    shoot_cmd_sub  = SubRegister("shoot_cmd", sizeof(Shoot_Cmd_t));
    gimbal_cmd_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Cmd_t));

    System_State_Report(ID_SHOOT, STATUS_PREPARING);

    return 1;
}

/**
 * @brief 发射与云台控制任务
 */
void Shoot_Control_Task(const Shoot_Motor_Group_t *s_motor,
                        const Gimbal_Motor_Group_t *g_motor,
                        float dt) {
    if (s_motor == NULL || g_motor == NULL || dt <= 0.0f) {
        System_State_Report(ID_SHOOT, STATUS_ERROR);
        return;
    }

    if (sys_state_sub)  SubGetMessage(sys_state_sub, &local_sys_state);
    if (shoot_cmd_sub)  SubGetMessage(shoot_cmd_sub, &local_shoot_cmd);
    if (gimbal_cmd_sub) SubGetMessage(gimbal_cmd_sub, &local_gimbal_cmd);

    System_State_Report(ID_SHOOT, STATUS_RUN);
    System_State_Report(ID_GIMBAL, STATUS_RUN);

    if (!Is_Group_Online(SHOOT))  System_State_Report(ID_SHOOT, STATUS_LOST);
    if (!Is_Group_Online(GIMBAL)) System_State_Report(ID_GIMBAL, STATUS_LOST);

    bool is_system_locked = (local_sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             local_sys_state.global_mode == GLOBAL_STANDBY ||
                             local_sys_state.global_mode == GLOBAL_MODULE_ERROR);

    if (is_system_locked)
    {
        PID_Clear(&shoot_ctrl.PID_Feed_P);
        PID_Clear(&shoot_ctrl.PID_Feed_S);
        PID_Clear(&shoot_ctrl.PID_Yaw_P);
        PID_Clear(&shoot_ctrl.PID_Yaw_S);
        PID_Clear(&shoot_ctrl.PID_Pull_P);
        PID_Clear(&shoot_ctrl.PID_Pull_S);

        shoot_ctrl.out_feed_torque = 0.0f;
        shoot_ctrl.out_yaw_curr = 0;
        shoot_ctrl.out_pull_curr = 0;

        goto EXECUTE_OUTPUT;
    }

    // --- 1. Yaw 校准状态机 ---
    switch (shoot_ctrl.calib_state)
    {
        case CALIB_START:
            shoot_ctrl.calib_state = CALIB_MOVING;
            break;

        case CALIB_MOVING:
            PID_Calculate(&shoot_ctrl.PID_Yaw_S, g_motor->DJI_3508_Yaw.Speed_now, YAW_CALIB_SPEED);
            shoot_ctrl.out_yaw_curr = shoot_ctrl.PID_Yaw_S.Output;
            shoot_ctrl.out_feed_torque = 0.0f;
            shoot_ctrl.out_pull_curr = 0;

            if (Check_Motor_Stuck(&g_motor->DJI_3508_Yaw, YAW_CALIB_SPEED, YAW_CALIB_STUCK_CURR, dt, YAW_CALIB_TIME_SEC)) {
                shoot_ctrl.calib_state = CALIB_DONE;
            }
            goto EXECUTE_OUTPUT; // 校准中，不执行发射逻辑

        case CALIB_DONE:
            shoot_ctrl.PID_Yaw_S.Iout = 0.0f;
            shoot_ctrl.zero_offset_angle = g_motor->DJI_3508_Yaw.Angle_Infinite;
            shoot_ctrl.mid_offset_angle = shoot_ctrl.zero_offset_angle - YAW_CENTER_OFFSET;
            shoot_ctrl.PID_Yaw_P.Ref = shoot_ctrl.mid_offset_angle;

            shoot_ctrl.calib_state = CALIB_NORMAL;
            break;

        case CALIB_NORMAL:
            break;
    }
    if (shoot_ctrl.calib_state == CALIB_NORMAL)
    {
        // --- 2. 正常模式解算 ---
        if (!s_motor->DM4310_Feed.offline.is_online) goto EXECUTE_OUTPUT;

        // 拨盘逻辑
        float feed_pulse = s_motor->DM4310_Feed.Angle_Infinite;
        if (!shoot_ctrl.feed_motor.is_init && feed_pulse != 0.0f) {
            shoot_ctrl.feed_motor.target_pos_cnt = (int32_t)floorf((feed_pulse - SHOOT_FEED_ZERO_OFFSET + SHOOT_FEED_INIT_BIAS) / SHOOT_COUNTS_PER_SHOT);
            shoot_ctrl.feed_motor.smooth_ref = feed_pulse;
            shoot_ctrl.feed_motor.is_init = true;
        }

        float feed_final_target = SHOOT_FEED_ZERO_OFFSET + ((float)shoot_ctrl.feed_motor.target_pos_cnt * SHOOT_COUNTS_PER_SHOT);

        // 基于 dt 的平滑斜坡
        float feed_step = SHOOT_FEED_RAMP_SPEED * dt;
        shoot_ctrl.feed_motor.smooth_ref = Math_Ramp(shoot_ctrl.feed_motor.smooth_ref, feed_final_target, feed_step);

        PID_Calculate(&shoot_ctrl.PID_Feed_P, feed_pulse, shoot_ctrl.feed_motor.smooth_ref);
        PID_Calculate(&shoot_ctrl.PID_Feed_S, s_motor->DM4310_Feed.Speed_now, shoot_ctrl.PID_Feed_P.Output);
        shoot_ctrl.out_feed_torque = -shoot_ctrl.PID_Feed_S.Output;

        // Yaw轴逻辑 (仅使用 target_yaw)
        shoot_ctrl.PID_Yaw_P.Ref += local_gimbal_cmd.target_yaw;
        shoot_ctrl.PID_Yaw_P.Ref = MATH_Limit_float(shoot_ctrl.PID_Yaw_P.Ref,
                                                    shoot_ctrl.mid_offset_angle - YAW_CENTER_OFFSET,
                                                    shoot_ctrl.mid_offset_angle + YAW_CENTER_OFFSET);

        PID_Calculate(&shoot_ctrl.PID_Yaw_P, g_motor->DJI_3508_Yaw.Angle_Infinite, shoot_ctrl.PID_Yaw_P.Ref);
        PID_Calculate(&shoot_ctrl.PID_Yaw_S, g_motor->DJI_3508_Yaw.Speed_now, shoot_ctrl.PID_Yaw_P.Output);
        shoot_ctrl.out_yaw_curr = shoot_ctrl.PID_Yaw_S.Output;

        // --- 3. 发射状态机 (重新梳理：抛石机物理流转) ---
        GPIO_PinState current_switch_v = HAL_GPIO_ReadPin(Switch_GPIO_Port, Switch_Pin);

        switch (shoot_ctrl.pull_state)
        {
            case PULL_STATE_NORMAL:
                shoot_ctrl.out_trigger_pwm = TRIGGER_PWM_OPEN;
                shoot_ctrl.PID_Pull_P.Ref = s_motor->DJI_3508_Pull.Angle_Infinite + 300.0f;

                if (current_switch_v == GPIO_PIN_RESET && shoot_ctrl.last_switch_v == GPIO_PIN_SET)
                {
                    shoot_ctrl.PID_Pull_P.Ref = s_motor->DJI_3508_Pull.Angle_Infinite; // 停在当前位置
                    shoot_ctrl.out_trigger_pwm = TRIGGER_PWM_CLOSE;
                    shoot_ctrl.pull_state = PULL_STATE_TRIGGERED;
                }
                break;

            case PULL_STATE_TRIGGERED:
                shoot_ctrl.pull_timer_sec += dt;
                if (shoot_ctrl.pull_timer_sec >= PULL_TRIGGER_DELAY_SEC) {
                    shoot_ctrl.PID_Pull_P.Ref -= PULL_RECOIL_ANGLE; // 设定回拉行程目标
                    shoot_ctrl.feed_motor.target_pos_cnt -= 1;      // 联动推弹
                    shoot_ctrl.pull_state = PULL_STATE_RESET;       // 开始回拉
                }
                break;

            case PULL_STATE_RESET:
                if (MATH_ABS_float(s_motor->DJI_3508_Pull.Angle_Infinite - shoot_ctrl.PID_Pull_P.Ref) < 1000.0f
                    && shoot_ctrl.feed_motor.smooth_ref == feed_final_target) {
                    shoot_ctrl.pull_state = PULL_STATE_STOPPED; // 到位后，回到 NORMAL 状态重新寻找限位锁闭
                    }
                break;
            case PULL_STATE_STOPPED:
                if (local_shoot_cmd.mode != SHOOT_CMD_SAFE &&
                    local_shoot_cmd.trigger_single ) // 检测单发上升沿
                {
                    shoot_ctrl.out_trigger_pwm = TRIGGER_PWM_OPEN;
                    shoot_ctrl.pull_timer_sec = 0.0f;               // 清零延时计时
                    shoot_ctrl.pull_state = PULL_STATE_NORMAL;   // 进入延时开火阶段
                }
                break;
        }

        shoot_ctrl.last_switch_v = current_switch_v;
        shoot_ctrl.last_cmd_trigger = local_shoot_cmd.trigger_single;

        // 计算扳机电流
        PID_Calculate(&shoot_ctrl.PID_Pull_P, s_motor->DJI_3508_Pull.Angle_Infinite, shoot_ctrl.PID_Pull_P.Ref);
        PID_Calculate(&shoot_ctrl.PID_Pull_S, s_motor->DJI_3508_Pull.Speed_now, shoot_ctrl.PID_Pull_P.Output);
        shoot_ctrl.out_pull_curr = shoot_ctrl.PID_Pull_S.Output;
}

EXECUTE_OUTPUT:
    // 统一CAN发送
    BSP_PWM_Set_Compare(&trigger_pwm, shoot_ctrl.out_trigger_pwm);
    DM_Motor_Send(&hfdcan3, 0x3FE, shoot_ctrl.out_feed_torque, 0, 0, 0);
    DJI_Motor_Send(&hfdcan3, 0x200, 0, shoot_ctrl.out_pull_curr, shoot_ctrl.out_yaw_curr, 0);
}

/**
 * @brief 基于 dt 的斜坡控制函数
 */
static float Math_Ramp(float current, float target, float step) {
    if (current < target) {
        current += step;
        if (current > target) current = target;
    } else if (current > target) {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

/**
 * @brief 基于 dt 的电机堵转检测
 */
static bool Check_Motor_Stuck(const DJI_MOTOR_DATA_Typedef* motor, float target_speed, float stuck_current, float dt, float confirm_time_sec)
{
    static float time_acc = 0.0f;
    static float last_angle = 0.0f;

    float pos_delta = MATH_ABS_float(motor->Angle_Infinite - last_angle);
    float current   = MATH_ABS_float(motor->current);
    float speed     = MATH_ABS_float(motor->Speed_now);

    if (pos_delta < 10.0f && current > stuck_current && speed < MATH_ABS_float(target_speed) * 0.2f) {
        time_acc += dt;
        if (time_acc >= confirm_time_sec) {
            time_acc = 0.0f;
            return true;
        }
    } else {
        time_acc = 0.0f;
    }

    last_angle = motor->Angle_Infinite;
    return false;
}