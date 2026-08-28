//
// Created by R0602 on 26-7-10.
//
#include "Gimbal_Ctrl.h"

#include "All_define.h"
#include "Comm_DualBoard.h"
#include "System_State.h"
#include "IMU_Task.h"
#include "Robot_Config.h"
#include "Robot_Cmd.h"
#include "Vofa.h"

static Gimbal_Ctrl_Block_t gimbal_ctrl;


/**
 * @brief 云台控制初始化
 * @param MOTOR 云台电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Gimbal_Control_Init(void)
{

    // Pitch PID参数初始化
    float PID_Pitch_P[3] = {/*0.45*/0.3f,   0.0f,  0.0f};
        PID_Init(&gimbal_ctrl.Pitch_P, 50.0f, 30.0f, PID_Pitch_P,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_Pitch_S[3] = {/*5.5*/-6.0f,   /*0.02*/0.0f,   0.0f};
    PID_Init(&gimbal_ctrl.Pitch_S, 30.0f, 5.0f, PID_Pitch_S,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //Yaw PID参数初始化
    float PID_Yaw_P[3] = {/*-0.28*/0.15f,   0.0f,  0.0f};
    PID_Init(&gimbal_ctrl.Yaw_P, 20.0f, 5.0f, PID_Yaw_P,
        0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_Yaw_S[3] = {/*-8.0*/-7.0f,   /*0.03*/0.0f,   0.0f};
    PID_Init(&gimbal_ctrl.Yaw_S, 30.0f, 4.0f, PID_Yaw_S,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //向系统下发底盘当前状态，准备中
    System_State_Report(ID_GIMBAL, STATUS_PREPARING);
    return 1;
}
/**
\ * @brief 云台控制任务
 */
void Gimbal_Control_Task(const Gimbal_Motor_Group_t *g_motor,const IMU_Data_t *g_imu)
{
    // 空指针保护
    if (g_motor == NULL || g_imu == NULL) {
        System_State_Report(ID_GIMBAL, STATUS_ERROR);
        return;
    }
    // 状态汇报
    if (!Is_Group_Online(GIMBAL)) {
        System_State_Report(ID_GIMBAL, STATUS_LOST);
    }
    else{System_State_Report(ID_GIMBAL, STATUS_RUN);}
    // 判断系统状态
    bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             sys_state.global_mode == GLOBAL_STANDBY ||
                             sys_state.global_mode == GLOBAL_INIT_STAGE);

    if (gimbal_cmd.mode == GIMBAL_CMD_SAFE || is_system_locked)
    {
        // 清空PID
        for (int i = 0; i < 4; i++) {
            PID_Clear(&gimbal_ctrl.Pitch_P);
            PID_Clear(&gimbal_ctrl.Pitch_S);
            PID_Clear(&gimbal_ctrl.Yaw_P );
            PID_Clear(&gimbal_ctrl.Yaw_S );

        }
    }
    else
    {
        gimbal_ctrl.Yaw_P.Ref = g_imu->yaw + normalize_to_pi((gimbal_cmd.target_yaw - g_imu->yaw) * DEG2RAD) * RAD2DEG;
        PID_Calculate(&gimbal_ctrl.Yaw_P, g_imu->yaw, gimbal_ctrl.Yaw_P.Ref);
        PID_Calculate(&gimbal_ctrl.Yaw_S,g_imu->gyro[2],gimbal_ctrl.Yaw_P.Output - 3*gimbal_cmd.target_yaw_rate);

        PID_Calculate(&gimbal_ctrl.Pitch_P,g_imu->pitch,gimbal_cmd.target_pitch);
        PID_Calculate(&gimbal_ctrl.Pitch_S,g_imu->gyro[1],gimbal_ctrl.Pitch_P.Output + 3.5f*gimbal_cmd.target_pitch_rate);
    }

    DM_Motor_Send(&hcan1, 0x3FE,
                            (int16_t)gimbal_ctrl.Yaw_S.Output,
                           (int16_t)gimbal_ctrl.Pitch_S.Output,
                           0,
                           0);

}