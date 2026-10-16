//
// Created by lemon on 2026/10/16.
//

#include "Yaw_Ctrl.h"



Yaw_Ctrl_Block_t yaw_ctrl_block = {0};

uint8_t Yaw_Init()
{
    float PID_YAW_P[3] = {-50.0f, -0.01f, 0.0f};
    PID_Init(&yaw_ctrl_block.Yaw_P, 700.0f, 10.0f, PID_YAW_P,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_YAW_S[3] = {20.0f, 0.0f, 0.5f};
    PID_Init(&yaw_ctrl_block.Yaw_S, 10000.0f, 0.0f, PID_YAW_S,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    //向系统下发底盘当前状态，准备中
    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    return 1;
}

void Yaw_track()
{
    // 状态汇报
    if (!Is_Group_Online(CHASSIS)) {
        System_State_Report(ID_CHASSIS, STATUS_LOST);
    }
    else{System_State_Report(ID_CHASSIS, STATUS_RUN);}
    // 判断系统状态
    bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             sys_state.global_mode == GLOBAL_STANDBY ||
                             sys_state.global_mode == GLOBAL_INIT_STAGE);

    if (vision_rx_cmd.mode == VISION_CMD_SAFE || is_system_locked)
    {
        // 清空PID
        for (int i = 0; i < 2; i++) {
            PID_Clear(&yaw_ctrl_block.Yaw_P );
            PID_Clear(&yaw_ctrl_block.Yaw_S );
        }
    }else
    {
        yaw_ctrl_block.Yaw_P.Ref = chassis_cmd.target_yaw;
        PID_Calculate(&yaw_ctrl_block.Yaw_P,-(vision_rx_cmd.Data.x0),yaw_ctrl_block.Yaw_P.Ref);
        PID_Calculate(&yaw_ctrl_block.Yaw_S,dart_rack_motors.DJI_2006_Yaw.Speed_now,yaw_ctrl_block.Yaw_P.Output);
    }
}