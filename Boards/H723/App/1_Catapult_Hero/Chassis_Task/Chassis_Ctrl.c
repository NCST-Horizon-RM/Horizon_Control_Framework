//
// Created by CaoKangqi on 2026/6/20.
//
#include "Chassis_Ctrl.h"
#include "All_define.h"
#include "Comm_DualBoard.h"
#include "Robot_Config.h"
#include "Power_CAP.h"
#include "Power_Ctrl.h"
#include "Referee.h"
#include "System_State.h"
#include "Robot_Cmd.h"

static Chassis_Ctrl_Block_t chassis_ctrl;

//功率控制
static Power_Ctrl_t chassis_model;
static Motor_Power_State_t m_states[4];//底盘共8个电机
static Power_Node_t drive_nodes[4]; // 用于驱动电机
static Power_Group_t pwr_groups[1];//两个电机组

static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge,
                                      float *out_cap_limit);
/**
 * @brief 底盘控制初始化
 * @param MOTOR 底盘电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Chassis_Init(Chassis_Cfg_t *cfg, Chassis_Type_e type)
{
    if (cfg == NULL) return 1;

    cfg->type = type;
    cfg->mass = 17.5f;
    cfg->inertia = 1.0f;
    cfg->torque_to_raw = ((1.0f / (15.7647f * 0.0157f * 0.85f)) * (16384.0f / 20.0f));
    cfg->steer_offset[0] = -120.0f * DEG2RAD;
    cfg->steer_offset[1] = -120.0f * DEG2RAD;
    cfg->steer_offset[2] =  60.0f * DEG2RAD;
    cfg->steer_offset[3] =  60.0f * DEG2RAD;

    switch (type) {
        case MECANUM:
            cfg->wheel_r = 0.075f;
            cfg->Lx = 0.20f;
            cfg->Ly = 0.20f;
            cfg->gear_ratio = 3591.0f / 187.0f;
            break;
        case OMNI:
            cfg->wheel_r = 0.075f;
            cfg->Lx = 0.25f;
            cfg->Ly = 0.25f;
            cfg->gear_ratio = 3591.0f / 187.0f;
            break;
        case SWERVE:
            cfg->wheel_r = 0.06f;
            cfg->Lx = 0.2f;
            cfg->Ly = 0.22f;
            cfg->gear_ratio = 15.76f;
            break;
        default:
            return 1;
    }
    return 0;
}

uint8_t Chassis_Control_Init(void)
{
    Chassis_Init(&chassis_ctrl.Chassis_Config,SWERVE);

    float PID_V_Param[3] = {8.0f, 0.0f, 0.0f};
    PID_Init(&chassis_ctrl.PID_Vx, 8.0f, 5.0f, PID_V_Param,0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    PID_Init(&chassis_ctrl.PID_Vy, 8.0f, 5.0f, PID_V_Param,0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

    float PID_Vw_Param[3] = {8.0f, 0.0f, 0.0f};
    PID_Init(&chassis_ctrl.PID_Vw, 8.0f, 8.0f, PID_Vw_Param,0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

    float PID_6020_Pos[3] = {500.0f, 0.0f, 0.0f};
    float PID_6020_Spd[3] = {85.0f,  0.0f, 0.0f};

    for (int i = 0; i < 4; i++)
    {
        // 6020 舵向位置环：输入弧度误差 -> 输出目标 RPM
        PID_Init(&chassis_ctrl.Steer_P[i], 250.0f,  30.0f,  PID_6020_Pos,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
        // 6020 舵向速度环：输入 RPM 误差 -> 输出电流
        PID_Init(&chassis_ctrl.Steer_S[i], 16384.0f, 4000.0f, PID_6020_Spd,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
        }
    Power_Ctrl_Init(&chassis_model);
    for(int i=0; i<4; i++) {
        // 配置驱动轮节点 (绑定 3508 模型)
        drive_nodes[i].state = &m_states[i];
        drive_nodes[i].model = &MODEL_M3508;
    }
    // 配置优先级：
    // groups[0]: 低优先级，驱动轮，超功率时优先降驱动轮功率
    pwr_groups[0].nodes = drive_nodes;
    pwr_groups[0].node_count = 4;

    //向系统下发底盘当前状态，准备中
    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    return 1;
}

/**
 * @brief 底盘控制任务
 */
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, float dt)
{
    if (c_motor == NULL) {
        System_State_Report(ID_CHASSIS, STATUS_ERROR);
        return;
    }
    if (!Is_Group_Online(CHASSIS)) {
        System_State_Report(ID_CHASSIS, STATUS_LOST);
    }
    else{System_State_Report(ID_CHASSIS, STATUS_RUN);}
    // 判断系统状态
    bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             sys_state.global_mode == GLOBAL_STANDBY ||
                             sys_state.global_mode == GLOBAL_INIT_STAGE);
    if (chassis_cmd.mode == CHASSIS_CMD_SAFE || is_system_locked)
    {
        PID_Clear(&chassis_ctrl.PID_Vx);
        PID_Clear(&chassis_ctrl.PID_Vy);
        PID_Clear(&chassis_ctrl.PID_Vw);

        for (int i = 0; i < 4; i++) {
            PID_Clear(&chassis_ctrl.Steer_P[i]);
            PID_Clear(&chassis_ctrl.Steer_S[i]);
        }
        DJI_Motor_Send(&hfdcan1, 0x200,0,0,0,0);
        DJI_Motor_Send(&hfdcan2, 0x1FE,0,0,0,0);
    }
    else
    {
        for (int i = 0; i < 4; i++) {
            chassis_ctrl.Chassis_Feedback.steer_angle[i] = (float)c_motor->DJI_6020_Steer[i].Angle_Infinite * ENCODER_TO_RAD;
            chassis_ctrl.Chassis_Feedback.steer_rpm[i]       = (float)c_motor->DJI_6020_Steer[i].Speed_now;
            chassis_ctrl.Chassis_Feedback.wheel_rpm[i]       = (float)c_motor->DJI_3508_Chassis[i].Speed_now;
        }

        Chassis_Forward(&chassis_ctrl.Chassis_Config,&chassis_ctrl.Chassis_Feedback);

        float vx_tar = chassis_cmd.target_vx;
        float vy_tar = chassis_cmd.target_vy;
        float vw_tar = chassis_cmd.target_vw;

        PID_Calculate(&chassis_ctrl.PID_Vx, chassis_ctrl.Chassis_Feedback.vx, vx_tar);
        PID_Calculate(&chassis_ctrl.PID_Vy, chassis_ctrl.Chassis_Feedback.vy, vy_tar);
        PID_Calculate(&chassis_ctrl.PID_Vw, chassis_ctrl.Chassis_Feedback.vw, vw_tar);

        /* 速度定舵向 + 加速度力矩前馈（力速混控） */
        Chassis_Mixed_Control(&chassis_ctrl.Chassis_Config,
                              vx_tar, vy_tar, vw_tar,
                              chassis_ctrl.PID_Vx.Output,
                              chassis_ctrl.PID_Vy.Output,
                              chassis_ctrl.PID_Vw.Output,
                              &chassis_ctrl.Chassis_Feedback,
                              &chassis_ctrl.Chassis_Command);

        for (int i = 0; i < 4; i++)
        {
            PID_Calculate(&chassis_ctrl.Steer_P[i],
                          chassis_ctrl.Chassis_Feedback.steer_angle[i],
                          chassis_ctrl.Chassis_Command.steer_angle_target[i]);

            PID_Calculate(&chassis_ctrl.Steer_S[i],
                          chassis_ctrl.Chassis_Feedback.steer_rpm[i],
                          chassis_ctrl.Steer_P[i].Output);
        }

    //     for(int i=0; i<4; i++) {
    //         m_states[i].speed_rpm = chassis_ctrl.swerve_fb.wheel_rpm[i];
    //         m_states[i].original_cmd = chassis_ctrl.Drive_S[i].Output;
    //
    //         m_states[i+4].speed_rpm = chassis_ctrl.swerve_fb.steer_rpm[i];
    //         m_states[i+4].original_cmd = chassis_ctrl.Steer_S[i].Output;
    //     }
    //
    //     bool trigger_discharge = true;
    //     float cap_board_limit = 0.0f;
    //     float final_limit = 0.0f;
    //     if (Referee.offline.is_online) {
    //         final_limit = Chassis_Power_Arbitrator(
    //                                 Referee.robot_status.chassis_power_limit,
    //                                 Referee.power_heat_data.buffer_energy,
    //                                 1, &cap, &trigger_discharge, &cap_board_limit);
    //     }
    //     else {
    //         trigger_discharge = FALSE;
    //         cap_board_limit = 75.0f;//
    //         final_limit = 75.0f;
    //     }
    //     Power_Ctrl_Calculate(&chassis_model, final_limit, pwr_groups, 2);
    //
    //     for(int i=0; i<4; i++) {
    //         chassis_ctrl.Drive_S[i].Output = m_states[i].limited_cmd;
    //         chassis_ctrl.Steer_S[i].Output = m_states[i+4].limited_cmd;
    //     }
    //
    //     CapSetData_t cap_cmd = {0};
    //     cap_cmd.Control.power_key     = 1;
    //     cap_cmd.Control.capPowerLimit = (uint8_t)cap_board_limit;
    //     cap_cmd.Control.buffer_now    = (uint8_t)Referee.power_heat_data.buffer_energy;
    //     cap_cmd.Control.robot_state   = (Referee.robot_status.current_HP > 0) ? 1 : 0;
    //     Power_Cap_Tx(&hfdcan2, 0x252, &cap_cmd);
    }
    //电流发送
    if (!is_system_locked)
    {
        DJI_Motor_Send(&hfdcan1, 0x200,
                       (int16_t)chassis_ctrl.Chassis_Command.wheel_torque_raw[0],
                       (int16_t)chassis_ctrl.Chassis_Command.wheel_torque_raw[1],
                       (int16_t)chassis_ctrl.Chassis_Command.wheel_torque_raw[2],
                       (int16_t)chassis_ctrl.Chassis_Command.wheel_torque_raw[3]);

        DJI_Motor_Send(&hfdcan2, 0x1FE,
                       (int16_t)chassis_ctrl.Steer_S[0].Output,
                       (int16_t)chassis_ctrl.Steer_S[1].Output,
                       (int16_t)chassis_ctrl.Steer_S[2].Output,
                       (int16_t)chassis_ctrl.Steer_S[3].Output);
    }
}

// 超级电容与缓冲能量调参宏定义
#define BUFFER_COMP_KP      2.0f    // 缓冲能量补偿的比例系数 (Kp)
#define TARGET_BUFFER       40.0f   // 目标期望缓冲能量 (J)
#define MIN_CAP_VOLTAGE     23.0f   // 超级电容最低放电阈值 (百分比)
#define RAMP_CAP_VOLTAGE    27.0f   // 斜坡衰减开始阈值 (百分比)
#define MAX_BOOST_POWER     150.0f  // 超级电容输出的最大冲刺功率 (W)

/**
 * @brief 功率策略仲裁器
 * * @param base_power_limit  裁判系统当前的基础功率上限
 * @param cur_buffer        裁判系统当前剩余的缓冲能量 (0~60J)
 * @param boost_intent      输入指令是否开启超电
 * @param cap_data          超级电容状态反馈 (包含在线状态、电量、故障码等)
 * @param out_discharge     [输出参数] 发送给超电是否开启
 * @param out_cap_limit     [输出参数] 发送给超电的功率限制
 * * @return float            返回最终决定的目标功率上限 (W)
 */
static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge,
                                      float *out_cap_limit)
{
    // 公式: power_comp = -Kp * (目标缓冲 - 当前缓冲)
    float power_comp = -BUFFER_COMP_KP * (TARGET_BUFFER - cur_buffer);
    float base_allowable_power = base_power_limit + power_comp;
    // 发给电容的功率限制
    *out_cap_limit = base_allowable_power;
    // 电机的目标功率上限初始化为基础功率
    float final_target_power = base_allowable_power;
    // 超级电容离线/硬件故障保护
    if (cap_data->get.offline.is_online == 0 || cap_data->get.cap_state != 0)
    {
        *out_discharge = false;
        return final_target_power - 5.0f;
    }
    // 在线且正常状态下的 放电/充电 逻辑
    if (boost_intent && cap_data->get.Cap_Capacity > MIN_CAP_VOLTAGE)
    {
        float boost_allowance = MAX_BOOST_POWER;
        // 斜坡衰减保护机制
        if (cap_data->get.Cap_Capacity < RAMP_CAP_VOLTAGE) {
            float ratio = (float)(cap_data->get.Cap_Capacity - MIN_CAP_VOLTAGE) /
                          (float)(RAMP_CAP_VOLTAGE - MIN_CAP_VOLTAGE);
            boost_allowance *= ratio;
        }
        // 最终允许的底盘功率上限 = 基础可用功率 + 超电补偿功率
        final_target_power += boost_allowance;
        *out_discharge = true;
    }
    else
    {
        // 留 5W 功率给超级电容充电
        final_target_power -= 5.0f;
        *out_discharge = false;
    }
    return final_target_power; // 返回给电机的最终功率限制
}
