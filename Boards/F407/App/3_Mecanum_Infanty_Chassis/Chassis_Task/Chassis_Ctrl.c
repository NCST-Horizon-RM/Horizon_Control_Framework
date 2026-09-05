//
// Created by CaoKangqi on 2026/6/20.
//
#include "Chassis_Ctrl.h"
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
static Motor_Power_State_t m_states[4];//底盘共4个电机
static Power_Node_t drive_nodes[4]; // 用于驱动电机
static Power_Group_t pwr_groups[1];//一个电机组

static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge,
                                      float *out_cap_limit);
/**
 * @brief  非对称线性斜坡限幅函数
 * @param  target      目标值
 * @param  current     当前值
 * @param  acc_step    加速最大步长 (绝对值)
 * @param  dec_step    减速最大步长 (绝对值)
 * @return float       经过限制的当前值
 */
static float Ramp_Calc(float target, float current, float acc_step, float dec_step, float dt)
{
    float step = 0.0f;
    bool is_accelerating = false;
    if (current >= 0.0f && target > current) {
        is_accelerating = true;
    }
    else if (current <= 0.0f && target < current) {
        is_accelerating = true;
    }
    step = is_accelerating ? acc_step * dt : dec_step * dt;
    if (target > current) {
        current += step;
        if (current > target) {
            current = target;
        }
    } else if (target < current) {
        current -= step;
        if (current < target) {
            current = target;
        }
    }
    return current;
}

uint8_t Mecanum_Init(mecanumInit_typdef *mecanumInitT)
{
    mecanumInitT->wheel_r = 0.076f;
    mecanumInitT->half_wheelbase = 0.169f;   // 前后轮中心距的一半 (Lx)
    mecanumInitT->half_track_width = 0.169f; // 左右轮中心距的一半 (Ly)
    mecanumInitT->deceleration_ratio = 3591.0f / 187.0f;
    return 0;
}

/**
 * @brief 底盘控制初始化
 * @param MOTOR 底盘电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Chassis_Control_Init(void)
{
    //底盘初始化
    Chassis_Init(&chassis_ctrl.chassis_cfg,chassis_ctrl.chassis_cfg.type);

    float PID_vx[3] = {9.0f,   0.0f,  0.0f};
    PID_Init(&chassis_ctrl.vx, 15.0f, 0.0f, PID_vx,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_vy[3] = {9.0f,   0.0f,  0.0f};
    PID_Init(&chassis_ctrl.vy, 15.0f, 0.0f, PID_vy,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    float PID_vw[3] = {9.0f,   0.0f,  0.0f};
    PID_Init(&chassis_ctrl.vw, 18.0f, 0.0f, PID_vw,
            0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    // 底盘跟随PID初始化
    float PID_Follow_Pos[3] = {18.0f,   0.0f,   0.0f};
    PID_Init(&chassis_ctrl.Follow_Pos, 15.0f, 0.0f, PID_Follow_Pos,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);

    float PID_Follow_Spd[3] = {1.5f,   0.0f,   0.0f};
    PID_Init(&chassis_ctrl.Follow_Spd, 15.0f, 1.0f, PID_Follow_Spd,
             0, 0, 0, 0, 0, Integral_Limit | ErrorHandle);
    // 功率控制初始化及参数配置
    Power_Ctrl_Init(&chassis_model);
    for(int i=0; i<4; i++) {
        // 配置驱动轮节点，3508 功率模型
        drive_nodes[i].state = &m_states[i];
        drive_nodes[i].model = &MODEL_M3508;
    }
    pwr_groups[0].nodes = drive_nodes;
    pwr_groups[0].node_count = 4;
    //向系统下发底盘当前状态，准备中
    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    return 1;
}

/**
 * @brief 底盘控制任务
 */
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, const IMU_Data_t *imu, float dt)
{
    // 空指针保护
    if (c_motor == NULL) {
        System_State_Report(ID_CHASSIS, STATUS_ERROR);
        return;
    }
    static float cur_vx_gimbal = 0.0f, cur_vy_gimbal = 0.0f, cur_vw = 0.0f;
    // 状态汇报
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
        // 清空PID
        for (int i = 0; i < 4; i++) {
            PID_Clear(&chassis_ctrl.Drive_S[i]);
        }
        PID_Clear(&chassis_ctrl.Follow_Pos);
        PID_Clear(&chassis_ctrl.Follow_Spd);
        // 清空斜坡函数
        cur_vx_gimbal = 0.0f;
        cur_vy_gimbal = 0.0f;
        cur_vw        = 0.0f;
    }
    else
    {
        float vw_tar = chassis_cmd.target_vw;
        // 底盘跟随模式下，计算底盘跟随PID
        if (chassis_cmd.mode == CHASSIS_CMD_FOLLOW) {
            PID_Calculate(&chassis_ctrl.Follow_Pos, chassis_cmd.offset_angle, 0.0f);
            vw_tar = PID_Calculate(&chassis_ctrl.Follow_Spd, imu->gyro[2], chassis_ctrl.Follow_Pos.Output);
        }
        // 非对称梯形加减速
        cur_vx_gimbal = Ramp_Calc(chassis_cmd.target_vx, cur_vx_gimbal, 5.0f, 100.0f,dt);
        cur_vy_gimbal = Ramp_Calc(chassis_cmd.target_vy, cur_vy_gimbal, 5.0f, 100.0f,dt);
        cur_vw        = Ramp_Calc(vw_tar,        cur_vw,        350.0f,  400.0f,dt);
        // 底盘坐标系旋转矩阵
        float cos_theta = arm_cos_f32(chassis_cmd.offset_angle);
        float sin_theta = arm_sin_f32(chassis_cmd.offset_angle);
        float cur_vx_chassis = cur_vx_gimbal * cos_theta + cur_vy_gimbal * sin_theta;
        float cur_vy_chassis = cur_vy_gimbal * cos_theta - cur_vx_gimbal * sin_theta;

        for (int i = 0; i < 4; i++)
        {
            chassis_ctrl.chassis_feedback.wheel_rpm[i] = c_motor->DJI_3508_Chassis[i].Speed_now;
        }
        Chassis_Forward(&chassis_ctrl.chassis_cfg,&chassis_ctrl.chassis_feedback);

        PID_Calculate(&chassis_ctrl.vx, chassis_ctrl.chassis_feedback.vx, cur_vx_chassis);
        PID_Calculate(&chassis_ctrl.vy, chassis_ctrl.chassis_feedback.vy, cur_vy_chassis);
        PID_Calculate(&chassis_ctrl.vw, chassis_ctrl.chassis_feedback.vw, cur_vw);

        // 逆运动学与速度环 PID 计算
        Chassis_Force(&chassis_ctrl.chassis_cfg,chassis_ctrl.vx.Output,chassis_ctrl.vy.Output,chassis_ctrl.vw.Output
            ,&chassis_ctrl.chassis_feedback,&chassis_ctrl.chassis_command);
        // 功率控制
        for(int i = 0; i < 4; i++) {
            m_states[i].speed_rpm = c_motor->DJI_3508_Chassis[i].Speed_now;
            m_states[i].original_cmd = chassis_ctrl.chassis_command.wheel_torque_raw[i];
        }
        bool trigger_discharge = chassis_cmd.is_cap_on;// 输入电容开启标志
        float cap_board_limit = 0.0f;
        float final_limit = 0.0f;
        if (Referee.offline.is_online) {
            final_limit = Chassis_Power_Arbitrator(
                                    Referee.robot_status.chassis_power_limit,
                                    Referee.power_heat_data.buffer_energy,
                                    1, &cap, &trigger_discharge, &cap_board_limit);
        }
        else {
            trigger_discharge = FALSE;
            cap_board_limit = 45.0f;//
            final_limit = 75.0f;
        }
        Power_Ctrl_Calculate(&chassis_model, final_limit, pwr_groups, 1);
        for(int i = 0; i < 4; i++) {
            chassis_ctrl.chassis_command.wheel_torque_raw[i] = m_states[i].limited_cmd;
        }
        // 下发电容通讯数据
        CapSetData_t cap_cmd = {0};
        cap_cmd.Control.power_key     = trigger_discharge;
        cap_cmd.Control.capPowerLimit = (uint8_t)cap_board_limit;
        cap_cmd.Control.buffer_now    = (uint8_t)Referee.power_heat_data.buffer_energy;
        cap_cmd.Control.robot_state   = (Referee.robot_status.current_HP > 0) ? 1 : 0;
        Power_Cap_Tx(&hcan1, 0x252, &cap_cmd);
    }

    if (!is_system_locked)
    {
        DJI_Motor_Send(&hcan2, 0x200,
                       (int16_t)chassis_ctrl.chassis_command.wheel_torque_raw[0],
                       (int16_t)chassis_ctrl.chassis_command.wheel_torque_raw[1],
                       (int16_t)chassis_ctrl.chassis_command.wheel_torque_raw[2],
                       (int16_t)chassis_ctrl.chassis_command.wheel_torque_raw[3]);
    }
}

// 超级电容与缓冲能量调参宏定义
#define BUFFER_COMP_KP      2.5f    // 缓冲能量补偿的比例系数 (Kp)
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
    if (boost_intent && cap_data->get.Cap_Capacity > MIN_CAP_VOLTAGE && cap_data->get.offline.is_online == 1)
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
    return final_target_power; // 返回给电机的最终功率限制
}