//
// Created by CaoKangqi on 2026/6/20.
//
#include "Chassis_Ctrl.h"
#include "All_define.h"
#include "Chassis_ESKF.h"
#include "Robot_Config.h"
#include "Power_CAP.h"
#include "System_State.h"
#include "Robot_Cmd.h"
#include "Classic_Control.h"
#include <math.h>
#include "Kalman_Observer.h"
#include "Power_Ctrl.h"
#include "Vofa.h"

#define WHEEL_RADIUS_M                    0.060f  // 车轮半径，用于轮速换算和质心离地高度计算
#define WHEEL_GEAR_RATIO                  15.76f  // 轮毂电机到车轮的减速比
#define WHEEL_HALF_TRACK_M                0.20f   // 半轮距，用于ROLL力矩与左右支撑力差换算
#define LEFT_WHEEL_SIGN                  -1.0f    // 左轮速度反馈方向修正
#define RIGHT_WHEEL_SIGN                  1.0f    // 右轮速度反馈方向修正
#define NM_ENCODER                       (1.0f/(0.315f*20.0f)*16384.0f) // 轮电机力矩到电流指令的换算系数
#define LESO_ENABLE                       1       // LESO补偿使能开关
#define JUMP_THRUST_FORCE_N               500.0f
#define JUMP_RETRACT_LENGTH_M             0.17f
#define JUMP_RETRACT_COMPLETE_TOL_M       0.003f

static Chassis_Ctrl_Block_t chassis_ctrl;
static uint16_t save_cnt = 0;
static uint16_t jump_cnt = 0;
Chassis_Control_Mode_t MODE;

//功率控制
static Motor_Power_State_t m_states[2];//底盘共4个电机
static Power_Motion_Node_t motion_nodes[2];          /**< 四轮模型、运动电流分量及电流上限。 */
static Power_Motion_Result_t chassis_power_result;   /**< 本周期保留比例、功率预测和分配状态。 */

static void Chassis_ResetSupportController(void)
{
    Body_Support_Reset(&chassis_ctrl.support);
}

static void Chassis_UpdateSupportForces(const IMU_Data_t *imu, float dt)
{
    Body_Support_Update(&chassis_ctrl.support,
                        &chassis_ctrl.vmc,
                        imu,
                        chassis_cmd.target_length,
                        chassis_cmd.target_roll,
                        chassis_ctrl.odometry.outstate.dot_s,
                        dt);
}

static void Chassis_UpdateSupportForcesToLength(const IMU_Data_t *imu,
                                                float target_length,
                                                float dt)
{
    Body_Support_Update(&chassis_ctrl.support,
                        &chassis_ctrl.vmc,
                        imu,
                        target_length,
                        chassis_cmd.target_roll,
                        chassis_ctrl.odometry.outstate.dot_s,
                        dt);
}

static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge,
                                      float *out_cap_limit);

uint8_t Chassis_Control_Init()
{
    VMC_Init(&chassis_ctrl.vmc, VMC_L1_LENGTH, VMC_L2_LENGTH,
             VMC_L3_LENGTH, VMC_L4_LENGTH);
    Estimator_Leg_Init(&chassis_ctrl.odometry);
    Estimator_Set_WheelHalfTrack(&chassis_ctrl.odometry,
                                 WHEEL_HALF_TRACK_M);
    LQR_Init(&chassis_ctrl.lqr);
    LESO_Init(&chassis_ctrl.leso);
    Body_Support_Init(&chassis_ctrl.support);
    Contact_Detector_Init(&chassis_ctrl.contact_left,0);
    Contact_Detector_Init(&chassis_ctrl.contact_right,0);

    float pid_joint_pos[3] = {8.0f, 0.0f, 0.0f};
    float pid_joint_vel[3] = {12.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; i++) {
        PID_Init(&chassis_ctrl.joint_pos[i], 3.5f, 0.0f,
            pid_joint_pos, 0, 0, 0, 0, 0, 0);
        PID_Init(&chassis_ctrl.joint_vel[i], 6.0f, 0.0f,
            pid_joint_vel, 0, 0, 0, 0, 0, 0);
    }


    chassis_ctrl.wheel_speed_mps = 0.0f;
    chassis_ctrl.body_position_m = 0.0f;
    chassis_ctrl.body_velocity_mps = 0.0f;
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[0],-1.50560224f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[1],0.489915133f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[2],-2.08909842f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[3],0.292033000f);

    chassis_power_result = (Power_Motion_Result_t){0};
    for (uint8_t wheel_index = 0; wheel_index < 2; wheel_index++) {
        motion_nodes[wheel_index].motor.state = &m_states[wheel_index];
        motion_nodes[wheel_index].motor.model = &MODEL_M3508;
        motion_nodes[wheel_index].max_cmd = 16000.0f;
    }
    //向系统下发底盘当前状态，准备中
    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    return 1;
}

float left_front_target;
float left_back_target;
float right_front_target;
float right_back_target;
/**
 * @brief 底盘控制任务
 */
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor, const Leg_Motor_Group_t *l_motor,const IMU_Data_t *imu, float dt)
{
    if (c_motor == NULL || l_motor == NULL) {
        System_State_Report(ID_CHASSIS, STATUS_ERROR);
        return;
    }
    bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                             sys_state.global_mode == GLOBAL_STANDBY ||
                             sys_state.global_mode == GLOBAL_INIT_STAGE ||
                             sys_state.global_mode == GLOBAL_MODULE_ERROR);
    const bool leso_learning_enabled =
        LESO_ENABLE && !is_system_locked &&
        chassis_cmd.mode == CHASSIS_CMD_FOLLOW;

    /* Keep feedback kinematics alive even while outputs are safety-locked. */
    const float pitch = (imu != NULL) ? imu->pitch * DEG2RAD : 0.0f;
    VMC_Update(&chassis_ctrl.vmc, dt, pitch,
        l_motor->BM_P1010B_Leg[0].pos_rad,
        l_motor->BM_P1010B_Leg[2].pos_rad,
        l_motor->BM_P1010B_Leg[1].pos_rad,
        l_motor->BM_P1010B_Leg[3].pos_rad);
    VMC_TorqueToForce(chassis_ctrl.vmc.JRM_l,
                      l_motor->BM_P1010B_Leg[0].IQ,
                      l_motor->BM_P1010B_Leg[2].IQ,
                      &chassis_ctrl.vmc.left);
    /* Right joint feedback uses the opposite sign convention. */
    VMC_TorqueToForce(chassis_ctrl.vmc.JRM_r,
                      -l_motor->BM_P1010B_Leg[1].IQ,
                      -l_motor->BM_P1010B_Leg[3].IQ,
                      &chassis_ctrl.vmc.right);

    /* Wheel odometry: motor speed feedback is RPM at the motor shaft. */
    const float left_wheel_radps =
        LEFT_WHEEL_SIGN * (float)c_motor->DJI_3508_Chassis[0].Speed_now
        * RPM_TO_RADS / WHEEL_GEAR_RATIO;
    const float right_wheel_radps =
        RIGHT_WHEEL_SIGN * (float)c_motor->DJI_3508_Chassis[1].Speed_now
        * RPM_TO_RADS / WHEEL_GEAR_RATIO;
    const float left_wheel_speed_mps = WHEEL_RADIUS_M * left_wheel_radps;
    const float right_wheel_speed_mps = WHEEL_RADIUS_M * right_wheel_radps;
    chassis_ctrl.wheel_speed_mps =
        0.5f * (left_wheel_speed_mps + right_wheel_speed_mps);
    Estimator_Set_WheelSpeeds(&chassis_ctrl.odometry,
                              left_wheel_speed_mps,
                              right_wheel_speed_mps);
    Buffer_Calc(&Meter,dt,50);
    Contact_Detector_Update(&chassis_ctrl.contact_left, &chassis_ctrl.vmc.left, imu->accel[2]);
    Contact_Detector_Update(&chassis_ctrl.contact_right, &chassis_ctrl.vmc.right, imu->accel[2]);

    chassis_ctrl.body_position_m = chassis_ctrl.odometry.outstate.s;
    chassis_ctrl.body_velocity_mps = chassis_ctrl.odometry.outstate.dot_s;
    if (!Is_Group_Online(CHASSIS)) {
        System_State_Report(ID_CHASSIS, STATUS_LOST);
    }
    else{System_State_Report(ID_CHASSIS, STATUS_RUN);}
    // 判断系统状态
    float applied_input[LQR_OUTPUT_SIZE] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (chassis_cmd.mode == CHASSIS_CMD_SAFE || is_system_locked)
    {
        Chassis_ResetSupportController();
        BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
        DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
    }
    else {
        if (chassis_cmd.mode == CHASSIS_CMD_FOLLOW) {

            switch (MODE) {
                case CTRL_SAVE:
                    Chassis_ResetSupportController();
                    // Handle save mode
                    // 逆解关节电机目标
                    VMC_InverseKinematics(&chassis_ctrl.vmc,0.15f,0,imu->pitch * DEG2RAD,&left_front_target,&left_back_target);
                    VMC_InverseKinematics(&chassis_ctrl.vmc,0.15f,0,imu->pitch * DEG2RAD,&right_front_target,&right_back_target);
                    float pos_target[4];
                    // 关节电机目标值归一化
                    pos_target[0] = l_motor->BM_P1010B_Leg[0].pos_single +
                                    normalize_to_pi(left_front_target - l_motor->BM_P1010B_Leg[0].pos_rad);
                    pos_target[2] = l_motor->BM_P1010B_Leg[2].pos_single +
                                    normalize_to_pi(left_back_target - l_motor->BM_P1010B_Leg[2].pos_rad);
                    pos_target[1] = l_motor->BM_P1010B_Leg[1].pos_single +
                                    normalize_to_pi(-right_front_target - l_motor->BM_P1010B_Leg[1].pos_rad);
                    pos_target[3] = l_motor->BM_P1010B_Leg[3].pos_single +
                                    normalize_to_pi(-right_back_target - l_motor->BM_P1010B_Leg[3].pos_rad);
                    for (int i=0;i<4;i++) {
                        PID_Calculate(&chassis_ctrl.joint_pos[i],l_motor->BM_P1010B_Leg[i].pos_single,pos_target[i]);
                        PID_Calculate(&chassis_ctrl.joint_vel[i],l_motor->BM_P1010B_Leg[i].vel_rad,chassis_ctrl.joint_pos[i].Output);
                    }
                    BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.joint_vel[0].Output,
                    chassis_ctrl.joint_vel[1].Output,
                    chassis_ctrl.joint_vel[2].Output,
                    chassis_ctrl.joint_vel[3].Output);
                        //BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
                    DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
                    if (fabsf(chassis_ctrl.vmc.left.theta[0]) < 0.1f &&
                        fabsf(chassis_ctrl.vmc.right.theta[0]) < 0.1f &&
                        fabsf(imu->pitch) < 12.0f &&
                        chassis_ctrl.vmc.left.length[0] < 0.17f &&
                        chassis_ctrl.vmc.right.length[0] < 0.17f) { //&& fabsf(chassis_ctrl.wheel_speed_mps) < 0.06f
                        save_cnt ++;
                        Estimator_Leg_Init(&chassis_ctrl.odometry);

                        if (save_cnt >= 50) {
                            chassis_cmd.target_length = 0.16f;
                            MODE = CTRL_STAND;
                            save_cnt = 0;
                        }
                    }
                    else {
                        save_cnt = 0;
                    }

                    break;
                case CTRL_STAND:
                    if (fabsf(chassis_ctrl.vmc.left.theta[0]) > 3.5f * chassis_ctrl.vmc.left.length[0] ||
                        fabsf(chassis_ctrl.vmc.right.theta[0]) > 3.5f * chassis_ctrl.vmc.right.length[0] ||
                        fabsf(imu->pitch) > 50.0f || fabsf(imu->roll) > 50.0f) {
                        Chassis_ResetSupportController();
                        BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
                        DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
                        save_cnt = 0;
                        chassis_ctrl.stand_initialized = false;
                        MODE = CTRL_SAVE;
                    }
                    else {
                        // 第一次进入 CTRL_STAND 时重置 LQR / LESO
                        if (!chassis_ctrl.stand_initialized) {
                            LQR_Init(&chassis_ctrl.lqr);
                            LESO_Init(&chassis_ctrl.leso);

                            // 同步目标位置和偏航，避免 x_ref 突变
                            chassis_ctrl.lqr.target.position_m = chassis_ctrl.odometry.outstate.s + 1.9f;
                            chassis_ctrl.lqr.target.yaw_rad = imu->YawTotalAngle * DEG2RAD;
                            chassis_ctrl.lqr.target.velocity_mps = 0.0f;
                            chassis_ctrl.lqr.target.yaw_rate_radps = 0.0f;
                            // 其他目标保持 0，让腿回到平衡角度

                            chassis_ctrl.stand_initialized = true;
                        }
                        Estimator_Task(&chassis_ctrl.odometry, *imu, dt);
                        LQR_SetTarget(&chassis_ctrl.lqr,
                          chassis_cmd.target_vx,
                          chassis_cmd.target_vw,
                          0.0f, 0.0f, 0.0f, dt);
                        LQR_Update(&chassis_ctrl.lqr, &chassis_ctrl.vmc,
                                   &chassis_ctrl.odometry, imu, &chassis_ctrl.leso,
                                   leso_learning_enabled);
                        chassis_ctrl.lqr.u[2] = MATH_Limit_float(chassis_ctrl.lqr.u[2], -6, 6);
                        chassis_ctrl.lqr.u[3] = MATH_Limit_float(chassis_ctrl.lqr.u[3], -6, 6);
                        Chassis_UpdateSupportForces(imu, dt);
                        VMC_ForceToTorque(chassis_ctrl.vmc.JRM_l,
                                          chassis_ctrl.support.left_leg_force_n,
                                          chassis_ctrl.lqr.u[0],
                                          &chassis_ctrl.vmc.left);
                        VMC_ForceToTorque(chassis_ctrl.vmc.JRM_r,
                                          chassis_ctrl.support.right_leg_force_n,
                                          chassis_ctrl.lqr.u[1],
                                          &chassis_ctrl.vmc.right);
                        BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.vmc.left.Tp_front,
                            -chassis_ctrl.vmc.right.Tp_front,
                            chassis_ctrl.vmc.left.Tp_back,
                            -chassis_ctrl.vmc.right.Tp_back);
                        int16_t wheel_raw[2];
                        wheel_raw[0] = (int16_t)(chassis_ctrl.lqr.u[3]
                            * NM_ENCODER);
                        wheel_raw[1] = (int16_t)(-chassis_ctrl.lqr.u[2]
                            * NM_ENCODER);
                        /* 功率预测使用当前电机 RPM。 */
                        for (uint8_t wheel_index = 0; wheel_index < 2; wheel_index++) {
                            m_states[wheel_index].speed_rpm = c_motor->DJI_3508_Chassis[wheel_index].Speed_now;
                            motion_nodes[wheel_index].translation_cmd = wheel_raw[wheel_index];
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
                            final_limit = Chassis_Power_Arbitrator(
                                                    5000.0f,Meter.buffer_energy,
                                                    1, &cap, &trigger_discharge, &cap_board_limit);
                        }
                        if (final_limit < 0.0f) final_limit = 0.0f;
                        /* 跟随模式优先旋转；小陀螺及其他运行模式优先保留平移。 */
                        Power_Motion_Priority_t power_priority = POWER_PRIORITY_TRANSLATION;
                        Power_Motion_Status_t power_status = Power_Ctrl_Allocate_Motion_With_Priority(
                            final_limit, motion_nodes, 2, power_priority, &chassis_power_result);
                        /* 仅发送已通过功率和电流约束检查的结果，分配失败时本周期输出零电流。 */
                        bool power_output_valid = power_status == POWER_MOTION_OK ||
                                                  power_status == POWER_MOTION_LIMITED;
                        for (uint8_t wheel_index = 0; wheel_index < 2; wheel_index++) {
                            wheel_raw[wheel_index] = power_output_valid ? m_states[wheel_index].limited_cmd : 0.0f;
                        }
                        DJI_Motor_Send(&hfdcan1, 0x200,wheel_raw[0],0,wheel_raw[1],0);
                        applied_input[0] = chassis_ctrl.lqr.u[0];
                        applied_input[1] = chassis_ctrl.lqr.u[1];
                        applied_input[2] = -wheel_raw[1]/NM_ENCODER;
                        applied_input[3] = wheel_raw[0]/NM_ENCODER;
                    }
                    break;
                case CTRL_JUMP:
                    break;
                default:break;
            }
        }
    }
    LESO_SetAppliedInput(&chassis_ctrl.leso, applied_input,
                         chassis_ctrl.lqr.u_eq);
        VOFA_JustFloat(&huart1, 11,
            chassis_ctrl.vmc.left.support_force,
            chassis_ctrl.vmc.left.support_torque,
            chassis_ctrl.vmc.left.theta[0],
            chassis_ctrl.vmc.left.theta[1],
            chassis_ctrl.vmc.left.theta[2],
            chassis_ctrl.vmc.left.length[0],
            chassis_ctrl.vmc.left.length[1],
            chassis_ctrl.vmc.left.length[2],
            imu->accel[2],
            (float)chassis_ctrl.contact_left.is_contact,
            (float)chassis_ctrl.contact_right.is_contact);

}

// 超级电容与缓冲能量调参宏定义
#define BUFFER_COMP_KP      5.0f    // 缓冲能量补偿的比例系数 (Kp)
#define TARGET_BUFFER       30.0f   // 目标期望缓冲能量 (J)
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
