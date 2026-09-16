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
#include "Vofa.h"

#define WHEEL_RADIUS_M       0.060f
#define WHEEL_GEAR_RATIO     15.76f
#define LEFT_WHEEL_SIGN      -1.0f
#define RIGHT_WHEEL_SIGN     1.0f
#define BODY_MASS_KG         8.0f
#define GRAVITY_MPS2         9.81f
#define LEG_LENGTH_TARGET_M  0.170f
#define NM_ENCODER (1.0f/(0.315*20)*16384)
#define LESO_ENABLE           0
static Chassis_Ctrl_Block_t chassis_ctrl;
static uint16_t save_cnt = 0;
Chassis_Control_Mode_t MODE;

//功率控制

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
    LQR_Init(&chassis_ctrl.lqr);
    LESO_Init(&chassis_ctrl.leso);

    float pid_pos[3] = {1500.0f, 0.0f, 20.0f};
    float pid_vel[3] = {0.0f, 0.0f, 0.0f};
    PID_Init(&chassis_ctrl.left_length_pos, 150.0f, 0.0f,
        pid_pos, 0, 0, 0, 0, 0, 0);
    PID_Init(&chassis_ctrl.left_length_vel, 50.0f, 0.0f,
        pid_vel, 0, 0, 0, 0, 0, 0);
    PID_Init(&chassis_ctrl.right_length_pos, 150.0f, 0.0f,
        pid_pos, 0, 0, 0, 0, 0, 0);
    PID_Init(&chassis_ctrl.right_length_vel, 50.0f, 0.0f,
        pid_vel, 0, 0, 0, 0, 0, 0);
    float pid_roll[3] = {1500.0f, 0.0f, 0.0f};
    PID_Init(&chassis_ctrl.roll, 100.0f, 0.0f,
        pid_roll, 0, 0, 0, 0, 0, 0);

    float pid_joint_pos[3] = {5.0f, 0.0f, 0.0f};
    float pid_joint_vel[3] = {12.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; i++) {
        PID_Init(&chassis_ctrl.joint_pos[i], 3.0f, 0.0f,
            pid_joint_pos, 0, 0, 0, 0, 0, 0);
        PID_Init(&chassis_ctrl.joint_vel[i], 5.0f, 0.0f,
            pid_joint_vel, 0, 0, 0, 0, 0, 0);
    }


    chassis_ctrl.wheel_speed_mps = 0.0f;
    chassis_ctrl.body_position_m = 0.0f;
    chassis_ctrl.body_velocity_mps = 0.0f;
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[0],-1.50560224f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[1],0.489915133f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[2],-2.08909842f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[3],0.292033000f);
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
    chassis_ctrl.wheel_speed_mps = 0.5f * WHEEL_RADIUS_M * (left_wheel_radps + right_wheel_radps);
    Estimator_Set_WheelSpeed(&chassis_ctrl.odometry,
                             chassis_ctrl.wheel_speed_mps);
    if (imu != NULL) {
        /* Leg-length position control plus vertical support/gravity feedforward. */
        PID_Calculate(&chassis_ctrl.left_length_pos,chassis_ctrl.vmc.left.length[0],chassis_cmd.target_length);
        const float left_pid = PID_Calculate(&chassis_ctrl.left_length_vel,chassis_ctrl.vmc.left.length[1],0);
        PID_Calculate(&chassis_ctrl.right_length_pos,chassis_ctrl.vmc.right.length[0],chassis_cmd.target_length);
        const float right_pid = PID_Calculate(&chassis_ctrl.right_length_vel,chassis_ctrl.vmc.right.length[1],0);

        PID_Calculate(&chassis_ctrl.roll, imu->roll * DEG2RAD, chassis_cmd.target_roll);

        const float left_body_theta = chassis_ctrl.vmc.left.theta[0] + pitch;
        const float right_body_theta = chassis_ctrl.vmc.right.theta[0] + pitch;
        const float support_each = 0.5f * BODY_MASS_KG * GRAVITY_MPS2;
        const float left_cos = fmaxf(cosf(left_body_theta), 0.2f);
        const float right_cos = fmaxf(cosf(right_body_theta), 0.2f);
        chassis_ctrl.left_leg_force_n = chassis_ctrl.left_length_pos.Output + left_pid + chassis_ctrl.roll.Output;// + support_each / left_cos;
        chassis_ctrl.right_leg_force_n = chassis_ctrl.right_length_pos.Output + right_pid - chassis_ctrl.roll.Output;// + support_each / right_cos;

        /* Parameter-observation stage: map LQR virtual pitch torques and
         * leg axial forces through VMC while keeping motor outputs disabled. */
        VMC_ForceToTorque(chassis_ctrl.vmc.JRM_l,
                          chassis_ctrl.left_leg_force_n,
                          chassis_ctrl.lqr.u[0],
                          &chassis_ctrl.vmc.left);
        VMC_ForceToTorque(chassis_ctrl.vmc.JRM_r,
                          chassis_ctrl.right_leg_force_n,
                          chassis_ctrl.lqr.u[1],
                          &chassis_ctrl.vmc.right);
    }
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
        BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
        DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
    }
    else {
        if (chassis_cmd.mode == CHASSIS_CMD_FREE) {
            applied_input[0] = chassis_ctrl.lqr.u[0];
            applied_input[1] = chassis_ctrl.lqr.u[1];
            // BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.vmc.left.Tp_front,
            //     -chassis_ctrl.vmc.right.Tp_front,
            //     chassis_ctrl.vmc.left.Tp_back,
            //     -chassis_ctrl.vmc.right.Tp_back);
            BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
            DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
        }

        if (chassis_cmd.mode == CHASSIS_CMD_FOLLOW) {

            switch (MODE) {
                case CTRL_SAVE:
                    // Handle save mode
                    VMC_InverseKinematics(&chassis_ctrl.vmc,0.15f,0,imu->pitch * DEG2RAD,&left_front_target,&left_back_target);
                    VMC_InverseKinematics(&chassis_ctrl.vmc,0.15f,0,imu->pitch * DEG2RAD,&right_front_target,&right_back_target);
                    float pos_target[4];
                    pos_target[0] = l_motor->BM_P1010B_Leg[0].pos_single + normalize_to_pi(left_front_target - l_motor->BM_P1010B_Leg[0].pos_rad);
                    pos_target[1] = l_motor->BM_P1010B_Leg[1].pos_single + normalize_to_pi(-right_front_target - l_motor->BM_P1010B_Leg[1].pos_rad);
                    pos_target[2] = l_motor->BM_P1010B_Leg[2].pos_single + normalize_to_pi(left_back_target - l_motor->BM_P1010B_Leg[2].pos_rad);
                    pos_target[3] = l_motor->BM_P1010B_Leg[3].pos_single + normalize_to_pi(-right_back_target - l_motor->BM_P1010B_Leg[3].pos_rad);
                    PID_Calculate(&chassis_ctrl.joint_pos[0],l_motor->BM_P1010B_Leg[0].pos_single,pos_target[0]);
                    PID_Calculate(&chassis_ctrl.joint_pos[1],l_motor->BM_P1010B_Leg[1].pos_single,pos_target[1]);
                    PID_Calculate(&chassis_ctrl.joint_pos[2],l_motor->BM_P1010B_Leg[2].pos_single,pos_target[2]);
                    PID_Calculate(&chassis_ctrl.joint_pos[3],l_motor->BM_P1010B_Leg[3].pos_single,pos_target[3]);
                    PID_Calculate(&chassis_ctrl.joint_vel[0],l_motor->BM_P1010B_Leg[0].vel_rad,chassis_ctrl.joint_pos[0].Output);
                    PID_Calculate(&chassis_ctrl.joint_vel[1],l_motor->BM_P1010B_Leg[1].vel_rad,chassis_ctrl.joint_pos[1].Output);
                    PID_Calculate(&chassis_ctrl.joint_vel[2],l_motor->BM_P1010B_Leg[2].vel_rad,chassis_ctrl.joint_pos[2].Output);
                    PID_Calculate(&chassis_ctrl.joint_vel[3],l_motor->BM_P1010B_Leg[3].vel_rad,chassis_ctrl.joint_pos[3].Output);
                    if (fabsf(imu->roll) < 5.0f) {
                        BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.joint_vel[0].Output,
                                chassis_ctrl.joint_vel[1].Output,
                                chassis_ctrl.joint_vel[2].Output,
                                chassis_ctrl.joint_vel[3].Output);
                        DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
                    }
                    if (chassis_ctrl.vmc.left.theta[0] < 0.1f &&
                        chassis_ctrl.vmc.left.theta[0] > -0.05f &&
                        chassis_ctrl.vmc.right.theta[0] < 0.1f &&
                        chassis_ctrl.vmc.right.theta[0] > -0.05f &&
                        fabsf(imu->pitch) < 12.0f &&
                        chassis_ctrl.vmc.left.length[0] < 0.17f &&
                        chassis_ctrl.vmc.right.length[0] < 0.17f &&
                        fabsf(chassis_ctrl.wheel_speed_mps) < 0.03f) {
                        save_cnt ++;
                        Estimator_Leg_Init(&chassis_ctrl.odometry);

                        if (save_cnt >= 50) {
                            chassis_cmd.target_length = 0.17f;
                            MODE = CTRL_STAND;
                            save_cnt = 0;
                        }
                    }
                    else {
                        save_cnt = 0;
                    }

                    break;
                case CTRL_STAND:
                    if (fabsf(chassis_ctrl.vmc.left.theta[0]) > 3.1f * chassis_ctrl.vmc.left.length[0] ||
                        fabsf(chassis_ctrl.vmc.right.theta[0]) > 3.1f * chassis_ctrl.vmc.right.length[0]) {
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
                            chassis_ctrl.lqr.target.position_m = chassis_ctrl.odometry.outstate.s + 3.7f;
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
                        // Handle stand mode
                        chassis_ctrl.lqr.u[2] = MATH_Limit_float(chassis_ctrl.lqr.u[2], -6, 6);
                        chassis_ctrl.lqr.u[3] = MATH_Limit_float(chassis_ctrl.lqr.u[3], -6, 6);
                        for (int input = 0; input < LQR_OUTPUT_SIZE; input++) {
                            applied_input[input] = chassis_ctrl.lqr.u[input];
                        }
                        BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.vmc.left.Tp_front,
                            -chassis_ctrl.vmc.right.Tp_front,
                            chassis_ctrl.vmc.left.Tp_back,
                            -chassis_ctrl.vmc.right.Tp_back);
                        DJI_Motor_Send(&hfdcan1, 0x200,
                                       (int16_t)( chassis_ctrl.lqr.u[3] * NM_ENCODER),
                                       0,
                                       (int16_t)( -chassis_ctrl.lqr.u[2] * NM_ENCODER),
                                       0);
                    }
                    break;
                case CTRL_JUMP:
                    // Handle jump mode
                    break;
                default:break;
            }
        }
    }
    // else {
    //
    // }
    LESO_SetAppliedInput(&chassis_ctrl.leso, applied_input,
                         chassis_ctrl.lqr.u_eq);
    VOFA_JustFloat(&huart1,2,
        chassis_ctrl.wheel_speed_mps
        );
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
