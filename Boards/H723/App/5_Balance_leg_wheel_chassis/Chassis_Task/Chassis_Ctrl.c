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

#define WHEEL_RADIUS_M                    0.060f  // 车轮半径，用于轮速换算和质心离地高度计算
#define WHEEL_GEAR_RATIO                  15.76f  // 轮毂电机到车轮的减速比
#define WHEEL_HALF_TRACK_M                0.20f   // 半轮距，用于ROLL力矩与左右支撑力差换算
#define LEFT_WHEEL_SIGN                  -1.0f    // 左轮速度反馈方向修正
#define RIGHT_WHEEL_SIGN                  1.0f    // 右轮速度反馈方向修正
#define BODY_MASS_KG                     17.5f    // 参与高度和ROLL动力学计算的整车质量
#define GRAVITY_MPS2                     9.81f    // 重力加速度
#define NM_ENCODER                       (1.0f/(0.315f*20.0f)*16384.0f) // 轮电机力矩到电流指令的换算系数
#define LESO_ENABLE                       1       // LESO补偿使能开关

#define HEIGHT_NATURAL_FREQ_HZ            2.0f   // 高度闭环自然频率，越大高度响应越快
#define HEIGHT_DAMPING_RATIO              0.8f   // 高度闭环阻尼比，越大振荡越小
#define ROLL_NATURAL_FREQ_HZ              3.0f   // ROLL闭环自然频率，越大回正和压弯越快
#define ROLL_DAMPING_RATIO                0.4f  // ROLL闭环阻尼比，越大ROLL振荡越小
#define BODY_ROLL_INERTIA_KGM2            0.36f  // 机身绕质心ROLL轴的转动惯量
#define BODY_COM_OFFSET_FROM_HIP_M        0.0f   // 质心相对髋部的竖直偏移，向上为正
#define LEG_AXIAL_FORCE_MAX_N           250.0f   // 单腿允许输出的最大轴向力
#define LEG_FORCE_RATE_LIMIT_NPS       2500.0f   // 单腿轴向力每秒最大变化量
#define LEG_VERTICAL_COS_MIN              0.35f  // 腿轴向力转竖直力时的最小余弦保护值
#define LEG_THEORETICAL_MAX_LENGTH_M      0.33f  // 虚拟腿理论最大长度，用于动态限制机身高度

#define CENTRIPETAL_LPF_HZ                4.0f   // 向心加速度低通滤波截止频率
#define CENTRIPETAL_ACCEL_LIMIT_MPS2     5.0f   // 允许参与补偿的最大向心加速度
#define ROLL_LEAN_SIGN                   -1.0f   // 主动内倾方向与IMU ROLL正方向的对应关系
#define ROLL_LEAN_LIMIT_RAD              (5.0f * DEG2RAD) // 主动压弯允许的最大ROLL倾角

static Chassis_Ctrl_Block_t chassis_ctrl;
static uint16_t save_cnt = 0;
Chassis_Control_Mode_t MODE;

static float clamp_float(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
}

static float rate_limit_float(float target, float previous, float max_step)
{
    return previous + clamp_float(target - previous, -max_step, max_step);
}

static void Chassis_ResetSupportController(void)
{
    chassis_ctrl.body_height_m = 0.0f;
    chassis_ctrl.body_height_rate_mps = 0.0f;
    chassis_ctrl.limited_height_target_m = 0.0f;
    chassis_ctrl.lateral_acceleration_mps2 = 0.0f;
    chassis_ctrl.roll_lean_target_rad = 0.0f;
    chassis_ctrl.effective_roll_target_rad = 0.0f;
    chassis_ctrl.total_vertical_force_n = 0.0f;
    chassis_ctrl.roll_moment_nm = 0.0f;
    chassis_ctrl.left_leg_force_n = 0.0f;
    chassis_ctrl.right_leg_force_n = 0.0f;
    chassis_ctrl.last_left_leg_force_n = 0.0f;
    chassis_ctrl.last_right_leg_force_n = 0.0f;
}

static void Chassis_UpdateSupportForces(const IMU_Data_t *imu, float dt)
{
    if (imu == NULL) {
        Chassis_ResetSupportController();
        return;
    }

    const float left_theta = chassis_ctrl.vmc.left.theta[0];
    const float right_theta = chassis_ctrl.vmc.right.theta[0];
    const float left_length = chassis_ctrl.vmc.left.length[0];
    const float right_length = chassis_ctrl.vmc.right.length[0];
    const float left_cos_raw = cosf(left_theta);
    const float right_cos_raw = cosf(right_theta);
    const float left_cos = fmaxf(left_cos_raw, LEG_VERTICAL_COS_MIN);
    const float right_cos = fmaxf(right_cos_raw, LEG_VERTICAL_COS_MIN);
    const float safe_dt = clamp_float(dt, 0.0f, 0.01f);

    const float left_height = left_length * left_cos_raw;
    const float right_height = right_length * right_cos_raw;
    const float left_height_rate =
        chassis_ctrl.vmc.left.length[1] * left_cos_raw
        - left_length * sinf(left_theta) * chassis_ctrl.vmc.left.theta[1];
    const float right_height_rate =
        chassis_ctrl.vmc.right.length[1] * right_cos_raw
        - right_length * sinf(right_theta) * chassis_ctrl.vmc.right.theta[1];

    chassis_ctrl.body_height_m = 0.5f * (left_height + right_height);
    chassis_ctrl.body_height_rate_mps =
        0.5f * (left_height_rate + right_height_rate);

    const float maximum_body_height = 0.5f
        * LEG_THEORETICAL_MAX_LENGTH_M
        * (fmaxf(left_cos_raw, 0.0f) + fmaxf(right_cos_raw, 0.0f));
    chassis_ctrl.limited_height_target_m = fminf(
        chassis_cmd.target_length,
        maximum_body_height);
    const float height_error =
        chassis_ctrl.limited_height_target_m - chassis_ctrl.body_height_m;
    const float height_natural_frequency =
        6.283185307f * HEIGHT_NATURAL_FREQ_HZ;
    const float height_acceleration_command =
        height_natural_frequency * height_natural_frequency * height_error
        - 2.0f * HEIGHT_DAMPING_RATIO * height_natural_frequency
        * chassis_ctrl.body_height_rate_mps;
    float total_vertical_force = BODY_MASS_KG * GRAVITY_MPS2
        + BODY_MASS_KG * height_acceleration_command;

    const float left_vertical_min = 0.0f;
    const float right_vertical_min = 0.0f;
    const float left_vertical_max = LEG_AXIAL_FORCE_MAX_N * left_cos;
    const float right_vertical_max = LEG_AXIAL_FORCE_MAX_N * right_cos;
    total_vertical_force = clamp_float(
        total_vertical_force,
        left_vertical_min + right_vertical_min,
        left_vertical_max + right_vertical_max);

    const float forward_speed = chassis_ctrl.odometry.outstate.dot_s;
    const float yaw_rate = imu->gyro[2];
    const float lateral_acceleration_raw = clamp_float(
        forward_speed * yaw_rate,
        -CENTRIPETAL_ACCEL_LIMIT_MPS2,
        CENTRIPETAL_ACCEL_LIMIT_MPS2);
    const float filter_omega_dt =
        6.283185307f * CENTRIPETAL_LPF_HZ * safe_dt;
    const float filter_alpha = (filter_omega_dt > 0.0f)
        ? filter_omega_dt / (1.0f + filter_omega_dt)
        : 0.0f;
    chassis_ctrl.lateral_acceleration_mps2 += filter_alpha
        * (lateral_acceleration_raw
           - chassis_ctrl.lateral_acceleration_mps2);

    float roll_lean_target = ROLL_LEAN_SIGN
        * atanf(chassis_ctrl.lateral_acceleration_mps2 / GRAVITY_MPS2);
    roll_lean_target = clamp_float(roll_lean_target,
                                   -ROLL_LEAN_LIMIT_RAD,
                                   ROLL_LEAN_LIMIT_RAD);
    chassis_ctrl.roll_lean_target_rad = roll_lean_target;
    chassis_ctrl.effective_roll_target_rad =
        chassis_cmd.target_roll + chassis_ctrl.roll_lean_target_rad;

    const float roll_angle = imu->roll * DEG2RAD;
    const float roll_error = chassis_ctrl.effective_roll_target_rad
        - roll_angle;
    const float roll_natural_frequency =
        6.283185307f * ROLL_NATURAL_FREQ_HZ;
    const float roll_acceleration_command =
        roll_natural_frequency * roll_natural_frequency * roll_error
        - 2.0f * ROLL_DAMPING_RATIO * roll_natural_frequency
        * imu->gyro[0];
    const float body_com_height = fmaxf(
        WHEEL_RADIUS_M + chassis_ctrl.body_height_m
        + BODY_COM_OFFSET_FROM_HIP_M,
        WHEEL_RADIUS_M);
    const float effective_roll_inertia = BODY_ROLL_INERTIA_KGM2;
    const float roll_lateral_acceleration = ROLL_LEAN_SIGN
        * chassis_ctrl.lateral_acceleration_mps2;
    const float centripetal_moment = BODY_MASS_KG * body_com_height * roll_lateral_acceleration;
    const float centripetal_gravity_moment = BODY_MASS_KG * body_com_height
        * (roll_lateral_acceleration * cosf(roll_angle)
           - GRAVITY_MPS2 * sinf(roll_angle));
    const float roll_moment = effective_roll_inertia * roll_acceleration_command + centripetal_moment;
    float vertical_force_difference = roll_moment / WHEEL_HALF_TRACK_M;
    const float difference_min = fmaxf(
        2.0f * left_vertical_min - total_vertical_force,
        total_vertical_force - 2.0f * right_vertical_max);
    const float difference_max = fminf(
        2.0f * left_vertical_max - total_vertical_force,
        total_vertical_force - 2.0f * right_vertical_min);
    vertical_force_difference = clamp_float(vertical_force_difference,
                                            difference_min,
                                            difference_max);

    const float left_vertical_force =
        0.5f * (total_vertical_force + vertical_force_difference);
    const float right_vertical_force =
        0.5f * (total_vertical_force - vertical_force_difference);
    float left_axial_force = clamp_float(left_vertical_force / left_cos,
                                         0.0f,
                                         LEG_AXIAL_FORCE_MAX_N);
    float right_axial_force = clamp_float(right_vertical_force / right_cos,
                                          0.0f,
                                          LEG_AXIAL_FORCE_MAX_N);

    const float max_force_step = LEG_FORCE_RATE_LIMIT_NPS * safe_dt;
    left_axial_force = rate_limit_float(left_axial_force,
                                        chassis_ctrl.last_left_leg_force_n,
                                        max_force_step);
    right_axial_force = rate_limit_float(right_axial_force,
                                         chassis_ctrl.last_right_leg_force_n,
                                         max_force_step);

    const float applied_left_vertical_force = left_axial_force * left_cos;
    const float applied_right_vertical_force = right_axial_force * right_cos;
    chassis_ctrl.total_vertical_force_n =
        applied_left_vertical_force + applied_right_vertical_force;
    chassis_ctrl.roll_moment_nm = WHEEL_HALF_TRACK_M
        * (applied_left_vertical_force - applied_right_vertical_force);
    chassis_ctrl.left_leg_force_n = left_axial_force;
    chassis_ctrl.right_leg_force_n = right_axial_force;
    chassis_ctrl.last_left_leg_force_n = left_axial_force;
    chassis_ctrl.last_right_leg_force_n = right_axial_force;
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
    Chassis_ResetSupportController();

    float pid_joint_pos[3] = {5.0f, 0.0f, 0.0f};
    float pid_joint_vel[3] = {12.0f, 0.0f, 0.0f};
    for (int i = 0; i < 4; i++) {
        PID_Init(&chassis_ctrl.joint_pos[i], 2.0f, 0.0f,
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
    const float left_wheel_speed_mps = WHEEL_RADIUS_M * left_wheel_radps;
    const float right_wheel_speed_mps = WHEEL_RADIUS_M * right_wheel_radps;
    chassis_ctrl.wheel_speed_mps =
        0.5f * (left_wheel_speed_mps + right_wheel_speed_mps);
    Estimator_Set_WheelSpeeds(&chassis_ctrl.odometry,
                              left_wheel_speed_mps,
                              right_wheel_speed_mps);
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
                        chassis_ctrl.lqr.u[2] = MATH_Limit_float(chassis_ctrl.lqr.u[2], -6, 6);
                        chassis_ctrl.lqr.u[3] = MATH_Limit_float(chassis_ctrl.lqr.u[3], -6, 6);
                        Chassis_UpdateSupportForces(imu, dt);
                        VMC_ForceToTorque(chassis_ctrl.vmc.JRM_l,
                                          chassis_ctrl.left_leg_force_n,
                                          chassis_ctrl.lqr.u[0],
                                          &chassis_ctrl.vmc.left);
                        VMC_ForceToTorque(chassis_ctrl.vmc.JRM_r,
                                          chassis_ctrl.right_leg_force_n,
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
                        if (chassis_ctrl.vmc.left.support_force > 20) {
                            wheel_raw[1] = 0;
                        }else if (chassis_ctrl.vmc.right.support_force > 20) {
                            wheel_raw[0] = 0;
                        }
                        DJI_Motor_Send(&hfdcan1, 0x200,wheel_raw[0],0,wheel_raw[1],0);
                        applied_input[0] = chassis_ctrl.lqr.u[0];
                        applied_input[1] = chassis_ctrl.lqr.u[1];
                        applied_input[2] = chassis_ctrl.lqr.u[2];
                        applied_input[3] = chassis_ctrl.lqr.u[3];
                    }
                    break;
                case CTRL_JUMP:
                    Chassis_ResetSupportController();
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
    // VOFA_JustFloat(&huart1, 7,
    //     chassis_ctrl.lqr.x[0],
    //     chassis_ctrl.lqr.x_ref[0],
    //     chassis_ctrl.lqr.x[5],
    //     chassis_ctrl.lqr.x_ref[5],
    //     imu->pitch,
    //     imu->roll,0.0f);
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
