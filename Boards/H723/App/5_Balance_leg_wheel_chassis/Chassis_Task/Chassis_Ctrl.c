/* 曹康祺于 2026 年 6 月 20 日创建。 */

#include "Chassis_Ctrl.h"

#define WHEEL_RADIUS_M          0.060f
#define WHEEL_GEAR_RATIO        15.76f
#define WHEEL_HALF_TRACK_M      0.20f
#define WHEEL_RAW_PER_NM        (16384.0f / (0.315f * 20.0f))
#define WHEEL_RAW_LIMIT         16000.0f
#define BALANCE_WHEEL_LIMIT_NM  6.0f
#define BM_ENCODING_LIMIT_NM   (32760.0f / (100.0f * 1.1785113f))
#define PREPARE_LEG_LENGTH_M    0.15f
#define STAND_HEIGHT_M          0.16f

/* 倒地自起。确认左右轮电流方向后填 +1 或 -1，再打开开关。 */
#define CHASSIS_SELF_RIGHTING_ENABLE 1
#define RECOVERY_LEFT_WHEEL_SIGN     1
#define RECOVERY_RIGHT_WHEEL_SIGN    -1
#if CHASSIS_SELF_RIGHTING_ENABLE && \
    ((RECOVERY_LEFT_WHEEL_SIGN != 1 && RECOVERY_LEFT_WHEEL_SIGN != -1) || \
     (RECOVERY_RIGHT_WHEEL_SIGN != 1 && RECOVERY_RIGHT_WHEEL_SIGN != -1))
#error "自起轮端方向必须设为 +1 或 -1"
#endif

/* 机身轮廓、虚拟髋和轮偏移使用仿真坐标，原点为双髋中点。 */
enum { RECOVERY_SUPPORT_POINT_COUNT = 509 };
static const float recovery_support_points_m[RECOVERY_SUPPORT_POINT_COUNT][3];

static const SelfRighting_Config_t recovery_config = {
    .support_points_m = recovery_support_points_m,
    .support_count = sizeof(recovery_support_points_m) / sizeof(recovery_support_points_m[0]),
    /* 与支撑点同源，取仿真 recovery_geometry.json 的髋轴和轮偏移。 */
    .hip_origin_m = {
        {0.0f,  0.1605f, 0.0f},   /* 左虚拟髋 X/Y/Z，m */
        {0.0f, -0.1605f, 0.0f},   /* 右虚拟髋 X/Y/Z，m */
    },
    .wheel_lateral_m = {0.04525f, -0.04525f},   /* 左、右轮相对本侧髋平面的 Y 偏移，m */

    /* 沿用当前实车工程填写的连续角范围，顺序 LF/RF/LB/RB。 */
    .hip_min_rad = {-1.0e6f, -1.0e6f, -1.0e6f, -1.0e6f},
    .hip_max_rad = { 1.0e6f,  1.0e6f,  1.0e6f,  1.0e6f},
    .hip_limit_nm = 20.0f,
    .wheel_limit_nm = BALANCE_WHEEL_LIMIT_NM,

    .min_length_m = VMC_LEG_MIN_LENGTH_M,
    .swing_length_m = VMC_LEG_MAX_LENGTH_M,  /* 按实车可用行程修改，当前逆解上限 0.320 m */
    .capture_length_m = 0.150f,
    .wheel_radius_m = WHEEL_RADIUS_M,
    .wheel_effective_mass_kg = 0.353f,      /* 轮载估计初值，实车标定后可改 */
    .timeout_s = 10.0f,
};

Chassis_Control_Mode_t MODE = CTRL_PREPARE;
Chassis_Ctrl_t chassis_ctrl;

/* 正轮侧力矩映射为 DJI 电流。关闭自起时保留原 LQR 的 CAN 映射。 */
static const int8_t wheel_command_sign[2] = {
    CHASSIS_SELF_RIGHTING_ENABLE ? RECOVERY_LEFT_WHEEL_SIGN : 1,
    CHASSIS_SELF_RIGHTING_ENABLE ? RECOVERY_RIGHT_WHEEL_SIGN : 1,
};
/* 反馈依次来自 0x203、0x201，发送时对应 CAN 电流槽位 3、1。 */
static const uint8_t wheel_slot_by_feedback[2] = {1u, 0u};

static const float hip_sign[4] = {1.0f, -1.0f, 1.0f, -1.0f};
static Motor_Power_State_t wheel_power_states[2];
static Power_Motion_Node_t motion_nodes[2];
static Power_Motion_Result_t chassis_power_result;

static float clamp(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
}

static void Chassis_ResetPreparePids(void)
{
    for (unsigned i = 0; i < 4; ++i) {
        PID_Clear(&chassis_ctrl.joint_pos[i]);
        PID_Clear(&chassis_ctrl.joint_vel[i]);
    }
}

static void Chassis_ResetOdometry(void)
{
    Estimator_Leg_Init(&chassis_ctrl.odometry);
    Estimator_Set_WheelHalfTrack(&chassis_ctrl.odometry, WHEEL_HALF_TRACK_M);
}

uint8_t Chassis_Control_Init(void)
{
    VMC_Init(&chassis_ctrl.vmc, VMC_L1_LENGTH, VMC_L2_LENGTH, VMC_L3_LENGTH, VMC_L4_LENGTH);
    Chassis_ResetOdometry();
    LQR_Init(&chassis_ctrl.lqr);
    LESO_Init(&chassis_ctrl.leso);
    Body_Support_Init(&chassis_ctrl.support);
    StepClimb_Reset(&chassis_ctrl.step);
    if (CHASSIS_SELF_RIGHTING_ENABLE) {
        SelfRighting_Init(&chassis_ctrl.recovery, &recovery_config);
    }

    float position_gains[3] = {10.0f, 0.0f, 0.0f};
    float velocity_gains[3] = {12.0f, 0.0f, 0.0f};
    for (unsigned i = 0; i < 4; ++i) {
        PID_Init(&chassis_ctrl.joint_pos[i], 3.5f, 0.0f,
                 position_gains, 0, 0, 0, 0, 0, 0);
        PID_Init(&chassis_ctrl.joint_vel[i], 7.0f, 0.0f,
                 velocity_gains, 0, 0, 0, 0, 0, 0);
    }
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[0], -1.50560224f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[1],  0.489915133f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[2], -2.08909842f);
    BM_save_zeroPoint_User(&leg_motors.BM_P1010B_Leg[3],  0.292033000f);

    chassis_ctrl.last_applied = (SelfRighting_Torque_t){0};
    chassis_ctrl.stand_initialized = false;
    chassis_ctrl.output_enabled = false;
    MODE = CTRL_PREPARE;
    chassis_power_result = (Power_Motion_Result_t){0};
    for (unsigned i = 0; i < 2; ++i) {
        motion_nodes[i] = (Power_Motion_Node_t){0};
        motion_nodes[i].motor.state = &wheel_power_states[i];
        motion_nodes[i].motor.model = &MODEL_M3508;
        motion_nodes[i].max_cmd = WHEEL_RAW_LIMIT;
    }
    System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    return 1U;
}

static void Chassis_Stop(bool operator_safe)
{
    if (operator_safe) {
        if (chassis_ctrl.recovery.configured) {
            SelfRighting_Reset(&chassis_ctrl.recovery);
        }
        MODE = CTRL_PREPARE;
    } else if (chassis_ctrl.output_enabled) {
        MODE = CTRL_FALL_HOLD;
    }
    Chassis_ResetPreparePids();
    Body_Support_Reset(&chassis_ctrl.support);
    VMC_ResetFeedbackHistory(&chassis_ctrl.vmc);
    StepClimb_Reset(&chassis_ctrl.step);
    chassis_ctrl.stand_initialized = false;
    chassis_ctrl.output_enabled = false;
    BM_Send_torque(&hfdcan2, 0x032, 0, 0, 0, 0);
    DJI_Motor_Send(&hfdcan1, 0x200, 0, 0, 0, 0);
    chassis_ctrl.last_applied = (SelfRighting_Torque_t){0};
}

static SelfRighting_Torque_t Chassis_PrepareTorque(const Leg_Motor_Group_t *legs, const IMU_Data_t *imu)
{
    const VMC_Control_t *vmc = &chassis_ctrl.vmc;
    SelfRighting_Torque_t torque = {0};
    /* 上电准备只收轮，不执行摆腿翻身。 */
    if (fabsf(imu->pitch) >= 12.0f || fabsf(imu->roll) >= 12.0f ||
        fabsf(vmc->left.theta[0]) >= 0.95f || fabsf(vmc->right.theta[0]) >= 0.95f) {
        Chassis_ResetPreparePids();
        return torque;
    }

    float q_ref[2];
    if (!VMC_InverseKinematics(vmc, PREPARE_LEG_LENGTH_M, 0.0f,
                              imu->pitch * DEG2RAD, &q_ref[0], &q_ref[1])) {
        MODE = CTRL_FALL_HOLD;
        return torque;
    }
    for (unsigned i = 0; i < 4; ++i) {
        const BM_MOTOR_DATA_Typedef *motor = &legs->BM_P1010B_Leg[i];
        const float q_now = hip_sign[i] * motor->pos_rad;
        const float q = VMC_UnwrapAngleNear(q_ref[i/2U], q_now);
        if (chassis_ctrl.recovery.configured &&
            (q < recovery_config.hip_min_rad[i] || q > recovery_config.hip_max_rad[i])) {
            MODE = CTRL_FALL_HOLD;
            return (SelfRighting_Torque_t){0};
        }
        const float raw_target = motor->pos_single + hip_sign[i]*(q-q_now);
        PID_Calculate(&chassis_ctrl.joint_pos[i], motor->pos_single, raw_target);
        PID_Calculate(&chassis_ctrl.joint_vel[i], motor->vel_rad, chassis_ctrl.joint_pos[i].Output);
        const VMC_AirSpring_t *spring = &vmc->spring[i % 2U];
        float spring_torque_nm;
        if (i < 2U) {
            spring_torque_nm = spring->torque_front_nm;
        } else {
            spring_torque_nm = spring->torque_back_nm;
        }
        torque.hip[i] = hip_sign[i] * chassis_ctrl.joint_vel[i].Output - spring_torque_nm;
    }
    if (fabsf(vmc->left.theta[0]) < 0.20f && fabsf(vmc->right.theta[0]) < 0.20f &&
        vmc->left.length[0] < 0.17f && vmc->right.length[0] < 0.17f) {
        MODE = CTRL_STAND;
        chassis_cmd.target_length = STAND_HEIGHT_M;
    }
    return torque;
}

static float Chassis_CaptureHeight(const float length_ref[2])
{
    return 0.5f * (length_ref[0] * cosf(chassis_ctrl.vmc.left.theta[0]) +
                   length_ref[1] * cosf(chassis_ctrl.vmc.right.theta[0]));
}

static SelfRighting_Torque_t Chassis_BalanceTorque(const IMU_Data_t *imu,
    bool capturing, const float length_ref[2], float dt)
{
    LQR_Control_t *lqr = &chassis_ctrl.lqr;
    float yaw_measured = chassis_cmd.offset_angle;
    float height = chassis_cmd.target_length;
    float roll = chassis_cmd.target_roll;
    if (capturing) {
        /* 捕获期间用 IMU 偏航，测量与参考都在同一个连续坐标系。 */
        yaw_measured = imu->YawTotalAngle * DEG2RAD;
        height = Chassis_CaptureHeight(length_ref);
        roll = 0.0f;
        LQR_SetTarget(lqr, 0.0f, 0.0f, 0.0f, chassis_ctrl.capture_yaw_rad, 0.0f, 0.0f, dt);
    } else if (chassis_ctrl.step.phase == STEP_CAPTURE) {
        yaw_measured = imu->YawTotalAngle * DEG2RAD;
        const float direction = chassis_ctrl.step.direction;
        const float speed = direction * clamp(direction*chassis_cmd.target_vx, 0.0f, 0.90f);
        LQR_SetTarget(lqr, speed, 0.0f, 0.0f, chassis_ctrl.capture_yaw_rad, 0.0f, 0.0f, dt);
    } else if (chassis_cmd.mode == CHASSIS_CMD_SPIN) {
        const float yaw_rate = chassis_cmd.target_vw;
        chassis_ctrl.spin_yaw_rad = yaw_measured + normalize_to_pi(
            chassis_ctrl.spin_yaw_rad + yaw_rate*dt - yaw_measured);
        LQR_SetTarget(lqr, chassis_cmd.target_vx, yaw_rate, 0.0f,
                      chassis_ctrl.spin_yaw_rad, 0.0f, 0.0f, dt);
    } else {
        LQR_SetTarget(lqr, chassis_cmd.target_vx, chassis_cmd.follow_rads,
                      0.0f, 0.0f, 0.0f, 0.0f, dt);
    }
    Body_Support_Update(&chassis_ctrl.support, &chassis_ctrl.vmc, imu, height, roll,
                        chassis_ctrl.odometry.outstate.dot_s, dt);
    LQR_Update(lqr, &chassis_ctrl.vmc, &chassis_ctrl.odometry, imu, &chassis_ctrl.leso,
               false, yaw_measured);
    VMC_CalcTorque(&chassis_ctrl.vmc, chassis_ctrl.support.left_leg_force_n,
                   chassis_ctrl.support.right_leg_force_n, lqr->u[0], lqr->u[1]);

    const VMC_Control_t *vmc = &chassis_ctrl.vmc;
    SelfRighting_Torque_t torque = {
        .hip = {
            vmc->left.Tp_front - vmc->spring[0].torque_front_nm,
            vmc->right.Tp_front - vmc->spring[1].torque_front_nm,
            vmc->left.Tp_back - vmc->spring[0].torque_back_nm,
            vmc->right.Tp_back - vmc->spring[1].torque_back_nm,
        },
        /* 转回原电流坐标时恢复原有 [+u3, -u2] 映射。 */
        .wheel = {wheel_command_sign[0] * lqr->u[3], -wheel_command_sign[1] * lqr->u[2]},
    };
    return torque;
}

static void Chassis_SendTorque(const Chassis_Motor_Group_t *wheels, const Leg_Motor_Group_t *legs,
                               SelfRighting_Torque_t torque)
{
    const SelfRighting_Config_t *cfg = &chassis_ctrl.recovery.cfg;
    const bool calibrated = chassis_ctrl.recovery.configured;
    float hip_limit = BM_ENCODING_LIMIT_NM;
    if (calibrated) {
        hip_limit = fminf(cfg->hip_limit_nm, BM_ENCODING_LIMIT_NM);
    }
    /* 只衰减朝机械限位方向的力矩，允许关节向内退回。 */
    for (unsigned i = 0; i < 4; ++i) {
        float lo = -hip_limit, hi = hip_limit;
        if (calibrated) {
            const float q = hip_sign[i] * legs->BM_P1010B_Leg[i].pos_rad;
            lo *= clamp((q - cfg->hip_min_rad[i])/0.03f, 0.0f, 1.0f);
            hi *= clamp((cfg->hip_max_rad[i] - q)/0.03f, 0.0f, 1.0f);
        }
        torque.hip[i] = clamp(torque.hip[i], lo, hi);
    }
    int16_t raw[2];
    for (unsigned i = 0; i < 2; ++i) {
        raw[i] = (int16_t)(wheel_command_sign[i] *
            clamp(torque.wheel[i], -BALANCE_WHEEL_LIMIT_NM, BALANCE_WHEEL_LIMIT_NM) * WHEEL_RAW_PER_NM);
    }
    /* 只有轮端有输出时才分配功率，发送并记录的是限功率后的电流。 */
    if (raw[0] != 0 || raw[1] != 0) {
        for (unsigned i = 0; i < 2; ++i) {
            wheel_power_states[i].speed_rpm = wheels->DJI_3508_Chassis[i].Speed_now;
            motion_nodes[i].translation_cmd = raw[wheel_slot_by_feedback[i]];
        }
        float base_limit = 5000.0f;
        float buffer;
        if (Referee.offline.is_online) {
            base_limit = Referee.robot_status.chassis_power_limit;
        }
        if (Referee.offline.is_online) {
            buffer = Referee.power_heat_data.buffer_energy;
        } else {
            buffer = Meter.buffer_energy;
        }
        /* 缓冲功率修正，超电容量从 20% 到 23% 时线性增加放电额度。 */
        const float capacity = cap.get.Cap_Capacity;
        const bool boost = cap.get.offline.is_online && cap.get.cap_state == 0 && capacity > 20.0f;
        float extra = -5.0f;
        if (boost) {
            extra = 150.0f * clamp((capacity - 20.0f) / 3.0f, 0.0f, 1.0f);
        }
        const float limit = base_limit + 0.5f*(buffer - 30.0f) + extra;
        const Power_Motion_Status_t status = Power_Ctrl_Allocate_Motion_With_Priority(
            fmaxf(0.0f, limit), motion_nodes, 2, POWER_PRIORITY_TRANSLATION, &chassis_power_result);
        const bool valid = status == POWER_MOTION_OK || status == POWER_MOTION_LIMITED;
        for (unsigned i = 0; i < 2; ++i) {
            const unsigned slot = wheel_slot_by_feedback[i];
            if (valid && isfinite(wheel_power_states[i].limited_cmd)) {
                raw[slot] = (int16_t)clamp(wheel_power_states[i].limited_cmd,
                                           -WHEEL_RAW_LIMIT, WHEEL_RAW_LIMIT);
            } else {
                raw[slot] = 0;
            }
        }
    }

    /* 右髋的电机安装方向与统一腿坐标相反。 */
    BM_Send_torque(&hfdcan2, 0x032, torque.hip[0], -torque.hip[1], torque.hip[2], -torque.hip[3]);
    DJI_Motor_Send(&hfdcan1, 0x200, raw[0], 0, raw[1], 0);
    torque.wheel[0] = wheel_command_sign[0] * raw[0] / WHEEL_RAW_PER_NM;
    torque.wheel[1] = wheel_command_sign[1] * raw[1] / WHEEL_RAW_PER_NM;
    chassis_ctrl.last_applied = torque;
}

void Chassis_Control_Task(const Chassis_Motor_Group_t *wheel_motors,
                          const Leg_Motor_Group_t *leg_feedback, const IMU_Data_t *imu, float dt)
{
    /* 拨回安全挡，停机时倒地锁定。 */
    const bool operator_safe = chassis_cmd.mode == CHASSIS_CMD_SAFE;
    if (wheel_motors == NULL || leg_feedback == NULL || imu == NULL) {
        System_State_Report(ID_CHASSIS, STATUS_ERROR);
        Chassis_Stop(operator_safe);
        return;
    }
    const bool motors_online = Is_Group_Online(CHASSIS);
    const bool period_valid = dt > 0.0f && dt <= 0.02f;
    const bool imu_ready = imu_ctrl_state == FUSION_RUN && imu_ctrl_flag.fusion_enabled &&
                           period_valid && isfinite(imu->roll);
    if (!motors_online) {
        System_State_Report(ID_CHASSIS, STATUS_LOST);
    } else if (imu_ready) {
        System_State_Report(ID_CHASSIS, STATUS_RUN);
    } else {
        System_State_Report(ID_CHASSIS, STATUS_PREPARING);
    }
    if (!motors_online || !imu_ready || operator_safe ||
        sys_state.global_mode != GLOBAL_NORMAL_MATCH || MODE == CTRL_FALL_HOLD) {
        Chassis_Stop(operator_safe);
        return;
    }

    /* 先刷新腿姿态、雅可比和气弹簧力矩，本拍所有控制共用这份反馈。 */
    VMC_Update(&chassis_ctrl.vmc, dt, imu->pitch * DEG2RAD,
               leg_feedback->BM_P1010B_Leg[0].pos_rad, leg_feedback->BM_P1010B_Leg[2].pos_rad,
               leg_feedback->BM_P1010B_Leg[1].pos_rad, leg_feedback->BM_P1010B_Leg[3].pos_rad);
    if (!chassis_ctrl.vmc.derivative_initialized) {
        Chassis_Stop(false);
        return;
    }

    const float wheel_radps[2] = {
        -(float)wheel_motors->DJI_3508_Chassis[0].Speed_now * RPM_TO_RADS / WHEEL_GEAR_RATIO,
         (float)wheel_motors->DJI_3508_Chassis[1].Speed_now * RPM_TO_RADS / WHEEL_GEAR_RATIO,
    };
    /* 先判定倒地自起，再决定本周期是否允许台阶控制。 */
    StepClimb_t *step = &chassis_ctrl.step;
    SelfRighting_Output_t recovery = {0};
    if (chassis_ctrl.recovery.configured) {
        SelfRighting_Input_t input = {
            .vmc = &chassis_ctrl.vmc,
            .wheel_vel_radps = {wheel_radps[0], wheel_radps[1]},
            .applied = chassis_ctrl.last_applied,
            .armed = true,
            .feedback_valid = true,
            /* 主动抬轮时屏蔽腿角倒地判据，机身翻倒判据仍有效。 */
            .step_protected = step->phase >= STEP_LIFT && step->phase <= STEP_PLANT,
        };
        memcpy(input.quat_wxyz, imu->q, sizeof(input.quat_wxyz));
        memcpy(input.gyro_radps, imu->gyro, sizeof(input.gyro_radps));
        memcpy(input.accel_mps2, imu->accel, sizeof(input.accel_mps2));
        for (unsigned i = 0; i < 4; ++i) {
            input.motor_pos_rad[i] = leg_feedback->BM_P1010B_Leg[i].pos_rad;
            input.motor_vel_radps[i] = leg_feedback->BM_P1010B_Leg[i].vel_rad;
        }
        recovery = SelfRighting_Update(&chassis_ctrl.recovery, &input, dt);
    }
    const bool recovery_controls_motion = recovery.active || recovery.use_balance || recovery.released;
    if (MODE != CTRL_STAND || recovery_controls_motion) {
        StepClimb_Reset(step);
    }
    /* 未配置自起时，继续用原来的腿角与机身姿态判定倒地。 */
    bool legacy_fall = false;
    if (!chassis_ctrl.recovery.configured && MODE == CTRL_STAND) {
        const VMC_Control_t *vmc = &chassis_ctrl.vmc;
        const bool legs_outside_balance =
            fabsf(vmc->left.theta[0]) > 3.5f * vmc->left.length[0] ||
            fabsf(vmc->right.theta[0]) > 3.5f * vmc->right.length[0];
        legacy_fall = legs_outside_balance || fabsf(imu->pitch) > 50.0f || fabsf(imu->roll) > 50.0f;
    }
    if (recovery.phase == SR_FAULT || legacy_fall) {
        MODE = CTRL_FALL_HOLD;
        Chassis_Stop(false);
        return;
    }

    SelfRighting_Torque_t torque = {0};
    if (recovery.active && !recovery.use_balance) {
        torque = recovery.direct;
    } else if (recovery.use_balance || MODE == CTRL_STAND) {
        if (recovery.handoff || !chassis_ctrl.stand_initialized) {
            /* 接管平衡时从当前朝向重新建立参考。 */
            Chassis_ResetOdometry();
            LQR_Init(&chassis_ctrl.lqr);
            LESO_Init(&chassis_ctrl.leso);
            Body_Support_Reset(&chassis_ctrl.support);
            chassis_ctrl.capture_yaw_rad = imu->YawTotalAngle * DEG2RAD;
            chassis_ctrl.spin_yaw_rad = chassis_cmd.offset_angle;
            chassis_ctrl.stand_initialized = true;
        }
        /* 台阶模式下，悬空轮速不能代表车身速度。 */
        chassis_ctrl.odometry.skip_wheel_update = step->phase != STEP_IDLE && !step->wheel_odometry_valid;
        /* 轮速扣除摆腿产生的轮心运动后，再用于底盘里程计。 */
        const float left_speed = WHEEL_RADIUS_M * wheel_radps[0];
        const float right_speed = WHEEL_RADIUS_M * wheel_radps[1];
        Estimator_Set_WheelSpeeds(&chassis_ctrl.odometry, left_speed, right_speed);
        const VMC_LegSite_t *legs[2] = {&chassis_ctrl.vmc.left, &chassis_ctrl.vmc.right};
        float hip_velocity[2];
        for (unsigned side = 0; side < 2; ++side) {
            const VMC_LegSite_t *leg = legs[side];
            hip_velocity[side] = WHEEL_RADIUS_M * leg->shank_rate_radps +
                                 leg->length[1] * sinf(leg->theta[0]) +
                                 leg->length[0] * cosf(leg->theta[0]) * leg->theta[1];
        }
        Estimator_Set_LegVelocityCorrection(&chassis_ctrl.odometry, hip_velocity[0], hip_velocity[1]);
        Estimator_Task(&chassis_ctrl.odometry, *imu, dt);
        if (MODE == CTRL_STAND && !recovery_controls_motion) {
            /* 平衡模式下，用本拍里程计和髋力矩判断是否磕上台阶。 */
            const VMC_Control_t *vmc = &chassis_ctrl.vmc;
            StepClimb_Input_t step_input = {
                .vmc = vmc,
                .quat_wxyz = imu->q,
                .gyro_radps = imu->gyro,
                .accel_mps2 = imu->accel,
                .speed_mps = chassis_ctrl.odometry.outstate.dot_s,
                .pitch_rad = imu->pitch * DEG2RAD,
                .roll_rad = imu->roll * DEG2RAD,
                .armed = chassis_cmd.mode == CHASSIS_CMD_FOLLOW && chassis_cmd.target_length >= 0.285f
                      && fminf(vmc->left.length[0], vmc->right.length[0]) >= 0.25f
                      && fabsf(imu->gyro[2]) < 0.65f,
                .feedback_valid = true,
            };
            const uint32_t now = HAL_GetTick();
            for (unsigned i = 0; i < 4; ++i) {
                const BM_MOTOR_DATA_Typedef *motor = &leg_feedback->BM_P1010B_Leg[i];
                /* IQ 是电机返回的力矩，单位 N·m。回报正向与 pos_rad 相反。 */
                step_input.hip_torque_nm[i] = -hip_sign[i] * motor->IQ;
                const uint32_t feedback_age_ms = now - motor->offline.last_feed_tick;
                if (feedback_age_ms > 20u) {
                    step_input.feedback_valid = false;
                }
            }
            const StepClimb_Phase_t previous = step->phase;
            StepClimb_Update(step, &step_input, &recovery_config, dt);
            const bool entered_capture = step->phase == STEP_CAPTURE && previous != STEP_CAPTURE;
            const bool left_step_mode = step->phase == STEP_IDLE && previous != STEP_IDLE;
            if (entered_capture || left_step_mode) {
                /* 从当前腿长和里程重建平衡参考，避免恢复到碰阶前的目标。 */
                const float length[2] = {vmc->left.length[0], vmc->right.length[0]};
                chassis_cmd.target_length = clamp(Chassis_CaptureHeight(length), 0.14f, 0.30f);
                Body_Support_Reset(&chassis_ctrl.support);
                LQR_Init(&chassis_ctrl.lqr);
                LESO_Init(&chassis_ctrl.leso);
                chassis_ctrl.lqr.target.position_m = chassis_ctrl.odometry.outstate.s;
                chassis_ctrl.capture_yaw_rad = imu->YawTotalAngle * DEG2RAD;
                chassis_ctrl.spin_yaw_rad = chassis_cmd.offset_angle;
            }
        }
        /* 抬轮到落轮由台阶轨迹直接给力矩，捕获阶段回到平衡控制。 */
        const bool step_controls_legs = step->phase == STEP_LIFT ||
                                       step->phase == STEP_SWING ||
                                       step->phase == STEP_PLANT;
        if (step_controls_legs) {
            torque = step->torque;
        } else {
            torque = Chassis_BalanceTorque(imu, recovery.phase == SR_RISE || recovery.released,
                                           recovery.length_ref_m, dt);
        }
        if (step->phase != STEP_IDLE) {
            const float hip_delta = 600.0f * dt;
            for (unsigned i = 0; i < 4; ++i) {
                const float prev = chassis_ctrl.last_applied.hip[i];
                torque.hip[i] = clamp(torque.hip[i], prev-hip_delta, prev+hip_delta);
            }
        }
        if (step->phase == STEP_CAPTURE) {
            /* 落轮后渐入轮端力矩，避免切回平衡时骤然牵引。 */
            const float gain = clamp(step->capture_s / 0.06f, 0.0f, 1.0f);
            for (unsigned i = 0; i < 2; ++i) {
                torque.wheel[i] *= gain;
            }
        }
    } else if (MODE == CTRL_PREPARE) {
        torque = Chassis_PrepareTorque(leg_feedback, imu);
    }

    /* 非有限力矩不得进入功率分配或 CAN 编码。 */
    bool torque_valid = MODE != CTRL_FALL_HOLD;
    for (unsigned i = 0; i < 4 && torque_valid; ++i) {
        torque_valid = isfinite(torque.hip[i]);
    }
    if (!torque_valid || !isfinite(torque.wheel[0]) || !isfinite(torque.wheel[1])) {
        MODE = CTRL_FALL_HOLD;
        Chassis_Stop(false);
        return;
    }

    /* 自起交接时先限轮端力矩，再把平衡输出与上一拍实发力矩衔接。 */
    if (recovery.use_balance) {
        for (unsigned i = 0; i < 2; ++i) {
            torque.wheel[i] = clamp(torque.wheel[i], -BALANCE_WHEEL_LIMIT_NM, BALANCE_WHEEL_LIMIT_NM);
        }
        torque = SelfRighting_MixBalance(&chassis_ctrl.recovery, &torque, &chassis_ctrl.last_applied, dt);
    }
    /* 最后一拍仍沿用交接输出，下拍才按正常站立指令计算。 */
    if (recovery.released) {
        MODE = CTRL_STAND;
        chassis_cmd.target_length = clamp(Chassis_CaptureHeight(recovery.length_ref_m), 0.14f, 0.30f);
        chassis_ctrl.spin_yaw_rad = chassis_cmd.offset_angle;
    }
    Chassis_SendTorque(wheel_motors, leg_feedback, torque);
    chassis_ctrl.output_enabled = true;
}

/*
 * 机身模型碰撞轮廓，保留原坐标和点顺序。
 */
static const float recovery_support_points_m[RECOVERY_SUPPORT_POINT_COUNT][3] = {
    {0.25107324074f, -0.0675000060405f, 0.0383362968758f},
    {0.250890224671f, -0.0672071057779f, 0.0390193102785f},
    {0.250935554013f, -0.0673467184651f, 0.0388501966993f},
    {0.250987768144f, -0.0674438865866f, 0.0386553195372f},
    {0.249671935513f, -0.0620857830154f, 0.0435660262117f},
    {0.249673560539f, -0.0621977458129f, 0.0435599447717f},
    {0.249686464711f, -0.0624160678568f, 0.0435118176007f},
    {0.249697566813f, -0.0625196714125f, 0.0434703625529f},
    {0.249728411797f, -0.0627092686492f, 0.0433552862726f},
    {0.249747753289f, 0.062792888069f, 0.0432831071555f},
    {0.249711618187f, 0.0626178238855f, 0.0434179665249f},
    {0.249686464519f, 0.0624160660395f, 0.0435118189455f},
    {0.249673560946f, 0.0621977501902f, 0.0435599451225f},
    {0.251073240858f, 0.0675000014467f, 0.038336295679f},
    {0.251044273632f, 0.067493714073f, 0.0384444427485f},
    {0.251015632859f, 0.0674749321335f, 0.0385512354414f},
    {0.250987767975f, 0.0674438869779f, 0.0386553185994f},
    {0.250960946073f, 0.0674009674763f, 0.0387553941524f},
    {0.250911861758f, 0.0672818294167f, 0.0389385357642f},
    {0.280806035114f, 0.0672071057342f, -0.0726279927065f},
    {0.28073531395f, 0.0674009708915f, -0.0723640779615f},
    {0.281948506404f, 0.0627928924948f, -0.076891792378f},
    {0.282024323173f, 0.0620857905358f, -0.077174697439f},
    {0.282017826124f, 0.0623083084433f, -0.0771504845385f},
    {0.281998694917f, 0.0625196712928f, -0.0770790521728f},
    {0.282024324791f, -0.0620857853252f, -0.0771747037097f},
    {0.281967848042f, -0.0627092713298f, -0.0769639682056f},
    {0.282022685354f, -0.0621977502337f, -0.0771686311972f},
    {0.280623018854f, -0.0675000014524f, -0.071944974268f},
    {0.280651987126f, -0.0674937141547f, -0.0720531287725f},
    {0.28070849181f, -0.0674438870216f, -0.0722640010274f},
    {0.280806034469f, -0.0672071066679f, -0.0726279909371f},
    {0.281948506569f, -0.0627928881507f, -0.0768917934226f},
    {0.280623018972f, 0.0675000060347f, -0.0719449754648f},
    {0.24967193583f, 0.0620857890445f, 0.0435660253272f},
    {0.249747752482f, -0.0627928925385f, 0.0432831097069f},
    {0.281984657947f, -0.0626178165552f, -0.0770266523283f},
    {0.281998694089f, -0.0625196702528f, -0.077079050134f},
    {0.28200981077f, -0.0624160661973f, -0.0771205087578f},
    {0.28201782626f, -0.0623083062014f, -0.0771504847016f},
    {0.280680596139f, -0.0674749320627f, -0.0721599146182f},
    {0.280735313712f, -0.06740096752f, -0.0723640765804f},
    {0.280760705547f, -0.0673467189941f, -0.0724588772543f},
    {0.280784398027f, -0.0672818294604f, -0.0725472181922f},
    {0.282022684789f, 0.0621977458074f, -0.0771686272505f},
    {0.282009810578f, 0.062416067699f, -0.0771205074129f},
    {0.281984656906f, 0.0626178244932f, -0.0770266514071f},
    {0.281967848907f, 0.0627092760556f, -0.0769639685308f},
    {0.280784398541f, 0.067281827004f, -0.0725472196081f},
    {0.280760705772f, 0.0673467184214f, -0.0724588791274f},
    {0.28070849254f, 0.0674438865429f, -0.0722640017222f},
    {0.280680596064f, 0.0674749304197f, -0.0721599150816f},
    {0.280651986618f, 0.0674937161147f, -0.0720531285447f},
    {0.250890225316f, 0.0672071066242f, 0.039019308509f},
    {0.250935555137f, 0.0673467189504f, 0.0388501950694f},
    {0.249728411763f, 0.0627092787363f, 0.0433552857043f},
    {0.24969756577f, 0.0625196701712f, 0.0434703638669f},
    {0.249678433525f, 0.0623083061577f, 0.0435418022735f},
    {0.249678432689f, -0.062308308449f, 0.0435418057064f},
    {0.249711617357f, -0.062617817125f, 0.0434179689566f},
    {0.250911861245f, -0.0672818270477f, 0.0389385371801f},
    {0.250960945835f, -0.0674009709352f, 0.0387553955335f},
    {0.251015633906f, -0.0674749303869f, 0.0385512323089f},
    {0.251044273167f, -0.0674937161584f, 0.0384444461167f},
    {-0.251073241063f, 0.0675000004105f, 0.0383362956879f},
    {-0.250987768177f, 0.0674438828194f, 0.0386553200615f},
    {-0.251015632916f, 0.067474933676f, 0.0385512354032f},
    {-0.249747753694f, 0.0627928903545f, 0.0432831073039f},
    {-0.249678433647f, 0.0623083085257f, 0.0435418022938f},
    {-0.249686464558f, 0.0624160642416f, 0.0435118199355f},
    {-0.249728411967f, 0.062709271368f, 0.0433552850815f},
    {-0.249747752689f, -0.0627928863434f, 0.0432831086201f},
    {-0.249728411643f, -0.0627092759665f, 0.0433552856073f},
    {-0.249697565592f, -0.0625196731084f, 0.0434703641654f},
    {-0.249673560365f, -0.0621977517069f, 0.0435599448409f},
    {-0.250890225296f, -0.0672071061734f, 0.0390193073474f},
    {-0.251073241319f, -0.0674999999026f, 0.0383362944877f},
    {-0.251044273348f, -0.0674937144211f, 0.0384444455126f},
    {-0.251015633352f, -0.0674749280283f, 0.0385512320214f},
    {-0.25098776799f, -0.0674438886114f, 0.0386553190896f},
    {-0.250935554451f, -0.0673467247202f, 0.0388501963909f},
    {-0.280806035032f, -0.0672071070065f, -0.0726279921261f},
    {-0.280784397593f, -0.0672818298786f, -0.0725472183544f},
    {-0.280708492135f, -0.067443888386f, -0.0722640017505f},
    {-0.280651986877f, -0.0674937139463f, -0.0720531276591f},
    {-0.281948506711f, -0.0627928885169f, -0.0768917929205f},
    {-0.282024324769f, -0.0620857873765f, -0.0771747025574f},
    {-0.282009809322f, -0.0624160661165f, -0.077120501856f},
    {-0.281998692968f, -0.0625196696034f, -0.0770790480885f},
    {-0.282024323164f, 0.0620857840716f, -0.0771746981865f},
    {-0.281948506817f, 0.0627928881801f, -0.0768917944801f},
    {-0.281998693897f, 0.062519667495f, -0.0770790499368f},
    {-0.282009809578f, 0.0624160625786f, -0.0771205043214f},
    {-0.282022685564f, 0.0621977461268f, -0.0771686304251f},
    {-0.280623018037f, 0.067500001831f, -0.0719449726698f},
    {-0.280651986982f, 0.0674937163045f, -0.0720531272902f},
    {-0.280680596264f, 0.0674749299748f, -0.0721599143984f},
    {-0.280708491423f, 0.0674438830439f, -0.0722640010221f},
    {-0.280760705861f, 0.0673467191535f, -0.0724588780799f},
    {-0.280784398822f, 0.0672818249219f, -0.0725472210751f},
    {-0.280806035109f, 0.0672071080109f, -0.072627992964f},
    {-0.280623018276f, -0.0675000059321f, -0.0719449737814f},
    {-0.2496719361f, -0.0620857895474f, 0.0435660239319f},
    {-0.249671936376f, 0.0620857855816f, 0.0435660249065f},
    {-0.250890224399f, 0.0672071088891f, 0.039019310105f},
    {-0.281967849644f, 0.0627092703549f, -0.0769639708917f},
    {-0.281984657749f, 0.0626178134576f, -0.0770266539325f},
    {-0.282017827254f, 0.0623083093657f, -0.0771504871672f},
    {-0.280735314234f, 0.0674009719309f, -0.0723640764621f},
    {-0.28202268448f, -0.0621977502293f, -0.0771686274507f},
    {-0.282017827657f, -0.0623083066873f, -0.0771504876669f},
    {-0.281984656852f, -0.0626178235644f, -0.0770266500848f},
    {-0.281967849311f, -0.0627092807046f, -0.0769639703217f},
    {-0.280760706084f, -0.0673467194335f, -0.0724588794715f},
    {-0.280735314439f, -0.067400969979f, -0.0723640785167f},
    {-0.280680597599f, -0.0674749317286f, -0.0721599175367f},
    {-0.250960946096f, -0.0674009700475f, 0.0387553946844f},
    {-0.250911862465f, -0.0672818305335f, 0.0389385357906f},
    {-0.249711617978f, -0.0626178265517f, 0.0434179687929f},
    {-0.249686464368f, -0.0624160682244f, 0.0435118186063f},
    {-0.249678432176f, -0.0623083074832f, 0.0435418051462f},
    {-0.249673560484f, 0.0621977484193f, 0.0435599454177f},
    {-0.249697565555f, 0.0625196677601f, 0.0434703658683f},
    {-0.24971161791f, 0.0626178142403f, 0.0434179684963f},
    {-0.250911861821f, 0.0672818243111f, 0.0389385364219f},
    {-0.25093555522f, 0.0673467212719f, 0.0388501940984f},
    {-0.250960946864f, 0.0674009718174f, 0.0387553931436f},
    {-0.251044273453f, 0.0674937158297f, 0.0384444458815f},
    {0.274117410571f, -0.100000001704f, -0.139134526638f},
    {0.27612504404f, -0.100000001704f, -0.138104722385f},
    {0.277817608179f, -0.100000001704f, -0.136612610303f},
    {0.27954492061f, -0.100000001704f, -0.133714706333f},
    {0.271918087634f, -0.108000003016f, -0.139638721323f},
    {0.274117410571f, -0.108000003016f, -0.139134526638f},
    {0.27515325053f, -0.108000003016f, -0.138682083336f},
    {0.27612504404f, -0.108000003016f, -0.138104722385f},
    {0.278512267995f, -0.108000003016f, -0.135720892396f},
    {0.279091030003f, -0.108000003016f, -0.13474993344f},
    {0.280097841587f, -0.108000003016f, -0.130386680214f},
    {0.280003041582f, -0.108000003016f, -0.129260302489f},
    {0.279400319816f, -0.108000003016f, -0.127085925066f},
    {0.273033439985f, -0.108000003016f, -0.139455050096f},
    {0.277017742372f, -0.108000003016f, -0.137411326361f},
    {0.277817608179f, -0.108000003016f, -0.136612610303f},
    {0.27954492061f, -0.108000003016f, -0.133714706333f},
    {0.279866994503f, -0.108000003016f, -0.132631198204f},
    {0.280052245549f, -0.108000003016f, -0.131516115276f},
    {0.279769330678f, -0.108000003016f, -0.128154367194f},
    {0.27515325053f, -0.100000001704f, -0.138682083336f},
    {0.279866994503f, -0.100000001704f, -0.132631198204f},
    {-0.274117409959f, -0.108000002791f, -0.137134521904f},
    {-0.276125044747f, -0.108000002791f, -0.136104719106f},
    {-0.277017741434f, -0.108000002791f, -0.135411320758f},
    {-0.277817606902f, -0.108000002791f, -0.134612604874f},
    {-0.278512270121f, -0.108000002791f, -0.133720890333f},
    {-0.279544919386f, -0.108000002791f, -0.131714701649f},
    {-0.280097843154f, -0.108000002791f, -0.128386677464f},
    {-0.280003040284f, -0.108000002791f, -0.127260296836f},
    {-0.271918087839f, -0.108000002791f, -0.137638717454f},
    {-0.273033440674f, -0.108000002791f, -0.137455046758f},
    {-0.275153248315f, -0.108000002791f, -0.136682091951f},
    {-0.279091029533f, -0.108000002791f, -0.132749929544f},
    {-0.279866993275f, -0.108000002791f, -0.130631193288f},
    {-0.280052244541f, -0.108000002791f, -0.129516124743f},
    {-0.279769331927f, -0.108000002791f, -0.126154364514f},
    {-0.279400319505f, -0.108000002791f, -0.12508593594f},
    {-0.271918087839f, 0.108000002812f, -0.137638717454f},
    {-0.274117409959f, 0.108000002812f, -0.137134521904f},
    {-0.275153248315f, 0.108000002812f, -0.136682091951f},
    {-0.276125044747f, 0.108000002812f, -0.136104719106f},
    {-0.278512270121f, 0.108000002812f, -0.133720890333f},
    {-0.279091029533f, 0.108000002812f, -0.132749929544f},
    {-0.280097843154f, 0.108000002812f, -0.128386677464f},
    {-0.280003040284f, 0.108000002812f, -0.127260296836f},
    {-0.279400319505f, 0.108000002812f, -0.12508593594f},
    {-0.273033440674f, 0.108000002812f, -0.137455046758f},
    {-0.277017741434f, 0.108000002812f, -0.135411320758f},
    {-0.277817606902f, 0.108000002812f, -0.134612604874f},
    {-0.279544919386f, 0.108000002812f, -0.131714701649f},
    {-0.279866993275f, 0.108000002812f, -0.130631193288f},
    {-0.280052244541f, 0.108000002812f, -0.129516124743f},
    {-0.279769331927f, 0.108000002812f, -0.126154364514f},
    {0.271918087508f, 0.108000002944f, -0.139638721157f},
    {0.276125042735f, 0.108000002944f, -0.138104720753f},
    {0.278512269208f, 0.108000002944f, -0.135720893919f},
    {0.279091031213f, 0.108000002944f, -0.134749935551f},
    {0.279544918618f, 0.108000002944f, -0.133714704968f},
    {0.279866992558f, 0.108000002944f, -0.132631196631f},
    {0.280052245709f, 0.108000002944f, -0.131516115403f},
    {0.280097841806f, 0.108000002944f, -0.130386679498f},
    {0.279769329902f, 0.108000002944f, -0.128154366158f},
    {0.279400318665f, 0.108000002944f, -0.127085923486f},
    {0.273033440385f, 0.100000001633f, -0.139455050416f},
    {0.278512269208f, 0.100000001633f, -0.135720893919f},
    {0.279091031213f, 0.100000001633f, -0.134749935551f},
    {0.280052245709f, 0.100000001633f, -0.131516115403f},
    {0.280097841806f, 0.100000001633f, -0.130386679498f},
    {0.277017743057f, 0.100000001633f, -0.137411326798f},
    {0.273033440385f, 0.108000002944f, -0.139455050416f},
    {0.274117409662f, 0.108000002944f, -0.13913452576f},
    {0.275153248475f, 0.108000002944f, -0.138682081106f},
    {0.277017743057f, 0.108000002944f, -0.137411326798f},
    {0.277817606633f, 0.108000002944f, -0.136612608199f},
    {0.280003041561f, 0.108000002944f, -0.129260302876f},
    {0.118808813903f, -0.0829999969997f, -0.164864094845f},
    {0.118563496013f, -0.0750000030527f, -0.164887056724f},
    {0.118563496173f, -0.08299999694f, -0.164887054537f},
    {0.118808813903f, 0.0750000028215f, -0.164864094845f},
    {0.118808814015f, 0.0829999967085f, -0.164864093316f},
    {0.118563496013f, 0.0829999967685f, -0.164887056724f},
    {0.220460684009f, 0.0763262489293f, 0.0574466438903f},
    {0.220422134407f, 0.0761361376951f, 0.0575905197431f},
    {0.220364152481f, 0.075709216332f, 0.0578069495132f},
    {0.22033427613f, -0.0752410704747f, 0.0579184048231f},
    {0.220345556626f, -0.075478626681f, 0.0578763524717f},
    {0.220364153227f, -0.0757092126041f, 0.0578069467292f},
    {0.220422132912f, -0.076136135288f, 0.0575905253234f},
    {0.220460683158f, -0.0763262495627f, 0.0574466470661f},
    {0.220389798405f, -0.0759294378224f, 0.0577112098627f},
    {0.220389799165f, 0.0759294396668f, 0.057711207028f},
    {-0.0280000016143f, -0.0750000026634f, -0.164585143293f},
    {-0.0299999985299f, -0.0730000007802f, -0.164585143293f},
    {-0.0284786311791f, -0.0749418917114f, -0.164585143293f},
    {-0.0287092078104f, -0.0748700331286f, -0.164585143293f},
    {-0.0289294471301f, -0.0747709134378f, -0.164585143293f},
    {-0.029136128703f, -0.0746459633269f, -0.164585143293f},
    {-0.0296459682131f, -0.0741361294132f, -0.164585143293f},
    {-0.0297709124381f, -0.073929443461f, -0.164585143293f},
    {-0.0298700340714f, -0.0737092137682f, -0.164585143293f},
    {-0.0299418822869f, -0.0734786292628f, -0.164585143293f},
    {-0.0299854179142f, -0.0732410725652f, -0.164585143293f},
    {-0.0299999988161f, 0.0729999972932f, -0.164585143293f},
    {-0.0299854178594f, 0.0732410674805f, -0.164585143293f},
    {-0.0299418822758f, 0.0734786343083f, -0.164585143293f},
    {-0.0298700330063f, 0.0737092109401f, -0.164585143293f},
    {-0.0296459687925f, 0.074136131833f, -0.164585143293f},
    {-0.0294970194065f, 0.0743262475876f, -0.164585143293f},
    {-0.0293262464671f, 0.0744970204391f, -0.164585143293f},
    {-0.0291361292909f, 0.0746459694801f, -0.164585143293f},
    {-0.0287092080579f, 0.0748700372008f, -0.164585143293f},
    {-0.0284786310031f, 0.0749418854167f, -0.164585143293f},
    {-0.0282410736643f, -0.0749854235692f, -0.164585143293f},
    {-0.0293262463207f, -0.074497023254f, -0.164585143293f},
    {-0.029497019172f, -0.0743262465893f, -0.164585143293f},
    {-0.0297709133158f, 0.0739294446718f, -0.164585143293f},
    {-0.0289294470642f, 0.0747709118428f, -0.164585143293f},
    {-0.0282410743055f, 0.0749854210439f, -0.164585143293f},
    {-0.0280000006577f, 0.0750000035222f, -0.164585143293f},
    {0.21750000038f, -0.0724043804956f, 0.0598250776299f},
    {0.21750000038f, 0.0715924190032f, 0.0607864521709f},
    {0.217270909927f, -0.0733337973519f, 0.0598188713386f},
    {0.217370032452f, -0.0731135705555f, 0.0598203427506f},
    {0.217485412143f, -0.0726454386588f, 0.0598234683058f},
    {0.217485412143f, 0.0718334846177f, 0.0607880614283f},
    {0.217441885851f, 0.0720710410785f, 0.0607896485187f},
    {0.217370032452f, 0.0723016090639f, 0.0607911869338f},
    {0.217145963691f, 0.0727285252703f, 0.060794036858f},
    {0.216997026585f, 0.0729186374914f, 0.0607953071645f},
    {0.217270909927f, 0.0725218433099f, 0.060792658512f},
    {0.217145963691f, -0.0735404793123f, 0.0598174929926f},
    {0.217441885851f, -0.0728829951204f, 0.0598218813319f},
    {-0.217999992003f, -0.0750000025975f, 0.0604148507061f},
    {-0.217645970216f, 0.0761361349476f, 0.0604148507061f},
    {-0.217770916452f, 0.0759294409407f, 0.0604148507061f},
    {-0.217870024076f, 0.0757092166794f, 0.0604148507061f},
    {-0.217941877475f, 0.0754786286605f, 0.0604148507061f},
    {-0.217999992003f, 0.075000003363f, 0.0604148507061f},
    {-0.217985418668f, -0.0752410735833f, 0.0604148507061f},
    {-0.217941877475f, -0.0754786278951f, 0.0604148507061f},
    {-0.217770916452f, -0.0759294401752f, 0.0604148507061f},
    {-0.217645970216f, -0.0761361341822f, 0.0604148507061f},
    {-0.217870024076f, -0.075709215914f, 0.0604148507061f},
    {-0.217497018209f, -0.0763262506473f, 0.0604148507061f},
    {-0.217497018209f, 0.0763262514127f, 0.0604148507061f},
    {-0.217985418668f, 0.0752410743487f, 0.0604148507061f},
    {-0.0945724665895f, -0.282377300982f, -0.0221999996548f},
    {-0.0937068324975f, -0.282900601697f, -0.0221999996548f},
    {-0.0935049156612f, -0.282937616088f, -0.0221999996548f},
    {0.0935049107655f, -0.282937616003f, -0.0221999996548f},
    {0.0937068302989f, -0.282900601425f, -0.0221999996548f},
    {0.0939028349477f, -0.282839536157f, -0.0221999996548f},
    {-0.0944273122167f, -0.282522469658f, -0.0221999996548f},
    {-0.0942657062853f, -0.282649068361f, -0.0221999996548f},
    {-0.0940900329152f, -0.282755286513f, -0.0221999996548f},
    {-0.0939028326966f, -0.282839537048f, -0.0221999996548f},
    {0.0940900264043f, -0.282755284361f, -0.0221999996548f},
    {0.0942657075141f, -0.282649071559f, -0.0221999996548f},
    {0.0944273077693f, -0.282522469731f, -0.0221999996548f},
    {0.0945724727201f, -0.282377304182f, -0.0221999996548f},
    {-0.0672071049311f, -0.262840240929f, 0.0604044578564f},
    {-0.0672818296334f, -0.262861878459f, 0.0603236842586f},
    {-0.0673467241628f, -0.262885540355f, 0.0602353485713f},
    {-0.0674009711188f, -0.262910932414f, 0.0601405435671f},
    {-0.0674438852923f, -0.262937754414f, 0.0600404701942f},
    {-0.0674749324073f, -0.262965649322f, 0.0599363829836f},
    {-0.0674937152731f, -0.262994259585f, 0.0598295923342f},
    {-0.0620857875733f, -0.261621951965f, 0.0649511757485f},
    {-0.0623083103402f, -0.261628418669f, 0.0649269539323f},
    {-0.0624160678358f, -0.261636465965f, 0.0648969633445f},
    {-0.062519672689f, -0.261647581785f, 0.0648555174549f},
    {0.0627928879345f, -0.261697739867f, 0.0646682519833f},
    {0.062709278797f, -0.26167839696f, 0.0647404363701f},
    {0.0625196679661f, -0.261647581681f, 0.0648555174281f},
    {0.0623083068511f, -0.26162841903f, 0.0649269523624f},
    {0.0621977468869f, -0.261623560617f, 0.0649450960916f},
    {0.0675000034459f, -0.292573004915f, -0.0505598310929f},
    {0.0672071021554f, -0.292756021557f, -0.0512428440369f},
    {0.0674749299815f, -0.292630613093f, -0.0507747653408f},
    {0.0674937124974f, -0.292602002901f, -0.0506679785147f},
    {0.0627928860924f, -0.293898522941f, -0.0555066422894f},
    {0.0620857854964f, -0.293974309479f, -0.0557895545261f},
    {0.0623083079134f, -0.293967842847f, -0.0557653365332f},
    {0.0624160647101f, -0.293959796593f, -0.0557353533483f},
    {0.0625196699133f, -0.293948680701f, -0.0556939036353f},
    {-0.0620857854422f, -0.293974310283f, -0.0557895558695f},
    {-0.062709270447f, -0.293917864889f, -0.0555788238077f},
    {-0.0626178167365f, -0.293934644304f, -0.0556415042073f},
    {-0.0625196707419f, -0.293948680805f, -0.0556939036086f},
    {-0.0624160657179f, -0.293959796359f, -0.0557353530375f},
    {-0.0623083096269f, -0.293967843456f, -0.0557653385429f},
    {-0.0621977496646f, -0.293972700071f, -0.0557834827593f},
    {-0.0672071055872f, -0.292756020573f, -0.051242840342f},
    {-0.0675000043698f, -0.292573005794f, -0.0505598314779f},
    {-0.0674937156852f, -0.292602002802f, -0.0506679802919f},
    {-0.0674009700307f, -0.292685329323f, -0.0509789269901f},
    {-0.0673467214027f, -0.292710721085f, -0.0510737317375f},
    {-0.0627928914091f, -0.293898523662f, -0.0555066455667f},
    {0.0672071065204f, -0.262840242f, 0.0604044538237f},
    {0.0620857907832f, -0.261621951335f, 0.0649511764163f},
    {-0.062792889218f, -0.261697739616f, 0.0646682522857f},
    {-0.0674749306044f, -0.292630612472f, -0.0507747616963f},
    {-0.0674438839421f, -0.29265850849f, -0.0508788533692f},
    {-0.0672818265839f, -0.292734383702f, -0.0511620722841f},
    {0.0621977454276f, -0.293972700916f, -0.0557834853986f},
    {0.062617823342f, -0.293934643082f, -0.0556415028773f},
    {0.0627092769105f, -0.293917865003f, -0.0555788254192f},
    {0.0672818265078f, -0.292734384099f, -0.0511620742624f},
    {0.0673467213871f, -0.292710722131f, -0.0510737347518f},
    {0.067400968343f, -0.292685330072f, -0.0509789297476f},
    {0.0674438828665f, -0.292658508f, -0.0508788525514f},
    {0.0675000022612f, -0.263023226885f, 0.0597214449038f},
    {0.0674937129095f, -0.262994259684f, 0.0598295941114f},
    {0.0674749322365f, -0.262965649059f, 0.0599363825809f},
    {0.0674438848763f, -0.262937754982f, 0.0600404670945f},
    {0.0674009709639f, -0.26291093325f, 0.0601405404717f},
    {0.0673467223359f, -0.262885541488f, 0.0602353452192f},
    {0.0672818275161f, -0.262861877973f, 0.0603236855222f},
    {0.0626178250875f, -0.261661618444f, 0.0648031170133f},
    {0.0624160666501f, -0.261636465315f, 0.0648969662755f},
    {-0.0621977485542f, -0.261623560743f, 0.0649450951511f},
    {-0.0626178187008f, -0.26166161868f, 0.0648031157775f},
    {-0.0627092726183f, -0.26167839773f, 0.0647404347398f},
    {-0.0675000055555f, -0.263023226865f, 0.0597214442752f},
    {-0.0478948685363f, -0.247097939064f, 0.0661000019589f},
    {-0.0545000001597f, -0.246726140319f, 0.0661000011609f},
    {0.0545000002312f, -0.246726140013f, 0.0661000026177f},
    {-0.0404999983943f, -0.253402829598f, 0.0660946879291f},
    {-0.0544999985161f, -0.253299862345f, 0.066100000446f},
    {-0.0545868208629f, -0.253299862453f, 0.0660923707446f},
    {-0.0545000001275f, -0.253408730338f, 0.0660940321976f},
    {0.0545602703819f, -0.253299862674f, 0.0660963487722f},
    {0.0544999978169f, -0.253299862922f, 0.0660999986609f},
    {0.0544999968198f, -0.253402829836f, 0.0660946865633f},
    {0.0404999998928f, -0.253418564429f, 0.0660929324549f},
    {-0.0939999968677f, -0.291187077775f, -0.0612000002213f},
    {-0.0939999968677f, -0.291187077435f, -0.0506069026119f},
    {-0.0939999968677f, -0.291177004741f, -0.050406526846f},
    {-0.0939999968677f, -0.29114693392f, -0.0502081698919f},
    {-0.0939999968677f, -0.291097134964f, -0.0500138217103f},
    {-0.0939999968677f, -0.291128963011f, -0.0616786291774f},
    {-0.0939999968677f, -0.291172504696f, -0.0614410712201f},
    {0.0920000007568f, -0.291187077104f, -0.0662000030637f},
    {-0.0919999996008f, -0.291187077104f, -0.0662000030637f},
    {-0.0922410705866f, -0.291172503768f, -0.0662000030637f},
    {-0.0927092129173f, -0.291057109176f, -0.0662000030637f},
    {-0.0931361311855f, -0.290833055316f, -0.0662000030637f},
    {-0.0933262476506f, -0.290684103309f, -0.0662000030637f},
    {0.0931361323415f, -0.290833055316f, -0.0662000030637f},
    {0.0929294457851f, -0.290957986652f, -0.0662000030637f},
    {0.0927092140733f, -0.291057109176f, -0.0662000030637f},
    {0.0924786335049f, -0.291128962575f, -0.0662000030637f},
    {0.0922410717426f, -0.291172503768f, -0.0662000030637f},
    {-0.0924786323489f, -0.291128962575f, -0.0662000030637f},
    {-0.0929294446291f, -0.290957986652f, -0.0662000030637f},
    {0.0933262488065f, -0.290684103309f, -0.0662000030637f},
    {0.093999996695f, -0.291187077435f, -0.0506069026119f},
    {0.093999996695f, -0.291097134964f, -0.0500138217103f},
    {0.093999996695f, -0.29114693392f, -0.0502081698919f},
    {0.093999996695f, -0.291187077775f, -0.0612000002213f},
    {0.093999996695f, -0.291172504696f, -0.0614410712201f},
    {0.093999996695f, -0.291128963011f, -0.0616786291774f},
    {0.093999996695f, -0.291177004741f, -0.050406526846f},
    {0.0944273134788f, 0.282522470047f, -0.0221999996359f},
    {0.0942657062383f, 0.282649069946f, -0.0221999996359f},
    {0.0940900332202f, 0.282755285837f, -0.0221999996359f},
    {0.0939028345662f, 0.282839537383f, -0.0221999996359f},
    {0.0935049155613f, 0.282937617671f, -0.0221999996359f},
    {-0.0935049128906f, 0.282937617056f, -0.0221999996359f},
    {-0.0937068280453f, 0.282900600211f, -0.0221999996359f},
    {-0.093902835143f, 0.282839538448f, -0.0221999996359f},
    {-0.0940900281842f, 0.282755283928f, -0.0221999996359f},
    {-0.0944273086263f, 0.28252246821f, -0.0221999996359f},
    {0.0945724713373f, 0.28237730112f, -0.0221999996359f},
    {0.0937068293466f, 0.282900601818f, -0.0221999996359f},
    {-0.0942657096636f, 0.282649068857f, -0.0221999996359f},
    {-0.0945724701051f, 0.282377302898f, -0.0221999996359f},
    {0.0675000040292f, 0.263023227856f, 0.0597214435133f},
    {0.0672071073656f, 0.262840241207f, 0.0604044580221f},
    {0.0674009670781f, 0.262910932306f, 0.0601405432398f},
    {0.0674438874878f, 0.262937754202f, 0.0600404698562f},
    {0.0674749312816f, 0.262965649604f, 0.0599363820326f},
    {0.0627928885237f, 0.261697739779f, 0.0646682522318f},
    {0.0620857843602f, 0.261621952099f, 0.0649511730158f},
    {0.0623083071286f, 0.261628419432f, 0.0649269515077f},
    {0.0624160636749f, 0.261636465911f, 0.0648969617617f},
    {0.0625196678755f, 0.261647581945f, 0.0648555170343f},
    {-0.0620857917223f, 0.261621951397f, 0.0649511753386f},
    {-0.0627928871302f, 0.261697739793f, 0.0646682526762f},
    {-0.0625196661184f, 0.261647582279f, 0.0648555159267f},
    {-0.0624160685366f, 0.261636465763f, 0.0648969642533f},
    {-0.0623083041659f, 0.261628419765f, 0.0649269511241f},
    {-0.0621977489913f, 0.261623561454f, 0.0649450936882f},
    {-0.0672071068034f, 0.262840241529f, 0.0604044566736f},
    {-0.0674937110869f, 0.262994259253f, 0.0598295936587f},
    {-0.0674749315483f, 0.262965648914f, 0.0599363820661f},
    {-0.0674438848652f, 0.262937754801f, 0.0600404699438f},
    {-0.0674009679739f, 0.262910932155f, 0.0601405420669f},
    {-0.0673467186208f, 0.262885540848f, 0.0602353480345f},
    {-0.0672818302607f, 0.262861877916f, 0.0603236841883f},
    {-0.0672071078626f, 0.292756020625f, -0.0512428421816f},
    {-0.0672818248581f, 0.292734384675f, -0.051162075772f},
    {-0.0673467206428f, 0.292710721006f, -0.0510737327156f},
    {-0.067400967575f, 0.292685329526f, -0.0509789273994f},
    {-0.0674749317785f, 0.292630613127f, -0.0507747659486f},
    {-0.0674937135495f, 0.292602002765f, -0.0506679803661f},
    {-0.0620857885359f, 0.293974309647f, -0.0557895532918f},
    {-0.062197749821f, 0.293972701036f, -0.0557834836575f},
    {-0.0623083076256f, 0.293967842401f, -0.0557653356672f},
    {-0.062416064218f, 0.293959795097f, -0.0557353500037f},
    {-0.0625196684187f, 0.293948680861f, -0.0556939047893f},
    {0.0620857837753f, 0.29397431131f, -0.0557895591654f},
    {0.0627092726125f, 0.293917864436f, -0.0555788224826f},
    {0.0626178156131f, 0.293934644164f, -0.0556415039962f},
    {0.0625196730252f, 0.293948680552f, -0.0556939037711f},
    {0.0624160679934f, 0.293959796145f, -0.0557353522518f},
    {0.062308311119f, 0.293967842092f, -0.055765335373f},
    {0.0621977484018f, 0.293972701427f, -0.0557834852822f},
    {0.06749371804f, 0.292602002604f, -0.0506679779076f},
    {0.0674749310976f, 0.292630611945f, -0.0507747626301f},
    {0.0674009674307f, 0.292685330651f, -0.050978929822f},
    {0.0673467255739f, 0.292710721009f, -0.0510737322834f},
    {0.0627928865407f, 0.293898523088f, -0.0555066442702f},
    {-0.0675000044793f, 0.292573005136f, -0.0505598316248f},
    {-0.0675000050673f, 0.263023226848f, 0.0597214443933f},
    {0.0675000046635f, 0.292573004271f, -0.0505598291529f},
    {0.0672071062601f, 0.292756021278f, -0.0512428444286f},
    {0.0674438843683f, 0.29265850793f, -0.0508788538599f},
    {0.0672818296713f, 0.292734384966f, -0.0511620757822f},
    {-0.0627928891132f, 0.293898523103f, -0.0555066438258f},
    {-0.0626178227712f, 0.29393464358f, -0.0556415026888f},
    {-0.0627092804659f, 0.293917863835f, -0.055578821047f},
    {-0.0674438879847f, 0.29265850853f, -0.0508788537723f},
    {-0.0627092806057f, 0.261678397447f, 0.0647404345734f},
    {-0.0626178236063f, 0.261661618619f, 0.0648031163305f},
    {0.0621977492778f, 0.261623560872f, 0.064945095659f},
    {0.062617814778f, 0.261661619203f, 0.0648031150231f},
    {0.0627092724727f, 0.261678398048f, 0.0647404331378f},
    {0.0672818242686f, 0.262861878207f, 0.0603236841781f},
    {0.0673467201459f, 0.262885540826f, 0.0602353485561f},
    {0.0674937130063f, 0.262994260042f, 0.0598295926111f},
    {0.047894867185f, 0.247097938605f, 0.0661000036341f},
    {-0.0544999981449f, 0.246726139612f, 0.0661000034019f},
    {-0.0478948694157f, 0.247097938788f, 0.0661000039057f},
    {0.0405000006366f, 0.253402828534f, 0.0660946910022f},
    {0.0544999971967f, 0.253299862091f, 0.066100001723f},
    {0.0544999983132f, 0.253408730621f, 0.0660940313074f},
    {0.0545868197002f, 0.253299862946f, 0.0660923689667f},
    {-0.0544999976913f, 0.253299862748f, 0.0660999990975f},
    {-0.0545602691077f, 0.253299861871f, 0.0660963513854f},
    {-0.0544999991408f, 0.253402828501f, 0.0660946918624f},
    {-0.0404999983984f, 0.253418565349f, 0.0660929285888f},
    {0.0939999967436f, 0.29118707702f, -0.0506069025193f},
    {0.0939999967436f, 0.291146934683f, -0.0502081701446f},
    {0.0939999967436f, 0.291097134854f, -0.0500138216409f},
    {0.0939999967436f, 0.291187078305f, -0.0612000009419f},
    {0.0939999967436f, 0.291172504473f, -0.0614410710219f},
    {0.0939999967436f, 0.291128962401f, -0.0616786295067f},
    {0.0939999967436f, 0.291177005191f, -0.0504065269731f},
    {0.0922410736223f, 0.291172504248f, -0.0662000030029f},
    {0.0924786361283f, 0.291128963387f, -0.0662000030029f},
    {0.0927092104733f, 0.2910571102f, -0.0662000030029f},
    {0.0929294438781f, 0.290957987712f, -0.0662000030029f},
    {0.0931361325681f, 0.290833056181f, -0.0662000030029f},
    {0.0933262515772f, 0.290684103696f, -0.0662000030029f},
    {-0.0933262449071f, 0.29068410471f, -0.0662000030029f},
    {-0.0931361309859f, 0.290833056239f, -0.0662000030029f},
    {-0.0929294465633f, 0.290957987379f, -0.0662000030029f},
    {-0.0924786297525f, 0.291128963552f, -0.0662000030029f},
    {-0.0919999979969f, 0.291187076942f, -0.0662000030029f},
    {0.0920000023876f, 0.291187077192f, -0.0662000030029f},
    {-0.0927092165443f, 0.291057109939f, -0.0662000030029f},
    {-0.0922410687338f, 0.291172505077f, -0.0662000030029f},
    {-0.0939999967226f, 0.291187077985f, -0.0506069027327f},
    {-0.0939999967226f, 0.291097134461f, -0.0500138216078f},
    {-0.0939999967226f, 0.291146933514f, -0.0502081698354f},
    {-0.0939999967226f, 0.291177004923f, -0.0504065269088f},
    {-0.0939999967226f, 0.291187077303f, -0.0612000006815f},
    {-0.0939999967226f, 0.291172505177f, -0.0614410714218f},
    {-0.0939999967226f, 0.291128962549f, -0.0616786287772f},
};
