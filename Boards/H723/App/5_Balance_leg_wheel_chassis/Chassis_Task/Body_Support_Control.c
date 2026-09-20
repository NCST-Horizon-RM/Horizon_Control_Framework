#include "Body_Support_Control.h"

#include "All_define.h"

#include <math.h>
#include <string.h>

/* 车体和轮距参数，用于重力前馈、向心力矩和左右支撑力分配。 */
#define SUPPORT_WHEEL_RADIUS_M                 0.060f /**< 车轮半径，单位 m。 */
#define SUPPORT_WHEEL_HALF_TRACK_M             0.20f  /**< 左右轮中心到车体中心距离，单位 m。 */
#define SUPPORT_BODY_MASS_KG                  17.5f   /**< 参与高度控制的等效车体质量，单位 kg。 */
#define SUPPORT_GRAVITY_MPS2                   9.81f  /**< 重力加速度，单位 m/s²。 */

/* 高度和 ROLL 二阶闭环目标参数。 */
#define SUPPORT_HEIGHT_NATURAL_FREQ_HZ         3.0f   /**< 机身高度闭环自然频率，单位 Hz。 */
#define SUPPORT_HEIGHT_DAMPING_RATIO           0.75f   /**< 机身高度闭环阻尼比。 */
#define SUPPORT_ROLL_NATURAL_FREQ_HZ           4.0f   /**< ROLL 闭环自然频率，单位 Hz。 */
#define SUPPORT_ROLL_DAMPING_RATIO             0.4f   /**< ROLL 闭环阻尼比。 */
#define SUPPORT_BODY_ROLL_INERTIA_KGM2         0.36f  /**< 机身绕质心 ROLL 轴转动惯量，单位 kg·m²。 */
#define SUPPORT_BODY_COM_OFFSET_FROM_HIP_M     0.0f   /**< 质心相对髋部的竖直偏移，向上为正，单位 m。 */

/* 单腿输出和机构安全约束。 */
#define SUPPORT_LEG_AXIAL_FORCE_MAX_N        2500.0f   /**< 单腿最大轴向力，单位 N。 */
#define SUPPORT_LEG_FORCE_RATE_LIMIT_NPS    2500.0f   /**< 单腿轴向力最大变化率，单位 N/s。 */
#define SUPPORT_LEG_VERTICAL_COS_MIN           0.35f  /**< 轴向力换算竖直力时的最小余弦。 */
#define SUPPORT_LEG_THEORETICAL_MAX_LENGTH_M   0.33f  /**< 虚拟腿理论最大长度，单位 m。 */

/* 转弯向心加速度滤波和主动内倾约束。 */
#define SUPPORT_CENTRIPETAL_LPF_HZ             4.0f   /**< 向心加速度低通截止频率，单位 Hz。 */
#define SUPPORT_CENTRIPETAL_ACCEL_LIMIT_MPS2   5.0f   /**< 参与主动内倾的最大向心加速度，单位 m/s²。 */
#define SUPPORT_ROLL_LEAN_SIGN                -1.0f   /**< 主动内倾方向与 IMU ROLL 正方向的映射。 */
#define SUPPORT_ROLL_LEAN_LIMIT_RAD           (9.0f * DEG2RAD) /**< 主动内倾角最大幅值。 */

/** @brief 将浮点数限制在指定闭区间。 */
static float support_clamp(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
}

/** @brief 限制单个控制周期内输出量的最大变化幅值。 */
static float support_rate_limit(float target, float previous, float max_step)
{
    return previous + support_clamp(target - previous, -max_step, max_step);
}

void Body_Support_Init(Body_Support_Control_t *support)
{
    Body_Support_Reset(support);
}

void Body_Support_Reset(Body_Support_Control_t *support)
{
    if (support == NULL) {
        return;
    }
    memset(support, 0, sizeof(*support));
}

void Body_Support_Update(Body_Support_Control_t *support,
                         const VMC_Control_t *vmc,
                         const IMU_Data_t *imu,
                         float target_height_m,
                         float target_roll_rad,
                         float forward_speed_mps,
                         float dt)
{
    if (support == NULL || vmc == NULL || imu == NULL) {
        return;
    }

    /* 1. 限制控制周期，避免异常 dt 放大微分和斜率限制结果。 */
    const float safe_dt = support_clamp(dt, 0.0f, 0.01f);

    /* 2. 根据左右虚拟腿长度和角度计算机身平均竖直高度。 */
    const float left_theta = vmc->left.theta[0];
    const float right_theta = vmc->right.theta[0];
    const float left_length = vmc->left.length[0];
    const float right_length = vmc->right.length[0];
    const float left_cos_raw = cosf(left_theta);
    const float right_cos_raw = cosf(right_theta);
    const float left_cos = fmaxf(left_cos_raw,
                                 SUPPORT_LEG_VERTICAL_COS_MIN);
    const float right_cos = fmaxf(right_cos_raw,
                                  SUPPORT_LEG_VERTICAL_COS_MIN);
    const float left_height = left_length * left_cos_raw;
    const float right_height = right_length * right_cos_raw;
    const float left_height_rate =
        vmc->left.length[1] * left_cos_raw
        - left_length * sinf(left_theta) * vmc->left.theta[1];
    const float right_height_rate =
        vmc->right.length[1] * right_cos_raw
        - right_length * sinf(right_theta) * vmc->right.theta[1];
    support->body_height_m = 0.5f * (left_height + right_height);
    support->body_height_rate_mps =
        0.5f * (left_height_rate + right_height_rate);

    /* 3. 按当前腿角约束机身高度目标，并由平均高度误差生成总竖直力。 */
    const float maximum_body_height = 0.5f
        * SUPPORT_LEG_THEORETICAL_MAX_LENGTH_M
        * (fmaxf(left_cos_raw, 0.0f) + fmaxf(right_cos_raw, 0.0f));
    support->limited_height_target_m = fminf(target_height_m,
                                              maximum_body_height);
    const float height_error =
        support->limited_height_target_m - support->body_height_m;
    const float height_natural_frequency =
        6.283185307f * SUPPORT_HEIGHT_NATURAL_FREQ_HZ;
    const float height_acceleration_command =
        height_natural_frequency * height_natural_frequency * height_error
        - 2.0f * SUPPORT_HEIGHT_DAMPING_RATIO * height_natural_frequency
            * support->body_height_rate_mps;
    float total_vertical_force = SUPPORT_BODY_MASS_KG * SUPPORT_GRAVITY_MPS2
        + SUPPORT_BODY_MASS_KG * height_acceleration_command;

    const float left_vertical_min = 0.0f;
    const float right_vertical_min = 0.0f;
    const float left_vertical_max =
        SUPPORT_LEG_AXIAL_FORCE_MAX_N * left_cos;
    const float right_vertical_max =
        SUPPORT_LEG_AXIAL_FORCE_MAX_N * right_cos;
    total_vertical_force = support_clamp(
        total_vertical_force,
        left_vertical_min + right_vertical_min,
        left_vertical_max + right_vertical_max);

    /* 4. 根据前进速度和偏航角速度估计向心加速度，并低通滤波。 */
    const float lateral_acceleration_raw = support_clamp(
        forward_speed_mps * imu->gyro[2],
        -SUPPORT_CENTRIPETAL_ACCEL_LIMIT_MPS2,
        SUPPORT_CENTRIPETAL_ACCEL_LIMIT_MPS2);
    const float filter_omega_dt =
        6.283185307f * SUPPORT_CENTRIPETAL_LPF_HZ * safe_dt;
    const float filter_alpha = (filter_omega_dt > 0.0f)
        ? filter_omega_dt / (1.0f + filter_omega_dt)
        : 0.0f;
    support->lateral_acceleration_mps2 += filter_alpha
        * (lateral_acceleration_raw - support->lateral_acceleration_mps2);

    /* 5. 生成主动内倾角，并与用户给定的 ROLL 目标叠加。 */
    support->roll_lean_target_rad = support_clamp(
        SUPPORT_ROLL_LEAN_SIGN
            * atanf(support->lateral_acceleration_mps2
                    / SUPPORT_GRAVITY_MPS2),
        -SUPPORT_ROLL_LEAN_LIMIT_RAD,
        SUPPORT_ROLL_LEAN_LIMIT_RAD);
    support->effective_roll_target_rad =
        target_roll_rad + support->roll_lean_target_rad;

    /* 6. 将 ROLL 误差转换为左右腿所需的竖直力差。 */
    const float roll_angle = imu->roll * DEG2RAD;
    const float roll_natural_frequency =
        6.283185307f * SUPPORT_ROLL_NATURAL_FREQ_HZ;
    const float roll_acceleration_command =
        roll_natural_frequency * roll_natural_frequency
            * (support->effective_roll_target_rad - roll_angle)
        - 2.0f * SUPPORT_ROLL_DAMPING_RATIO * roll_natural_frequency
            * imu->gyro[0];
    const float body_com_height = fmaxf(
        SUPPORT_WHEEL_RADIUS_M + support->body_height_m
            + SUPPORT_BODY_COM_OFFSET_FROM_HIP_M,
        SUPPORT_WHEEL_RADIUS_M);
    const float roll_lateral_acceleration = SUPPORT_ROLL_LEAN_SIGN
        * support->lateral_acceleration_mps2;
    const float desired_roll_moment =
        SUPPORT_BODY_ROLL_INERTIA_KGM2 * roll_acceleration_command
        + SUPPORT_BODY_MASS_KG * body_com_height
            * roll_lateral_acceleration;
    float vertical_force_difference =
        desired_roll_moment / SUPPORT_WHEEL_HALF_TRACK_M;

    /* 7. 在两腿可输出范围内，将总竖直力按 ROLL 力矩要求进行分配。 */
    const float difference_min = fmaxf(
        2.0f * left_vertical_min - total_vertical_force,
        total_vertical_force - 2.0f * right_vertical_max);
    const float difference_max = fminf(
        2.0f * left_vertical_max - total_vertical_force,
        total_vertical_force - 2.0f * right_vertical_min);
    vertical_force_difference = support_clamp(vertical_force_difference,
                                               difference_min,
                                               difference_max);
    const float left_vertical_force =
        0.5f * (total_vertical_force + vertical_force_difference);
    const float right_vertical_force =
        0.5f * (total_vertical_force - vertical_force_difference);

    /* 8. 将左右竖直力换算成腿轴向力，并施加幅值和变化率限制。 */
    float left_axial_force = support_clamp(
        left_vertical_force / left_cos,
        0.0f, SUPPORT_LEG_AXIAL_FORCE_MAX_N);
    float right_axial_force = support_clamp(
        right_vertical_force / right_cos,
        0.0f, SUPPORT_LEG_AXIAL_FORCE_MAX_N);
    const float maximum_force_step =
        SUPPORT_LEG_FORCE_RATE_LIMIT_NPS * safe_dt;
    left_axial_force = support_rate_limit(
        left_axial_force,
        support->last_left_leg_force_n,
        maximum_force_step);
    right_axial_force = support_rate_limit(
        right_axial_force,
        support->last_right_leg_force_n,
        maximum_force_step);

    /* 9. 保存最终输出及其实际产生的总竖直力和 ROLL 力矩。 */
    const float applied_left_vertical_force = left_axial_force * left_cos;
    const float applied_right_vertical_force = right_axial_force * right_cos;
    support->total_vertical_force_n =
        applied_left_vertical_force + applied_right_vertical_force;
    support->roll_moment_nm = SUPPORT_WHEEL_HALF_TRACK_M
        * (applied_left_vertical_force - applied_right_vertical_force);
    support->left_leg_force_n = left_axial_force;
    support->right_leg_force_n = right_axial_force;
    support->last_left_leg_force_n = left_axial_force;
    support->last_right_leg_force_n = right_axial_force;
}
