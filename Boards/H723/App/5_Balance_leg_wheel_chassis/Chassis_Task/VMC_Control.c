#include "VMC_Control.h"
#include <math.h>
#include <string.h>

#define VMC_TWO_PI 6.2831853071795864769f
#define VMC_ANGLE_EQUAL_TOLERANCE_RAD 1.0e-6f

float VMC_WrapAngle(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

float VMC_UnwrapAngleNear(float angle, float reference)
{
    return reference + VMC_WrapAngle(angle - reference);
}

float VMC_SelectAngleTarget(float start_unwrapped, float target_theta,
                            VMC_Angle_Path_t direction)
{
    if (!isfinite(start_unwrapped) || !isfinite(target_theta) ||
        (direction != VMC_ANGLE_PATH_SHORTEST &&
         direction != VMC_ANGLE_PATH_POSITIVE &&
         direction != VMC_ANGLE_PATH_NEGATIVE)) {
        return NAN;
    }

    float delta = VMC_WrapAngle(target_theta - start_unwrapped);
    if (fabsf(delta) <= VMC_ANGLE_EQUAL_TOLERANCE_RAD) {
        return start_unwrapped;
    }
    if (direction == VMC_ANGLE_PATH_POSITIVE && delta < 0.0f) {
        delta += VMC_TWO_PI;
    } else if (direction == VMC_ANGLE_PATH_NEGATIVE && delta > 0.0f) {
        delta -= VMC_TWO_PI;
    }
    return start_unwrapped + delta;
}

static float diff(float now, float last, float dt)
{
    return (dt > 1.0e-6f) ? (now - last) / dt : 0.0f;
}

static void update_leg(VMC_Control_t *vmc, float JRM[2][2],
                       VMC_LegSite_t *leg, float front, float back,
                       float body_pitch_rad)
{
    const float xb = vmc->l1 * cosf(back);
    const float yb = vmc->l1 * sinf(back);
    const float xd = vmc->l4 * cosf(front);
    const float yd = vmc->l4 * sinf(front);
    const float dx = xd - xb;
    const float dy = yd - yb;
    const float bd = sqrtf(dx * dx + dy * dy);
    const float a = 2.0f * vmc->l2 * dx;
    const float b = 2.0f * vmc->l2 * dy;
    const float c = vmc->l2 * vmc->l2 + bd * bd - vmc->l3 * vmc->l3;
    const float disc = fmaxf(a * a + b * b - c * c, 0.0f);
    // 与原半角公式选择同一交点，避免分子、分母同时接近零时损失精度。
    const float phi3 = atan2f(b, a) + atan2f(sqrtf(disc), c);
    const float xc = xb + vmc->l2 * cosf(phi3);
    const float yc = yb + vmc->l2 * sinf(phi3);
    const float phi2 = atan2f(yc - yd, xc - xd);
    const float theta_body = -atan2f(xc, yc);
    const float theta = VMC_WrapAngle(theta_body + body_pitch_rad);
    const float length = sqrtf(xc * xc + yc * yc);
    const float s23 = sinf(phi2 - phi3);
    const float safe_s23 = (fabsf(s23) > 1.0e-5f) ? s23 : copysignf(1.0e-5f, s23);

    leg->length_last[0] = leg->length[0];
    leg->theta_last[0] = leg->theta[0];
    leg->length[0] = length;
    leg->theta[0] = theta;
    leg->phi = atan2f(yc, xc);

    const float phi3_world = phi3 + body_pitch_rad;
    const float phi2_world = phi2 + body_pitch_rad;
    JRM[0][0] = vmc->l4 * cosf(theta - phi3_world) * sinf(phi2 - front) / safe_s23;
    JRM[0][1] = -vmc->l4 * sinf(theta - phi3_world) * sinf(phi2 - front) /
                (safe_s23 * fmaxf(length, 1.0e-5f));
    JRM[1][0] = vmc->l1 * cosf(theta - phi2_world) * sinf(back - phi3) / safe_s23;
    JRM[1][1] = -vmc->l1 * sinf(theta - phi2_world) * sinf(back - phi3) /
                (safe_s23 * fmaxf(length, 1.0e-5f));
}

static void update_springs(VMC_Control_t *vmc)
{
    for (int side = 0; side < 2; ++side) {
        const VMC_LegSite_t *leg = side ? &vmc->right : &vmc->left;
        VMC_AirSpring_t *spring = &vmc->spring[side];
        const float (*JRM)[2] = side ? vmc->JRM_r : vmc->JRM_l;
        spring->torque_front_nm = spring->torque_back_nm = 0.0f;

        const float L = leg->length[0];
        float angle = acosf(fminf(fmaxf(
            (vmc->l1 * vmc->l1 + vmc->l2 * vmc->l2 - L * L)
            / (2.0f * vmc->l1 * vmc->l2), -1.0f), 1.0f));
        float gain = L / (vmc->l1 * vmc->l2 * sinf(angle));
        angle -= spring->angle_offset_rad;
        spring->length_m = hypotf(spring->s3 - spring->s2 * cosf(angle), spring->s2 * sinf(angle));
        spring->force_n = spring->force_ref_n + spring->k * (spring->length_ref_m - spring->length_m);
        gain *= spring->force_n * spring->s2 * spring->s3 * sinf(angle) / spring->length_m;
        if (!isfinite(gain)) {
            spring->length_m = spring->force_n = 0.0f;
            continue;
        }
        spring->torque_front_nm = gain * JRM[0][0];
        spring->torque_back_nm = gain * JRM[1][0];
    }
}

void VMC_Init(VMC_Control_t *vmc, float l1, float l2, float l3, float l4)
{
    if (vmc == NULL) {
        return;
    }
    memset(vmc, 0, sizeof(*vmc));
    vmc->l1 = l1; vmc->l2 = l2; vmc->l3 = l3; vmc->l4 = l4;
    vmc->spring[0] = (VMC_AirSpring_t) {
        .s2 = 0.05750f,
        .s3 = 0.18623f,
        .angle_offset_rad = 12.49f * (VMC_TWO_PI / 360.0f),
        .length_ref_m = 0.19f,
        .force_ref_n = 300.0f,
        .k = 50.0f / 0.06f,
    };
    vmc->spring[1] = vmc->spring[0];
}

void VMC_ResetFeedbackHistory(VMC_Control_t *vmc)
{
    if (vmc == NULL) {
        return;
    }
    vmc->derivative_initialized = false;
    vmc->left.length[1] = 0.0f;
    vmc->right.length[1] = 0.0f;
    vmc->left.theta[1] = 0.0f;
    vmc->right.theta[1] = 0.0f;
}

void VMC_Update(VMC_Control_t *vmc, float dt, float body_pitch_rad,
                float pos_front_L, float pos_back_L,
                float pos_front_R, float pos_back_R)
{
    if (vmc == NULL) {
        return;
    }
    if (!isfinite(body_pitch_rad) || !isfinite(pos_front_L) ||
        !isfinite(pos_back_L) || !isfinite(pos_front_R) ||
        !isfinite(pos_back_R)) {
        VMC_ResetFeedbackHistory(vmc);
        return;
    }

    // 正运动学解算腿长腿角等数据
    update_leg(vmc, vmc->JRM_l, &vmc->left, pos_front_L,
               pos_back_L, body_pitch_rad);
    update_leg(vmc, vmc->JRM_r, &vmc->right, -pos_front_R,
               -pos_back_R, body_pitch_rad);
    // 与输出模式无关；首次采样和无效 dt 时，也先刷新几何与弹簧反馈。
    update_springs(vmc);

    if (!vmc->derivative_initialized) {
        VMC_ResetFeedbackHistory(vmc);
        vmc->left.theta_unwrapped = vmc->left.theta[0];
        vmc->right.theta_unwrapped = vmc->right.theta[0];
        vmc->derivative_initialized = true;
        return;
    }

    // 环形差分同时消除腿角 atan2 和机身 Pitch 跨 +/-pi 时的假跳变。
    const float left_theta_step =
        VMC_WrapAngle(vmc->left.theta[0] - vmc->left.theta_last[0]);
    const float right_theta_step =
        VMC_WrapAngle(vmc->right.theta[0] - vmc->right.theta_last[0]);
    vmc->left.theta_unwrapped += left_theta_step;
    vmc->right.theta_unwrapped += right_theta_step;

    if (!isfinite(dt) || dt <= 1.0e-6f) {
        // 位置历史已更新，下次有效 dt 从本帧开始差分；连续角保留累计圈数。
        vmc->left.length[1] = 0.0f;
        vmc->right.length[1] = 0.0f;
        vmc->left.theta[1] = 0.0f;
        vmc->right.theta[1] = 0.0f;
        return;
    }

    // 腿长变化率
    vmc->left.length[1] = diff(vmc->left.length[0], vmc->left.length_last[0], dt);
    vmc->right.length[1] = diff(vmc->right.length[0], vmc->right.length_last[0], dt);

    // 腿角速度：差分后使用一阶低通，抑制编码器量化噪声被 1/dt 放大。
    const float left_theta_rate_raw =
        left_theta_step / dt;
    const float right_theta_rate_raw =
        right_theta_step / dt;
    const float omega_dt = VMC_TWO_PI * VMC_THETA_RATE_LPF_HZ * dt;
    const float alpha = (omega_dt > 0.0f) ? omega_dt / (1.0f + omega_dt) : 0.0f;
    vmc->left.theta[1] += alpha * (left_theta_rate_raw - vmc->left.theta[1]);
    vmc->right.theta[1] += alpha * (right_theta_rate_raw - vmc->right.theta[1]);
}

void VMC_ForceToTorque(const float JRM[2][2], float force, float Tp_target,VMC_LegSite_t *leg)
{
    leg->Tp_front = JRM[0][0] * force + JRM[0][1] * Tp_target;
    leg->Tp_back = JRM[1][0] * force + JRM[1][1] * Tp_target;
}

void VMC_CalcTorque(VMC_Control_t *vmc, float left_force, float right_force,
                     float left_torque, float right_torque)
{
    for (int side = 0; side < 2; ++side) {
        VMC_LegSite_t *leg = side ? &vmc->right : &vmc->left;
        const VMC_AirSpring_t *spring = &vmc->spring[side];
        const float (*JRM)[2] = side ? vmc->JRM_r : vmc->JRM_l;
        VMC_ForceToTorque(JRM, side ? right_force : left_force,
                          side ? right_torque : left_torque, leg);
    }
}

bool VMC_TorqueToForce(const float JRM[2][2], float torque_front,
                       float torque_back, VMC_LegSite_t *leg)
{
    const float determinant =
        JRM[0][0] * JRM[1][1] - JRM[0][1] * JRM[1][0];
    if (fabsf(determinant) < 1.0e-6f) {
        leg->support_force = 0.0f;
        leg->support_torque = 0.0f;
        return false;
    }

    const float determinant_inv = 1.0f / determinant;
    leg->support_force =
        (JRM[1][1] * torque_front - JRM[0][1] * torque_back) *
        determinant_inv;
    leg->support_torque =
        (JRM[0][0] * torque_back - JRM[1][0] * torque_front) *
        determinant_inv;
    return true;
}

static float vmc_clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/*
 * 逆解算：
 * 输入 target_length 目标虚拟腿长
 * 输入 target_theta  目标虚拟腿角，与 VMC_Update 输出的 leg.theta[0] 同坐标系
 * 输入 body_pitch_rad 当前机体俯仰角，弧度
 * 输出 pos_front / pos_back 前后关节电机目标角度，弧度
 *
 * 返回 false 表示目标不可达。
 */
bool VMC_InverseKinematics(const VMC_Control_t *vmc,
                           float target_length,
                           float target_theta,
                           float body_pitch_rad,
                           float *pos_front,
                           float *pos_back)
{
    if (vmc == NULL || pos_front == NULL || pos_back == NULL) {
        return false;
    }

    const float l1 = vmc->l1;
    const float l2 = vmc->l2;
    const float l3 = vmc->l3;
    const float l4 = vmc->l4;
    const float L  = target_length;

    if (!isfinite(L) || !isfinite(target_theta) ||
        !isfinite(body_pitch_rad) ||
        !isfinite(l1) || !isfinite(l2) ||
        !isfinite(l3) || !isfinite(l4) ||
        L < VMC_LEG_MIN_LENGTH_M || L > VMC_LEG_MAX_LENGTH_M ||
        l1 <= 1.0e-6f || l2 <= 1.0e-6f ||
        l3 <= 1.0e-6f || l4 <= 1.0e-6f) {
        return false;
    }

    /* 与 update_leg 中 theta = theta_body + body_pitch_rad 对应 */
    const float theta_body = VMC_WrapAngle(target_theta - body_pitch_rad);

    /*
     * update_leg 中：
     *   theta_body = -atan2f(xc, yc)
     *   length     = sqrtf(xc*xc + yc*yc)
     * 因此反算虚拟足端 C：
     */
    const float xc = -L * sinf(theta_body);
    const float yc =  L * cosf(theta_body);

    /* O -> C 方向角 */
    const float gamma = atan2f(yc, xc);

    /*
     * 后腿三角形 O-B-C：
     *   |OB| = l1, |BC| = l2, |OC| = L
     * 余弦定理求 B 方向相对 gamma 的夹角 delta_back
     */
    const float cos_delta_back =
        (l1 * l1 + L * L - l2 * l2) / (2.0f * l1 * L);

    /*
     * 前腿三角形 O-D-C：
     *   |OD| = l4, |DC| = l3, |OC| = L
     */
    const float cos_delta_front =
        (l4 * l4 + L * L - l3 * l3) / (2.0f * l4 * L);

    const float reach_tolerance = 1.0e-5f;
    if (cos_delta_back < -1.0f - reach_tolerance ||
        cos_delta_back > 1.0f + reach_tolerance ||
        cos_delta_front < -1.0f - reach_tolerance ||
        cos_delta_front > 1.0f + reach_tolerance) {
        return false;
    }

    const float delta_back =
        acosf(vmc_clampf(cos_delta_back, -1.0f, 1.0f));
    const float delta_front =
        acosf(vmc_clampf(cos_delta_front, -1.0f, 1.0f));

    /*
     * 选择与 update_leg 中 sqrt(disc) 分支一致的非奇异装配：
     *   后腿 B = gamma + delta_back
     *   前腿 D = gamma - delta_front
     * 如果你的实际机构装配相反，可交换加减号。
     */
    float back  = gamma + delta_back;
    float front = gamma - delta_front;

    /* 保留原接口的单圈输出；连续关节目标使用下方的连续逆解接口。 */
    back  = VMC_WrapAngle(back);
    front = VMC_WrapAngle(front);

    if (!isfinite(back) || !isfinite(front)) {
        return false;
    }

    *pos_back  = back;
    *pos_front = front;

    return true;
}

bool VMC_InverseKinematicsContinuous(const VMC_Control_t *vmc,
                                     float target_length,
                                     float target_theta,
                                     float body_pitch_rad,
                                     float reference_front,
                                     float reference_back,
                                     float *pos_front,
                                     float *pos_back)
{
    if (pos_front == NULL || pos_back == NULL ||
        !isfinite(reference_front) || !isfinite(reference_back)) {
        return false;
    }

    float front;
    float back;
    if (!VMC_InverseKinematics(vmc, target_length, target_theta,
                               body_pitch_rad, &front, &back)) {
        return false;
    }

    front = VMC_UnwrapAngleNear(front, reference_front);
    back = VMC_UnwrapAngleNear(back, reference_back);
    if (!isfinite(front) || !isfinite(back)) {
        return false;
    }
    *pos_front = front;
    *pos_back = back;
    return true;
}
