#include "VMC_Control.h"
#include "IMU_Task.h"
#include <math.h>
#include <string.h>

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
    const float phi3 = 2.0f * atan2f(b + sqrtf(disc), a + c);
    const float xc = xb + vmc->l2 * cosf(phi3);
    const float yc = yb + vmc->l2 * sinf(phi3);
    const float phi2 = atan2f(yc - yd, xc - xd);
    const float theta_body = -atan2f(xc, yc);
    const float theta = theta_body + body_pitch_rad;
    const float length = sqrtf(xc * xc + yc * yc);
    const float s23 = sinf(phi2 - phi3);
    const float safe_s23 = (fabsf(s23) > 1.0e-5f) ? s23 : copysignf(1.0e-5f, s23);

    leg->length_last[0] = leg->length[0];
    leg->theta_last[0] = leg->theta[0];
    leg->length[0] = length;
    leg->theta[0] = theta;
    leg->phi = atan2f(yc, xc);

    JRM[0][0] = vmc->l4 * cosf(theta - phi3) * sinf(phi2 - front) / safe_s23;
    JRM[0][1] = -vmc->l4 * sinf(theta - phi3) * sinf(phi2 - front) /
                (safe_s23 * fmaxf(length, 1.0e-5f));
    JRM[1][0] = vmc->l1 * cosf(theta - phi2) * sinf(back - phi3) / safe_s23;
    JRM[1][1] = -vmc->l1 * sinf(theta - phi2) * sinf(back - phi3) /
                (safe_s23 * fmaxf(length, 1.0e-5f));
}

void VMC_Init(VMC_Control_t *vmc, float l1, float l2, float l3, float l4)
{
    memset(vmc, 0, sizeof(*vmc));
    vmc->l1 = l1; vmc->l2 = l2; vmc->l3 = l3; vmc->l4 = l4;
}

void VMC_Update(VMC_Control_t *vmc, float dt, float body_pitch_rad,
                float pos_front_L, float pos_back_L,
                float pos_front_R, float pos_back_R)
{
    // 正运动学解算腿长腿角等数据
    update_leg(vmc, vmc->JRM_l, &vmc->left, pos_front_L,
               pos_back_L, body_pitch_rad);
    update_leg(vmc, vmc->JRM_r, &vmc->right, -pos_front_R,
               -pos_back_R, body_pitch_rad);

    if (!vmc->derivative_initialized) {
        vmc->left.length[1] = 0.0f;
        vmc->right.length[1] = 0.0f;
        vmc->left.theta[1] = 0.0f;
        vmc->right.theta[1] = 0.0f;
        vmc->derivative_initialized = true;
        return;
    }

    // 腿长变化率
    vmc->left.length[1] = diff(vmc->left.length[0], vmc->left.length_last[0], dt);
    vmc->right.length[1] = diff(vmc->right.length[0], vmc->right.length_last[0], dt);

    // 腿角速度：差分后使用一阶低通，抑制编码器量化噪声被 1/dt 放大。
    const float left_theta_rate_raw =
        diff(vmc->left.theta[0], vmc->left.theta_last[0], dt);
    const float right_theta_rate_raw =
        diff(vmc->right.theta[0], vmc->right.theta_last[0], dt);
    const float omega_dt = 6.283185307f * VMC_THETA_RATE_LPF_HZ * dt;
    const float alpha = (omega_dt > 0.0f) ? omega_dt / (1.0f + omega_dt) : 0.0f;
    vmc->left.theta[1] += alpha * (left_theta_rate_raw - vmc->left.theta[1]);
    vmc->right.theta[1] += alpha * (right_theta_rate_raw - vmc->right.theta[1]);
}

void VMC_ForceToTorque(const float JRM[2][2], float force, float Tp_target,VMC_LegSite_t *leg)
{
    leg->Tp_front = JRM[0][0] * force + JRM[0][1] * Tp_target;
    leg->Tp_back = JRM[1][0] * force + JRM[1][1] * Tp_target;
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
        L <= 1.0e-6f || l1 <= 1.0e-6f || l4 <= 1.0e-6f) {
        return false;
    }

    /* 与 update_leg 中 theta = theta_body + body_pitch_rad 对应 */
    const float theta_body = target_theta - body_pitch_rad;

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

    /* 归一化到 [-pi, pi]，便于直接给电机位置 */
    back  = atan2f(sinf(back),  cosf(back));
    front = atan2f(sinf(front), cosf(front));

    *pos_back  = back;
    *pos_front = front;

    return true;
}
