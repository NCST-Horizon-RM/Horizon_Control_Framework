//
// Created by CaoKangqi on 2026/6/24.
//
#include "Model_Control.h"
#include <string.h>

/* ======================== VMC ======================== */
void VMC_Init(VMC_t *vmc, float l1, float l2, float l3, float l4, float l5) {
    vmc->l1 = l1; vmc->l2 = l2; vmc->l3 = l3;
    vmc->l4 = l4; vmc->l5 = l5;
    vmc->L0 = 0.0f; vmc->phi0 = 0.0f;
    memset(vmc->JRM, 0, sizeof(vmc->JRM));
}

void VMC_Update_Kinematics(VMC_t *vmc, float theta_front, float theta_back) {
    float x_B = 0.0f, y_B = 0.0f, x_C = 0.0f, y_C = 0.0f, x_D = 0.0f, y_D = 0.0f;
    float A0 = 0.0f, B0 = 0.0f, C0 = 0.0f;
    float l_BD = 0.0f;
    float phi2 = 0.0f, phi3 = 0.0f;

    // 五连杆正运动学几何解算
    x_B = -vmc->l5 / 2.0f + cosf(theta_front) * vmc->l1;
    y_B = sinf(theta_front) * vmc->l1;
    x_D =  vmc->l5 / 2.0f + cosf(theta_back)  * vmc->l4;
    y_D = sinf(theta_back)  * vmc->l4;

    A0 = 2.0f * vmc->l2 * (x_D - x_B);
    B0 = 2.0f * vmc->l2 * (y_D - y_B);
    l_BD = sqrtf((x_D - x_B) * (x_D - x_B) + (y_D - y_B) * (y_D - y_B));
    C0 = vmc->l2 * vmc->l2 + l_BD * l_BD - vmc->l3 * vmc->l3;

    phi2 = 2.0f * atan2f(B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);

    x_C = -vmc->l5 / 2.0f + vmc->l1 * cosf(theta_front) + vmc->l2 * cosf(phi2);
    y_C =                   vmc->l1 * sinf(theta_front) + vmc->l2 * sinf(phi2);

    phi3 = atan2f(y_C - y_D, x_C - x_D);

    // 更新虚拟状态
    vmc->L0   = sqrtf(x_C * x_C + y_C * y_C);
    vmc->phi0 = atan2f(y_C, x_C);

    // 雅可比矩阵解算
    vmc->JRM[0][0] = -vmc->l1 * sinf(vmc->phi0 - phi3) * sinf(theta_front - phi2) / sinf(phi2 - phi3);
    vmc->JRM[0][1] = -vmc->l1 * sinf(theta_front - phi2) * cosf(vmc->phi0 - phi3) / (vmc->L0 * sinf(phi2 - phi3));
    vmc->JRM[1][0] = -vmc->l4 * sinf(vmc->phi0 - phi2) * sinf(phi3 - theta_back) / sinf(phi2 - phi3);
    vmc->JRM[1][1] = -vmc->l4 * sinf(phi3 - theta_back) * cosf(vmc->phi0 - phi2) / (vmc->L0 * sinf(phi2 - phi3));
}

void VMC_Inverse_Dynamics(VMC_t *vmc, float F0_target, float Tp_target, float *tau_front, float *tau_back) {
    *tau_front = vmc->JRM[0][0] * F0_target + vmc->JRM[0][1] * Tp_target;
    *tau_back  = vmc->JRM[1][0] * F0_target + vmc->JRM[1][1] * Tp_target;
}

/* ====================== LQR ======================== */
void LQR_Init(LQR_t *lqr, float coeffs[12][4]) {
    memcpy(lqr->K_coeffs, coeffs, sizeof(float) * 12 * 4);
    memset(lqr->K_matrix, 0, sizeof(lqr->K_matrix));
    lqr->Tp_out = 0.0f;
    lqr->Tw_out = 0.0f;
}

void LQR_Update_K_Matrix(LQR_t *lqr, float current_L0) {
    float L0_2 = current_L0 * current_L0;
    float L0_3 = L0_2 * current_L0;

    for (int i = 0; i < 12; i++) {
        float k_val = lqr->K_coeffs[i][0] * L0_3 +
                      lqr->K_coeffs[i][1] * L0_2 +
                      lqr->K_coeffs[i][2] * current_L0 +
                      lqr->K_coeffs[i][3];

        lqr->K_matrix[i / 6][i % 6] = k_val;
    }
}

void LQR_Calculate(LQR_t *lqr, float *measure, float *ref) {
    lqr->Tp_out = 0.0f;
    lqr->Tw_out = 0.0f;

    for (int i = 0; i < 6; i++) {
        float err = measure[i] - ref[i];
        lqr->Tp_out += -lqr->K_matrix[0][i] * err;
        lqr->Tw_out += -lqr->K_matrix[1][i] * err;
    }
}

/* ==================== Estimator ==================== */
void Estimator_Init(Estimator_t *est) {
    est->s = 0.0f;
    est->dot_s = 0.0f;
    est->last_wheel_speed = 0.0f;
    // 初始化KF维度: 状态量x(2维: s, dot_s), 控制量u(1维: accel_x), 观测量z(1维: 轮速解算的dot_s)
    Kalman_Filter_Init(&est->kf, 2, 1, 1);
    // 禁用自动调整，使用固定矩阵维度
    est->kf.UseAutoAdjustment = 0;
    // 配置状态观测矩阵 H: z = 0 * s + 1 * dot_s
    est->kf.H_data[0] = 0.0f;
    est->kf.H_data[1] = 1.0f;
    // 配置过程噪声协方差矩阵 Q (2x2 对角阵，信任动力学模型程度)
    est->kf.Q_data[0] = 0.001f;  // s 的过程噪声
    est->kf.Q_data[3] = 0.005f;  // dot_s 的过程噪声

    // 配置测量噪声协方差矩阵 R (1x1，信任轮式里程计的程度)
    est->kf.R_data[0] = 0.05f;   // 轮速测量噪声
    // 初始化误差协方差矩阵 P (2x2)
    est->kf.P_data[0] = 1.0f;
    est->kf.P_data[3] = 1.0f;
    // 初始化状态最小值限制防止过度收敛
    est->kf.StateMinVariance[0] = 1e-4f;
    est->kf.StateMinVariance[1] = 1e-4f;
}

void Estimator_Update(Estimator_t *est, float wheel_rpm, float imu_accel_x, float dt) {
    // 1. 将轮速转为线速度测量值 (假设轮周长155mm)
    float raw_dot_s = wheel_rpm * (0.155f * 3.14159f) / 60.0f;
    // 2. 动态更新状态转移矩阵 F (2x2) 行优先存储
    // [ 1  dt ]
    // [ 0   1 ]
    est->kf.F_data[0] = 1.0f;
    est->kf.F_data[1] = dt;
    est->kf.F_data[2] = 0.0f;
    est->kf.F_data[3] = 1.0f;
    // 3. 动态更新控制矩阵 B (2x1) 行优先存储
    // [ 0.5 * dt^2 ]
    // [     dt     ]
    est->kf.B_data[0] = 0.5f * dt * dt;
    est->kf.B_data[1] = dt;
    // 4. 装载控制量 U 和观测量 Z
    est->kf.ControlVector[0] = imu_accel_x; // IMU 加速度作为前馈控制输入
    est->kf.MeasuredVector[0] = raw_dot_s;  // 轮速里程计作为观测量
    // 5. 执行滤波迭代
    float* filtered_states = Kalman_Filter_Update(&est->kf);
    // 6. 提取平滑后的状态
    est->s = filtered_states[0];
    est->dot_s = filtered_states[1];
}