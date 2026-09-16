//
// Created by qza on 2026/7/17.
//

#include "Kalman_Observer.h"
#include "All_define.h"
#include <string.h>

/* ==================== Estimator ==================== */
void Estimator_Leg_Init(Kalman_Observer_t *est) {
    est->outstate.s = 0.0f;
    est->outstate.dot_s = 0.0f;
    est->last_wheel_distance = 0.0f;
    est->last_wheel_speed = 0.0f;
    est->wheel_speed_input = 0.0f;
    // 初始化KF维度: 状态量x(2维: s, dot_s), 控制量u(1维: accel_x), 观测量z(2维: 轮速里程计的dot_s)
    Kalman_Filter_Init(&est->kf, 3, 0, 2);
    // 禁用自动调整，使用固定矩阵维度
    est->kf.UseAutoAdjustment = 0;
    // 配置状态观测矩阵 H: z = 0 * s + 1 * dot_s
    // H 是 2x3
    est->kf.H_data[0] = 0.0f;
    est->kf.H_data[1] = 1.0f;   // z0 = dot_s，轮速
    est->kf.H_data[2] = 0.0f;

    est->kf.H_data[3] = 0.0f;
    est->kf.H_data[4] = 0.0f;
    est->kf.H_data[5] = 1.0f;   // z1 = ddot_s，IMU 加速度

    // 配置过程噪声协方差矩阵 Q (2x2 对角阵，信任动力学模型程度)
    est->kf.Q_data[0] = 0.01f;  // s 的过程噪声
    est->kf.Q_data[4] = 0.05f;  // dot_s 的过程噪声
    est->kf.Q_data[8] = 0.1f;

    // 配置测量噪声协方差矩阵 R (2x2，信任轮式里程计的程度)
    est->kf.R_data[0] = 0.005f;   // 轮位移测量噪声
    est->kf.R_data[3] = 0.05f;

    // 初始化误差协方差矩阵 P (3x3)
    est->kf.P_data[0] = 1.0f;
    est->kf.P_data[4] = 1.0f;
    est->kf.P_data[8] = 1.0f;

    // 初始化状态最小值限制防止过度收敛
    est->kf.StateMinVariance[0] = 1e-4f;
    est->kf.StateMinVariance[1] = 1e-4f;
    est->kf.StateMinVariance[2] = 1e-4f;
}

void Estimator_Leg_Update(Kalman_Observer_t *est, float wheel_speed,
                          const float imu_accel_x, const float dt) {
    const float raw_dot_s = wheel_speed;
    const float raw_ddot_s = imu_accel_x;
    // 2. 动态更新状态转移矩阵 F (2x2) 行优先存储
    // [ 1  dt ]
    // [ 0  1  ]
    est->kf.F_data[0] = 1.0f;
    est->kf.F_data[1] = dt;
    est->kf.F_data[2] = dt*dt*0.5f;
    est->kf.F_data[3] = 0.0f;
    est->kf.F_data[4] = 1.0f;
    est->kf.F_data[5] = dt;
    est->kf.F_data[6] = 0.0f;
    est->kf.F_data[7] = 0.0f;
    est->kf.F_data[8] = 1.0f;

    // 4. 装载控制量 U 和观测量 Z
    est->kf.MeasuredVector[0] = raw_dot_s ; // 轮速里程计作为观测量
    est->kf.MeasuredVector[1] = raw_ddot_s ; // IMU 加速度观测量
    // 5. 执行滤波迭代
    float* filtered_states = Kalman_Filter_Update(&est->kf);
    // 6. 提取平滑后的状态
    est->outstate.s = filtered_states[0];
    est->outstate.dot_s = filtered_states[1];
}

void Estimator_Set_WheelSpeed(Kalman_Observer_t *est, float wheel_speed)
{
    if (est != NULL) {
        est->wheel_speed_input = wheel_speed;
    }
}

void Estimator_QR_Change(Kalman_Observer_t *est,const float Q_data[4],const float R_data[1]) {
    est->kf.Q_data[0] = Q_data[0];
    est->kf.Q_data[1] = Q_data[1];
    est->kf.Q_data[2] = Q_data[2];
    est->kf.Q_data[3] = Q_data[3];

    est->kf.R_data[0] = R_data[0];

}

void Quat_Rotate_Vector(const float q[4], const float v[3], float out[3]) {
    float x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3]) * v[0]
             + 2 * v[1] * (q[2]*q[1] - q[0]*q[3])
             + 2 * v[2] * (q[0]*q[2] + q[3]*q[1]);
    float y = 2 * v[0] * (q[0]*q[3] + q[2]*q[1])
             + v[1] * (1 - 2*q[1]*q[1] - 2*q[3]*q[3])
             + 2 * v[2] * (q[2]*q[3] - q[1]*q[0]);
    float z = 2 * v[0] * (q[3]*q[1] - q[0]*q[2])
             + 2 * v[1] * (q[0]*q[1] + q[3]*q[2])
             + v[2] * (1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}
void Quat_Rotate_Vector_Inv(const float q[4], const float v[3], float out[3]) {
    float qconj[4] = { q[0], -q[1], -q[2], -q[3] };
    Quat_Rotate_Vector(qconj, v, out);
}

void Decompose_Acceleration(const float acc_meas_body[3],const float q_bw[4],
    const float g_world[3],float acc_motion_world[3],float acc_motion_body[3])
{
    // 1. 重力投影到车体坐标系
    float g_body[3];
    Quat_Rotate_Vector_Inv(q_bw, g_world, g_body);
    // 2. 车体运动加速度 = 测量值 - 重力分量
    for (int i = 0; i < 3; i++) {
        acc_motion_body[i] = acc_meas_body[i] - g_body[i];
    }
    // 3. 转到世界系
    Quat_Rotate_Vector(q_bw, acc_motion_body, acc_motion_world);
 }
float Forward_Acc_XZ(const float q_bw[4], const float acc_motion_world[3]) {
    float body_x[3] = {1.0f, 0.0f, 0.0f};
    float body_z[3] = {0.0f, 0.0f, 1.0f};
    float world_x[3], world_z[3];

    Quat_Rotate_Vector(q_bw, body_x, world_x);
    Quat_Rotate_Vector(q_bw, body_z, world_z);

    // 归一化（理论上旋转保模长，数值误差保险）
    float nx = sqrtf(world_x[0]*world_x[0]
                   + world_x[1]*world_x[1]
                   + world_x[2]*world_x[2]);
    float nz = sqrtf(world_z[0]*world_z[0]
                   + world_z[1]*world_z[1]
                   + world_z[2]*world_z[2]);
    if (nx < 1e-6f || nz < 1e-6f) return 0.0f;

    world_x[0] /= nx; world_x[1] /= nx; world_x[2] /= nx;
    world_z[0] /= nz; world_z[1] /= nz; world_z[2] /= nz;

    // 投影到机体 x、z 轴
    float a_x = acc_motion_world[0]*world_x[0]
              + acc_motion_world[1]*world_x[1]
              + acc_motion_world[2]*world_x[2];

    float a_z = acc_motion_world[0]*world_z[0]
              + acc_motion_world[1]*world_z[1]
              + acc_motion_world[2]*world_z[2];

    // xz 平面内合成，用 a_x 定符号
    float a_xz = sqrtf(a_x*a_x + a_z*a_z);
    return (a_x >= 0.0f) ? a_xz : -a_xz;
}

void Estimator_Task(Kalman_Observer_t *est, const IMU_Data_t imu_data, const float dt) {

    float g_world[3]={0.0f, 0.0f, 9.81f};
    float acc_motion_world[3];
    float acc_motion_body[3];
    Decompose_Acceleration(imu_data.accel,imu_data.q, g_world,
    acc_motion_world,acc_motion_body);
    float imu_accel_x = Forward_Acc_XZ(imu_data.q,acc_motion_world);
    Estimator_Leg_Update(est, est->wheel_speed_input, imu_accel_x, dt);
}
