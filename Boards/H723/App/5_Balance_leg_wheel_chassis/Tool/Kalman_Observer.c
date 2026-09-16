//
// Created by qza on 2026/7/17.
//

#include "Kalman_Observer.h"
#include "All_define.h"
#include <math.h>
#include <string.h>

#define ESTIMATOR_STATE_DIM 3
#define ESTIMATOR_MIN_VARIANCE 1.0e-6f

static float Estimator_Clamp(const float value, const float min_value,
                             const float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void Estimator_Symmetrize_Covariance(float covariance[9])
{
    for (int row = 0; row < ESTIMATOR_STATE_DIM; row++) {
        for (int col = row + 1; col < ESTIMATOR_STATE_DIM; col++) {
            const float value = 0.5f *
                (covariance[row * ESTIMATOR_STATE_DIM + col] +
                 covariance[col * ESTIMATOR_STATE_DIM + row]);
            covariance[row * ESTIMATOR_STATE_DIM + col] = value;
            covariance[col * ESTIMATOR_STATE_DIM + row] = value;
        }
        if (covariance[row * ESTIMATOR_STATE_DIM + row] <
            ESTIMATOR_MIN_VARIANCE) {
            covariance[row * ESTIMATOR_STATE_DIM + row] =
                ESTIMATOR_MIN_VARIANCE;
        }
    }
}

static void Estimator_Predict(Kalman_Observer_t *est,
                              const float imu_accel_x, const float dt)
{
    const float dt2 = dt * dt;
    const float velocity = est->state[1];
    const float accel = imu_accel_x - est->state[2];

    est->state[0] += velocity * dt + 0.5f * accel * dt2;
    est->state[1] += accel * dt;

    /* F = [1 dt -dt^2/2; 0 1 -dt; 0 0 1], P = F P F' + Q */
    const float F[9] = {
        1.0f, dt, -0.5f * dt2,
        0.0f, 1.0f, -dt,
        0.0f, 0.0f, 1.0f
    };
    float FP[9] = {0.0f};
    float predicted_covariance[9] = {0.0f};

    for (int row = 0; row < ESTIMATOR_STATE_DIM; row++) {
        for (int col = 0; col < ESTIMATOR_STATE_DIM; col++) {
            for (int k = 0; k < ESTIMATOR_STATE_DIM; k++) {
                FP[row * ESTIMATOR_STATE_DIM + col] +=
                    F[row * ESTIMATOR_STATE_DIM + k] *
                    est->covariance[k * ESTIMATOR_STATE_DIM + col];
            }
        }
    }

    for (int row = 0; row < ESTIMATOR_STATE_DIM; row++) {
        for (int col = 0; col < ESTIMATOR_STATE_DIM; col++) {
            for (int k = 0; k < ESTIMATOR_STATE_DIM; k++) {
                predicted_covariance[row * ESTIMATOR_STATE_DIM + col] +=
                    FP[row * ESTIMATOR_STATE_DIM + k] *
                    F[col * ESTIMATOR_STATE_DIM + k];
            }
        }
    }

    const float accel_variance = est->accel_noise * est->accel_noise;
    const float bias_variance = est->accel_bias_noise *
                                est->accel_bias_noise * dt;
    predicted_covariance[0] += 0.25f * dt2 * dt2 * accel_variance;
    predicted_covariance[1] += 0.5f * dt2 * dt * accel_variance;
    predicted_covariance[3] += 0.5f * dt2 * dt * accel_variance;
    predicted_covariance[4] += dt2 * accel_variance;
    predicted_covariance[8] += bias_variance;

    memcpy(est->covariance, predicted_covariance,
           sizeof(predicted_covariance));
    Estimator_Symmetrize_Covariance(est->covariance);
}

static void Estimator_Update_WheelSpeed(Kalman_Observer_t *est,
                                        const float wheel_speed,
                                        const float dt)
{
    const float residual = wheel_speed - est->state[1];
    const float slip_now = Estimator_Clamp(
        (fabsf(residual) - est->slip_threshold) /
        (est->slip_threshold + 1.0e-6f), 0.0f, 1.0f);
    const float slip_alpha = Estimator_Clamp(dt / 0.15f, 0.0f, 1.0f);
    est->slip_score += slip_alpha * (slip_now - est->slip_score);

    const float measurement_variance =
        est->wheel_speed_noise * est->wheel_speed_noise *
        (1.0f + est->slip_gain * slip_now);
    const float innovation_variance = est->covariance[4] +
                                      measurement_variance;
    if (innovation_variance <= ESTIMATOR_MIN_VARIANCE) {
        return;
    }

    float gain[ESTIMATOR_STATE_DIM];
    float velocity_covariance_row[ESTIMATOR_STATE_DIM];
    for (int i = 0; i < ESTIMATOR_STATE_DIM; i++) {
        gain[i] = est->covariance[i * ESTIMATOR_STATE_DIM + 1] /
                  innovation_variance;
        velocity_covariance_row[i] = est->covariance[3 + i];
        est->state[i] += gain[i] * residual;
    }

    for (int row = 0; row < ESTIMATOR_STATE_DIM; row++) {
        for (int col = 0; col < ESTIMATOR_STATE_DIM; col++) {
            est->covariance[row * ESTIMATOR_STATE_DIM + col] -=
                gain[row] * velocity_covariance_row[col];
        }
    }
    Estimator_Symmetrize_Covariance(est->covariance);
}

/* ==================== Estimator ==================== */
void Estimator_Leg_Init(Kalman_Observer_t *est)
{
    if (est == NULL) {
        return;
    }

    memset(est, 0, sizeof(*est));
    est->covariance[0] = 1.0f;
    est->covariance[4] = 1.0f;
    est->covariance[8] = 0.5f;
    est->accel_noise = 0.32f;
    est->accel_bias_noise = 0.05f;
    est->wheel_speed_noise = 0.071f;
    est->slip_threshold = 0.15f;
    est->slip_gain = 20.0f;
}

void Estimator_Leg_Update(Kalman_Observer_t *est, float wheel_speed,
                          const float imu_accel_x, const float dt)
{
    if (est == NULL) {
        return;
    }

    const float valid_dt = Estimator_Clamp(dt, 1.0e-4f, 0.05f);
    if (!est->initialized) {
        est->state[1] = wheel_speed;
        est->initialized = 1U;
    }

    Estimator_Predict(est, imu_accel_x, valid_dt);
    Estimator_Update_WheelSpeed(est, wheel_speed, valid_dt);

    est->last_wheel_speed = wheel_speed;
    est->last_wheel_distance += wheel_speed * valid_dt;
    est->outstate.s = est->state[0];
    est->outstate.dot_s = est->state[1];
}

void Estimator_Set_WheelSpeed(Kalman_Observer_t *est, float wheel_speed)
{
    if (est != NULL) {
        est->wheel_speed_input = wheel_speed;
    }
}

void Estimator_QR_Change(Kalman_Observer_t *est,
                         const float Q_data[4], const float R_data[1])
{
    if (est == NULL || Q_data == NULL || R_data == NULL) {
        return;
    }

    /* 保留原接口；Q[0] 调整加速度噪声，Q[3] 调整零偏随机游走。 */
    est->accel_noise = sqrtf(fmaxf(Q_data[0], ESTIMATOR_MIN_VARIANCE));
    est->accel_bias_noise = sqrtf(fmaxf(Q_data[3], ESTIMATOR_MIN_VARIANCE));
    est->wheel_speed_noise = sqrtf(fmaxf(R_data[0],
                                        ESTIMATOR_MIN_VARIANCE));
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
