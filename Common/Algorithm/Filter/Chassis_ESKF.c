//
// Created by qza on 2026/9/8.
//

#include "Chassis_ESKF.h"

#include <math.h>
#include <string.h>

#define STATE_DIM 5

static float eskf_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void mat5_identity(float m[25], float diag)
{
    memset(m, 0, 25 * sizeof(float));
    m[0] = diag;
    m[6] = diag;
    m[12] = diag;
    m[18] = diag;
    m[24] = diag;
}

static void mat5_copy(const float src[25], float dst[25])
{
    memcpy(dst, src, 25 * sizeof(float));
}

static void mat5_transpose(const float a[25], float at[25])
{
    for (int r = 0; r < STATE_DIM; r++) {
        for (int c = 0; c < STATE_DIM; c++) {
            at[c * STATE_DIM + r] = a[r * STATE_DIM + c];
        }
    }
}

static void mat5_multiply(const float a[25], const float b[25], float out[25])
{
    float tmp[25];
    for (int r = 0; r < STATE_DIM; r++) {
        for (int c = 0; c < STATE_DIM; c++) {
            float sum = 0.0f;
            for (int k = 0; k < STATE_DIM; k++) {
                sum += a[r * STATE_DIM + k] * b[k * STATE_DIM + c];
            }
            tmp[r * STATE_DIM + c] = sum;
        }
    }
    mat5_copy(tmp, out);
}

static void mat5_symmetrize(float p[25])
{
    for (int r = 0; r < STATE_DIM; r++) {
        for (int c = r + 1; c < STATE_DIM; c++) {
            float v = 0.5f * (p[r * STATE_DIM + c] + p[c * STATE_DIM + r]);
            p[r * STATE_DIM + c] = v;
            p[c * STATE_DIM + r] = v;
        }
    }
}

static void eskf_gravity_compensate(const Chassis_ESKF_t *f,
                                    const Chassis_ESKF_Input_t *in,
                                    float *ax, float *ay, float *az)
{
    if (in->attitude_valid) {
        float sr = sinf(in->roll);
        float cr = cosf(in->roll);
        float sp = sinf(in->pitch);
        float cp = cosf(in->pitch);

        float gx = -f->gravity * sp;
        float gy =  f->gravity * sr * cp;
        float gz =  f->gravity * cr * cp;

        *ax = in->imu_ax - gx;
        *ay = in->imu_ay - gy;
        *az = in->imu_az - gz;
    }
    else {
        *ax = in->imu_ax;
        *ay = in->imu_ay;
        *az = in->imu_az;
    }
}

static uint8_t eskf_is_static(const Chassis_ESKF_t *f,
                              const Chassis_ESKF_Input_t *in,
                              float ax, float ay, float az)
{
    float gyro_norm = sqrtf(in->imu_gx * in->imu_gx +
                            in->imu_gy * in->imu_gy +
                            in->imu_gz * in->imu_gz);

    if (gyro_norm > f->static_gyro_th) {
        return 0;
    }

    if (!in->wheel_valid) {
        return 0;
    }

    if (fabsf(in->wheel_vx) > f->static_wheel_v_th ||
        fabsf(in->wheel_vy) > f->static_wheel_v_th ||
        fabsf(in->wheel_vw) > f->static_wheel_w_th) {
        return 0;
    }

    if (in->attitude_valid) {
        float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
        if (acc_norm > 0.8f) {
            return 0;
        }
    }

    return 1;
}

static void eskf_predict(Chassis_ESKF_t *f, float ax, float ay, float gz, float dt)
{
    float vx0 = f->x[0];
    float vy0 = f->x[1];
    float bax = f->x[2];
    float bay = f->x[3];
    float bgz = f->x[4];
    float wz  = gz - bgz;

    f->x[0] = vx0 + (ax - bax + wz * vy0) * dt;
    f->x[1] = vy0 + (ay - bay - wz * vx0) * dt;

    float F[25];
    mat5_identity(F, 1.0f);
    F[1]  = wz * dt;
    F[2]  = -dt;
    F[4]  = -vy0 * dt;
    F[5]  = -wz * dt;
    F[8]  = -dt;
    F[9]  =  vx0 * dt;

    float FP[25];
    float Ft[25];
    float Pnew[25];
    mat5_multiply(F, f->P, FP);
    mat5_transpose(F, Ft);
    mat5_multiply(FP, Ft, Pnew);

    float q_v   = f->sigma_acc * f->sigma_acc * dt * dt;
    float q_ba  = f->sigma_bias_acc * f->sigma_bias_acc * dt;
    float q_bgz = f->sigma_bias_gyr * f->sigma_bias_gyr * dt;

    Pnew[0]  += q_v;
    Pnew[6]  += q_v;
    Pnew[12] += q_ba;
    Pnew[18] += q_ba;
    Pnew[24] += q_bgz;

    mat5_copy(Pnew, f->P);
    mat5_symmetrize(f->P);
}

static void eskf_update_velocity(Chassis_ESKF_t *f,
                                 float vx_meas, float vy_meas,
                                 float r_vx, float r_vy)
{
    float p00 = f->P[0];
    float p01 = f->P[1];
    float p10 = f->P[5];
    float p11 = f->P[6];

    float s00 = p00 + r_vx;
    float s01 = p01;
    float s10 = p10;
    float s11 = p11 + r_vy;
    float det = s00 * s11 - s01 * s10;
    if (fabsf(det) < 1e-8f) {
        return;
    }

    float inv00 =  s11 / det;
    float inv01 = -s01 / det;
    float inv10 = -s10 / det;
    float inv11 =  s00 / det;

    float res0 = vx_meas - f->x[0];
    float res1 = vy_meas - f->x[1];

    float row0[STATE_DIM];
    float row1[STATE_DIM];
    memcpy(row0, &f->P[0], STATE_DIM * sizeof(float));
    memcpy(row1, &f->P[5], STATE_DIM * sizeof(float));

    float K0[STATE_DIM];
    float K1[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        float p_i0 = f->P[i * STATE_DIM + 0];
        float p_i1 = f->P[i * STATE_DIM + 1];
        K0[i] = p_i0 * inv00 + p_i1 * inv10;
        K1[i] = p_i0 * inv01 + p_i1 * inv11;
        f->x[i] += K0[i] * res0 + K1[i] * res1;
    }

    for (int r = 0; r < STATE_DIM; r++) {
        for (int c = 0; c < STATE_DIM; c++) {
            f->P[r * STATE_DIM + c] -= K0[r] * row0[c] + K1[r] * row1[c];
        }
    }

    mat5_symmetrize(f->P);
}

static void eskf_update_bgz(Chassis_ESKF_t *f, float bgz_meas, float r_bgz)
{
    float s = f->P[24] + r_bgz;
    if (s < 1e-8f) {
        return;
    }

    float res = bgz_meas - f->x[4];
    float row4[STATE_DIM];
    memcpy(row4, &f->P[20], STATE_DIM * sizeof(float));

    float K[STATE_DIM];
    for (int i = 0; i < STATE_DIM; i++) {
        K[i] = f->P[i * STATE_DIM + 4] / s;
        f->x[i] += K[i] * res;
    }

    for (int r = 0; r < STATE_DIM; r++) {
        for (int c = 0; c < STATE_DIM; c++) {
            f->P[r * STATE_DIM + c] -= K[r] * row4[c];
        }
    }

    mat5_symmetrize(f->P);
}

void Chassis_ESKF_Reset(Chassis_ESKF_t *f)
{
    if (f == NULL) {
        return;
    }

    memset(f->x, 0, sizeof(f->x));
    mat5_identity(f->P, 0.0f);
    f->P[0]  = 1.0f;
    f->P[6]  = 1.0f;
    f->P[12] = 0.5f;
    f->P[18] = 0.5f;
    f->P[24] = 0.2f;
    f->slip_score = 0.0f;
    f->initialized = 0;
}

void Chassis_ESKF_Init(Chassis_ESKF_t *f)
{
    if (f == NULL) {
        return;
    }

    memset(f, 0, sizeof(*f));
    f->gravity = 9.80665f;
    f->sigma_acc = 1.0f;
    f->sigma_bias_acc = 0.12f;
    f->sigma_bias_gyr = 0.02f;
    f->wheel_v_meas_std = 0.005f;
    f->wheel_w_meas_std = 0.02f;
    f->slip_gain = 500.0f;
    f->slip_threshold_v = 0.005f;
    f->slip_threshold_w = 0.03f;
    f->static_gyro_th = 0.02f;
    f->static_wheel_v_th = 0.02f;
    f->static_wheel_w_th = 0.03f;
    Chassis_ESKF_Reset(f);
}

void Chassis_ESKF_Update(Chassis_ESKF_t *f,
                         const Chassis_ESKF_Input_t *in,
                         Chassis_ESKF_Output_t *out)
{
    if (f == NULL || in == NULL || out == NULL) {
        return;
    }

    float dt = in->dt;
    if (dt <= 1e-5f) {
        dt = 1e-3f;
    }
    if (dt > 0.05f) {
        dt = 0.05f;
    }

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    eskf_gravity_compensate(f, in, &ax, &ay, &az);

    if (!f->initialized) {
        f->x[0] = in->wheel_valid ? in->wheel_vx : 0.0f;
        f->x[1] = in->wheel_valid ? in->wheel_vy : 0.0f;
        f->x[2] = 0.0f;
        f->x[3] = 0.0f;
        f->x[4] = in->wheel_valid ? (in->imu_gz - in->wheel_vw) : 0.0f;
        f->initialized = 1;
    }

    eskf_predict(f, ax, ay, in->imu_gz, dt);

    uint8_t static_state = eskf_is_static(f, in, ax, ay, az);

    if (in->wheel_valid) {
        float res_vx = in->wheel_vx - f->x[0];
        float res_vy = in->wheel_vy - f->x[1];
        float vel_res_norm = sqrtf(res_vx * res_vx + res_vy * res_vy);
        float bgz_meas = in->imu_gz - in->wheel_vw;
        float res_bgz = bgz_meas - f->x[4];

        float slip_v_now = eskf_clampf((vel_res_norm - f->slip_threshold_v) /
                                       (f->slip_threshold_v + 1e-6f), 0.0f, 1.0f);
        float slip_w_now = eskf_clampf((fabsf(res_bgz) - f->slip_threshold_w) /
                                       (f->slip_threshold_w + 1e-6f), 0.0f, 1.0f);

        float slip_now = fmaxf(slip_v_now, slip_w_now);
        if (static_state) {
            slip_now = 0.0f;
        }

        float alpha = eskf_clampf(dt / 0.15f, 0.0f, 1.0f);
        f->slip_score += alpha * (slip_now - f->slip_score);
        f->slip_score = eskf_clampf(f->slip_score, 0.0f, 1.0f);

        float r_scale_v = 1.0f + f->slip_gain * slip_v_now;
        float r_scale_w = 1.0f + f->slip_gain * slip_w_now;
        if (static_state) {
            r_scale_v = 0.05f;
            r_scale_w = 0.05f;
        }

        float r_v = f->wheel_v_meas_std * f->wheel_v_meas_std * r_scale_v;
        float r_w = f->wheel_w_meas_std * f->wheel_w_meas_std * r_scale_w;

        eskf_update_velocity(f, in->wheel_vx, in->wheel_vy, r_v, r_v);
        eskf_update_bgz(f, bgz_meas, r_w);
    }
    else {
        f->slip_score += (0.0f - f->slip_score) * eskf_clampf(dt / 0.5f, 0.0f, 1.0f);
    }

    out->vx = f->x[0];
    out->vy = f->x[1];
    out->vw = in->imu_gz - f->x[4];
    out->bax = f->x[2];
    out->bay = f->x[3];
    out->bgz = f->x[4];
    out->slip_score = f->slip_score;

    float cov_xy = 0.5f * (f->P[0] + f->P[6]);
    float cov_w  = f->P[24];
    out->vx_var = f->P[0];
    out->vy_var = f->P[6];
    out->vw_var = cov_w;

    float conf = 1.0f / (1.0f + 2.0f * sqrtf(fmaxf(cov_xy, 0.0f)) + 1.5f * sqrtf(fmaxf(cov_w, 0.0f)));
    conf *= (1.0f - 0.8f * f->slip_score);
    out->confidence = eskf_clampf(conf, 0.0f, 1.0f);
}
