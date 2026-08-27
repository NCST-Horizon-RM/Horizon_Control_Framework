/**
 * @file       vqf_filter.c
 * @brief      VQF姿态滤波算法
 * @note
 */
#include "VQF_filter.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define EPS FLT_EPSILON
#define M_PIf       3.14159265358979323846f

struct VQF_FILTER_t vqf_filter;


static float vqf_square(float x) { return x * x; }
static float vqf_max(float a, float b) { return a > b ? a : b; }

static vqf_real_t vqf_norm(const vqf_real_t vec[], size_t N) {
    vqf_real_t s = 0;
    for(size_t i = 0; i < N; i++) s += vec[i]*vec[i];
    return sqrtf(s);
}

static void vqf_normalize(vqf_real_t vec[], size_t N) {
    vqf_real_t n = vqf_norm(vec, N);
    if (n < EPS) return;
    for(size_t i = 0; i < N; i++) vec[i] /= n;
}

static void vqf_clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max) {
    for(size_t i = 0; i < N; i++) {
        if (vec[i] < min) vec[i] = min;
        else if (vec[i] > max) vec[i] = max;
    }
}

static void vqf_quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4]) {
    vqf_real_t w = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    vqf_real_t x = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    vqf_real_t y = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    vqf_real_t z = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
    out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

static void vqf_quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3]) {
    vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
    vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
    vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    out[0] = x; out[1] = y; out[2] = z;
}

static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_double_t outB[], vqf_double_t outA[]) {
    vqf_double_t fc = (M_SQRT2 / (2.0*M_PIf))/(vqf_double_t)(tau);
    vqf_double_t C = tanf(M_PIf*fc*(vqf_double_t)(Ts));
    vqf_double_t D = C*C + M_SQRT2*C + 1;
    vqf_double_t b0 = C*C/D;
    outB[0] = b0; outB[1] = 2*b0; outB[2] = b0;
    outA[0] = 2*(C*C-1)/D;
    outA[1] = (1-M_SQRT2*C+C*C)/D;
}

static void filterInitialState(vqf_real_t x0, const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t out[]) {
    out[0] = x0*(1 - b[0]);
    out[1] = x0*(b[2] - a[1]);
}

static vqf_real_t filterStep(vqf_real_t x, const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t state[2]) {
    vqf_double_t y = b[0]*x + state[0];
    state[0] = b[1]*x - a[0]*y + state[1];
    state[1] = b[2]*x - a[1]*y;
    return y;
}

static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts, const vqf_double_t b[3],
                    const vqf_double_t a[2], vqf_double_t state[], vqf_real_t out[]) {
    if (isnan(state[0])) {
        if (isnan(state[1])) {
            state[1] = 0;
            for(size_t i = 0; i < N; i++) state[2+i] = 0;
        }
        state[1]++;
        for (size_t i = 0; i < N; i++) {
            state[2+i] += x[i];
            out[i] = state[2+i]/state[1];
        }
        if (state[1]*Ts >= tau) {
            for(size_t i = 0; i < N; i++) filterInitialState(out[i], b, a, state+2*i);
        }
        return;
    }
    for (size_t i = 0; i < N; i++) {
        out[i] = filterStep(x[i], b, a, state+2*i);
    }
}

static void matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9]) {
    memset(out, 0, 9*sizeof(vqf_real_t));
    out[0] = out[4] = out[8] = scale;
}

static void matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]) {
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[3] + in1[2]*in2[6]; tmp[1] = in1[0]*in2[1] + in1[1]*in2[4] + in1[2]*in2[7]; tmp[2] = in1[0]*in2[2] + in1[1]*in2[5] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[3] + in1[5]*in2[6]; tmp[4] = in1[3]*in2[1] + in1[4]*in2[4] + in1[5]*in2[7]; tmp[5] = in1[3]*in2[2] + in1[4]*in2[5] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[3] + in1[8]*in2[6]; tmp[7] = in1[6]*in2[1] + in1[7]*in2[4] + in1[8]*in2[7]; tmp[8] = in1[6]*in2[2] + in1[7]*in2[5] + in1[8]*in2[8];
    memcpy(out, tmp, sizeof(tmp));
}

static void matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]) {
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[3]*in2[3] + in1[6]*in2[6]; tmp[1] = in1[0]*in2[1] + in1[3]*in2[4] + in1[6]*in2[7]; tmp[2] = in1[0]*in2[2] + in1[3]*in2[5] + in1[6]*in2[8];
    tmp[3] = in1[1]*in2[0] + in1[4]*in2[3] + in1[7]*in2[6]; tmp[4] = in1[1]*in2[1] + in1[4]*in2[4] + in1[7]*in2[7]; tmp[5] = in1[1]*in2[2] + in1[4]*in2[5] + in1[7]*in2[8];
    tmp[6] = in1[2]*in2[0] + in1[5]*in2[3] + in1[8]*in2[6]; tmp[7] = in1[2]*in2[1] + in1[5]*in2[4] + in1[8]*in2[7]; tmp[8] = in1[2]*in2[2] + in1[5]*in2[5] + in1[8]*in2[8];
    memcpy(out, tmp, sizeof(tmp));
}

static void matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9]) {
    vqf_real_t tmp[9];
    tmp[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2]; tmp[1] = in1[0]*in2[3] + in1[1]*in2[4] + in1[2]*in2[5]; tmp[2] = in1[0]*in2[6] + in1[1]*in2[7] + in1[2]*in2[8];
    tmp[3] = in1[3]*in2[0] + in1[4]*in2[1] + in1[5]*in2[2]; tmp[4] = in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5]; tmp[5] = in1[3]*in2[6] + in1[4]*in2[7] + in1[5]*in2[8];
    tmp[6] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2]; tmp[7] = in1[6]*in2[3] + in1[7]*in2[4] + in1[8]*in2[5]; tmp[8] = in1[6]*in2[6] + in1[7]*in2[7] + in1[8]*in2[8];
    memcpy(out, tmp, sizeof(tmp));
}

static bool matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9]) {
    vqf_double_t A = in[4]*in[8] - in[5]*in[7]; vqf_double_t D = in[2]*in[7] - in[1]*in[8]; vqf_double_t G = in[1]*in[5] - in[2]*in[4];
    vqf_double_t B = in[5]*in[6] - in[3]*in[8]; vqf_double_t E = in[0]*in[8] - in[2]*in[6]; vqf_double_t H = in[2]*in[3] - in[0]*in[5];
    vqf_double_t C = in[3]*in[7] - in[4]*in[6]; vqf_double_t F = in[1]*in[6] - in[0]*in[7]; vqf_double_t I = in[0]*in[4] - in[1]*in[3];
    vqf_double_t det = in[0]*A + in[1]*B + in[2]*C;
    if (det >= -EPS && det <= EPS) {
        memset(out, 0, 9*sizeof(vqf_real_t));
        return false;
    }
    out[0] = A/det; out[1] = D/det; out[2] = G/det; out[3] = B/det; out[4] = E/det; out[5] = H/det; out[6] = C/det; out[7] = F/det; out[8] = I/det;
    return true;
}


void vqf_init(struct VQF_FILTER_t *f, float dt)
{
    // 初始化参数
    f->tauAcc = 3.0f;
    f->motionBiasEstEnabled = true;
    f->restBiasEstEnabled = true;
    f->biasSigmaInit = 0.5f;
    f->biasForgettingTime = 100.0f;
    f->biasClip = 2.0f;
    f->biasSigmaMotion = 0.1f;
    f->biasVerticalForgettingFactor = 0.0001f;
    f->biasSigmaRest = 0.03f;
    f->restMinT = 1.5f;
    f->restFilterTau = 0.5f;
    f->restThGyr = 2.0f;
    f->restThAcc = 0.5f;

    // 时间常数
    f->gyrTs = dt;
    f->accTs = dt;

    // 计算滤波器系数
    filterCoeffs(f->tauAcc, f->accTs, f->accLpB, f->accLpA);
    f->biasP0 = vqf_square(f->biasSigmaInit*100.0f);
    f->biasV = vqf_square(0.1f*100.0f) * f->accTs / f->biasForgettingTime;

    vqf_real_t pMotion = vqf_square(f->biasSigmaMotion*100.0f);
    f->biasMotionW = vqf_square(pMotion) / f->biasV + pMotion;
    f->biasVerticalW = f->biasMotionW / vqf_max(f->biasVerticalForgettingFactor, 1e-10f);

    vqf_real_t pRest = vqf_square(f->biasSigmaRest*100.0f);
    f->biasRestW = vqf_square(pRest) / f->biasV + pRest;

    filterCoeffs(f->restFilterTau, f->gyrTs, f->restGyrLpB, f->restGyrLpA);
    filterCoeffs(f->restFilterTau, f->accTs, f->restAccLpB, f->restAccLpA);

    // 重置状态
    f->gyrQuat[0] = 1; f->gyrQuat[1] = 0; f->gyrQuat[2] = 0; f->gyrQuat[3] = 0;
    f->accQuat[0] = 1; f->accQuat[1] = 0; f->accQuat[2] = 0; f->accQuat[3] = 0;
    f->restDetected = false;
    memset(f->lastAccLp, 0, sizeof(f->lastAccLp));
    memset(f->bias, 0, sizeof(f->bias));
    matrix3SetToScaledIdentity(f->biasP0, f->biasP);

    for (size_t i = 0; i < 6; i++) {
        f->accLpState[i] = NAN;
        f->restGyrLpState[i] = NAN;
        f->restAccLpState[i] = NAN;
    }
    for (size_t i = 0; i < 18; i++) f->motionBiasEstRLpState[i] = NAN;
    for (size_t i = 0; i < 4; i++) f->motionBiasEstBiasLpState[i] = NAN;
    for (size_t i = 0; i < 3; i++) {
        f->restLastGyrLp[i] = NAN;
        f->restLastAccLp[i] = 0.0f;
    }
    f->restLastSquaredDeviations[0] = f->restLastSquaredDeviations[1] = 0.0f;
    f->restT = 0.0f;

    f->pitch = 0; f->roll = 0; f->yaw = 0;
    f->yaw_laps = 0; f->last_yaw = 0; f->YawTotalAngle = 0;
}

void vqf_update(struct VQF_FILTER_t *f, float gx, float gy, float gz, float ax, float ay, float az)
{
    vqf_real_t gyr[3] = {gx, gy, gz};
    vqf_real_t acc[3] = {ax, ay, az};

    /* ----------------- 更新陀螺仪 ----------------- */
    if (f->restBiasEstEnabled) {
        filterVec(gyr, 3, f->restFilterTau, f->gyrTs, f->restGyrLpB, f->restGyrLpA,
                  f->restGyrLpState, f->restLastGyrLp);
        f->restLastSquaredDeviations[0] = vqf_square(gyr[0] - f->restLastGyrLp[0])
                + vqf_square(gyr[1] - f->restLastGyrLp[1]) + vqf_square(gyr[2] - f->restLastGyrLp[2]);

        vqf_real_t biasClip = f->biasClip*(M_PIf/180.0f);
        if (f->restLastSquaredDeviations[0] >= vqf_square(f->restThGyr*(M_PIf/180.0f))
                || fabsf(f->restLastGyrLp[0]) > biasClip || fabsf(f->restLastGyrLp[1]) > biasClip
                || fabsf(f->restLastGyrLp[2]) > biasClip) {
            f->restT = 0.0;
            f->restDetected = false;
        }
    }

    vqf_real_t gyrNoBias[3] = {gyr[0]-f->bias[0], gyr[1]-f->bias[1], gyr[2]-f->bias[2]};
    vqf_real_t gyrNorm = vqf_norm(gyrNoBias, 3);
    vqf_real_t angle = gyrNorm * f->gyrTs;
    if (gyrNorm > EPS) {
        vqf_real_t c = cosf(angle/2);
        vqf_real_t s = sinf(angle/2)/gyrNorm;
        vqf_real_t gyrStepQuat[4] = {c, s*gyrNoBias[0], s*gyrNoBias[1], s*gyrNoBias[2]};
        vqf_quatMultiply(f->gyrQuat, gyrStepQuat, f->gyrQuat);
        vqf_normalize(f->gyrQuat, 4);
    }

    /* ----------------- 更新加速度计 ----------------- */
    if (acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f) return;

    if (f->restBiasEstEnabled) {
        filterVec(acc, 3, f->restFilterTau, f->accTs, f->restAccLpB, f->restAccLpA,
                  f->restAccLpState, f->restLastAccLp);
        f->restLastSquaredDeviations[1] = vqf_square(acc[0] - f->restLastAccLp[0])
                + vqf_square(acc[1] - f->restLastAccLp[1]) + vqf_square(acc[2] - f->restLastAccLp[2]);

        if (f->restLastSquaredDeviations[1] >= vqf_square(f->restThAcc)) {
            f->restT = 0.0;
            f->restDetected = false;
        } else {
            f->restT += f->accTs;
            if (f->restT >= f->restMinT) f->restDetected = true;
        }
    }

    vqf_real_t accEarth[3];
    vqf_quatRotate(f->gyrQuat, acc, accEarth);
    filterVec(accEarth, 3, f->tauAcc, f->accTs, f->accLpB, f->accLpA, f->accLpState, f->lastAccLp);

    vqf_quatRotate(f->accQuat, f->lastAccLp, accEarth);
    vqf_normalize(accEarth, 3);

    vqf_real_t accCorrQuat[4];
    vqf_real_t q_w = sqrtf((accEarth[2]+1)/2);
    if (q_w > 1e-6f) {
        accCorrQuat[0] = q_w; accCorrQuat[1] = 0.5f*accEarth[1]/q_w;
        accCorrQuat[2] = -0.5f*accEarth[0]/q_w; accCorrQuat[3] = 0;
    } else {
        accCorrQuat[0] = 0; accCorrQuat[1] = 1; accCorrQuat[2] = 0; accCorrQuat[3] = 0;
    }
    vqf_quatMultiply(accCorrQuat, f->accQuat, f->accQuat);
    vqf_normalize(f->accQuat, 4);

    f->lastAccCorrAngularRate = acosf(accEarth[2])/f->accTs;

    // 卡尔曼零偏估计 (Kalman Filter Bias Estimation)
    if (f->motionBiasEstEnabled || f->restBiasEstEnabled) {
        vqf_real_t biasClip = f->biasClip*(M_PIf/180.0f);
        vqf_real_t accGyrQuat[4], R[9], biasLp[2];

        vqf_quatMultiply(f->accQuat, f->gyrQuat, accGyrQuat);
        R[0] = 1 - 2*vqf_square(accGyrQuat[2]) - 2*vqf_square(accGyrQuat[3]);
        R[1] = 2*(accGyrQuat[2]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[3]);
        R[2] = 2*(accGyrQuat[0]*accGyrQuat[2] + accGyrQuat[3]*accGyrQuat[1]);
        R[3] = 2*(accGyrQuat[0]*accGyrQuat[3] + accGyrQuat[2]*accGyrQuat[1]);
        R[4] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[3]);
        R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[1]*accGyrQuat[0]);
        R[6] = 2*(accGyrQuat[3]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[2]);
        R[7] = 2*(accGyrQuat[0]*accGyrQuat[1] + accGyrQuat[3]*accGyrQuat[2]);
        R[8] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[2]);

        biasLp[0] = R[0]*f->bias[0] + R[1]*f->bias[1] + R[2]*f->bias[2];
        biasLp[1] = R[3]*f->bias[0] + R[4]*f->bias[1] + R[5]*f->bias[2];

        filterVec(R, 9, f->tauAcc, f->accTs, f->accLpB, f->accLpA, f->motionBiasEstRLpState, R);
        filterVec(biasLp, 2, f->tauAcc, f->accTs, f->accLpB, f->accLpA, f->motionBiasEstBiasLpState, biasLp);

        vqf_real_t w[3], e[3];
        if (f->restDetected && f->restBiasEstEnabled) {
            e[0] = f->restLastGyrLp[0] - f->bias[0]; e[1] = f->restLastGyrLp[1] - f->bias[1]; e[2] = f->restLastGyrLp[2] - f->bias[2];
            matrix3SetToScaledIdentity(1.0, R);
            w[0] = w[1] = w[2] = f->biasRestW;
        } else if (f->motionBiasEstEnabled) {
            e[0] = -accEarth[1]/f->accTs + biasLp[0] - R[0]*f->bias[0] - R[1]*f->bias[1] - R[2]*f->bias[2];
            e[1] = accEarth[0]/f->accTs + biasLp[1] - R[3]*f->bias[0] - R[4]*f->bias[1] - R[5]*f->bias[2];
            e[2] = - R[6]*f->bias[0] - R[7]*f->bias[1] - R[8]*f->bias[2];
            w[0] = f->biasMotionW; w[1] = f->biasMotionW; w[2] = f->biasVerticalW;
        } else {
            w[0] = w[1] = w[2] = -1.0f;
        }

        if (f->biasP[0] < f->biasP0) f->biasP[0] += f->biasV;
        if (f->biasP[4] < f->biasP0) f->biasP[4] += f->biasV;
        if (f->biasP[8] < f->biasP0) f->biasP[8] += f->biasV;

        if (w[0] >= 0) {
            vqf_clip(e, 3, -biasClip, biasClip);
            vqf_real_t K[9];
            matrix3MultiplyTpsSecond(f->biasP, R, K);
            matrix3Multiply(R, K, K);
            K[0] += w[0]; K[4] += w[1]; K[8] += w[2];
            matrix3Inv(K, K);
            matrix3MultiplyTpsFirst(R, K, K);
            matrix3Multiply(f->biasP, K, K);

            f->bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
            f->bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
            f->bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];

            matrix3Multiply(K, R, K);
            matrix3Multiply(K, f->biasP, K);
            for(size_t i = 0; i < 9; i++) f->biasP[i] -= K[i];

            vqf_clip(f->bias, 3, -biasClip, biasClip);
        }
    }
}

void vqf_output(struct VQF_FILTER_t *f)
{
    vqf_quatMultiply(f->accQuat, f->gyrQuat, f->q);

    float q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];

    float sinp = 2.0f * (q0 * q2 - q1 * q3);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    f->pitch = asinf(sinp) * RAD2DEG;
    f->roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    f->yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;

    float yaw_diff = f->yaw - f->last_yaw;
    if (yaw_diff > 180.0f) {
        yaw_diff -= 360.0f;
        f->yaw_laps --;
    }
    else if (yaw_diff < -180.0f) {
        yaw_diff += 360.0f;
        f->yaw_laps ++;
    }
    f->YawTotalAngle += yaw_diff;
    f->last_yaw = f->yaw;
}