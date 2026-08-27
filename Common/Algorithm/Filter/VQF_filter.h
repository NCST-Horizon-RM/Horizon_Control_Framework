/**
 * @file vqf_filter.h
 * @brief VQF姿态滤波算法头文件
 */

#ifndef HORIZON_VQF_FILTER_H
#define HORIZON_VQF_FILTER_H

#include <stdbool.h>
#include <stdint.h>
#include "All_define.h"

typedef float vqf_real_t;
typedef double vqf_double_t;

/**
 * @brief VQF滤波器结构体
 */
struct VQF_FILTER_t {
    /* ================= 参数 (Params) ================= */
    vqf_real_t tauAcc;
    bool motionBiasEstEnabled;
    bool restBiasEstEnabled;
    vqf_real_t biasSigmaInit;
    vqf_real_t biasForgettingTime;
    vqf_real_t biasClip;
    vqf_real_t biasSigmaMotion;
    vqf_real_t biasVerticalForgettingFactor;
    vqf_real_t biasSigmaRest;
    vqf_real_t restMinT;
    vqf_real_t restFilterTau;
    vqf_real_t restThGyr;
    vqf_real_t restThAcc;

    /* ================= 系数 (Coeffs) ================= */
    vqf_real_t gyrTs;
    vqf_real_t accTs;
    vqf_double_t accLpB[3];
    vqf_double_t accLpA[2];
    vqf_real_t biasP0;
    vqf_real_t biasV;
    vqf_real_t biasMotionW;
    vqf_real_t biasVerticalW;
    vqf_real_t biasRestW;
    vqf_double_t restGyrLpB[3];
    vqf_double_t restGyrLpA[2];
    vqf_double_t restAccLpB[3];
    vqf_double_t restAccLpA[2];

    /* ================= 状态 (State) ================= */
    vqf_real_t gyrQuat[4];
    vqf_real_t accQuat[4];
    bool restDetected;
    vqf_real_t lastAccLp[3];
    vqf_double_t accLpState[6];
    vqf_real_t lastAccCorrAngularRate;
    vqf_real_t bias[3];
    vqf_real_t biasP[9];
    vqf_double_t motionBiasEstRLpState[18];
    vqf_double_t motionBiasEstBiasLpState[4];
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    vqf_double_t restGyrLpState[6];
    vqf_real_t restLastAccLp[3];
    vqf_double_t restAccLpState[6];

    /* ================= 输出 (Outputs) ================= */
    float q[4];
    float pitch, roll, yaw;     // 俯仰/横滚/偏航角
    float yaw_laps;
    float last_yaw, YawTotalAngle; // 累积偏航角
    float rMat[3][3];           // 旋转矩阵
};

extern struct VQF_FILTER_t vqf_filter;

void vqf_init(struct VQF_FILTER_t *f, float dt);
void vqf_update(struct VQF_FILTER_t *f, float gx, float gy, float gz, float ax, float ay, float az);
void vqf_output(struct VQF_FILTER_t *f);

#endif //HORIZON_VQF_FILTER_H