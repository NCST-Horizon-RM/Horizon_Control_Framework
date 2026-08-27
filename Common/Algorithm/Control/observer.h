//
// Created by CaoKangqi on 2026/6/19.
//

#ifndef HORIZON_OBSERVER_H
#define HORIZON_OBSERVER_H
#include <stdint.h>
#include "user_lib.h"

/************************* LINEAR DISTURBANCE OBSERVER *************************/
typedef struct __packed
{
    float c[3]; // G(s) = 1/(c2s^2 + c1s + c0)

    float Measure;//观测器输入，通常为系统输出
    float Last_Measure;//上次观测器输入

    float u; //观测器输入，通常为系统控制输入

    float DeadBand;// 扰动输出死区带宽占比

    uint32_t DWT_CNT;
    float dt;

    float LPF_RC; // RC = 1/omegac	一阶低通滤波参数

    float Measure_dot;//观测器输入的微分
    float Measure_ddot;//观测器输入的二阶微分
    float Last_Measure_dot;//上次观测器输入的微分

    uint16_t Measure_dot_OLS_Order;//最小二乘提取信号微分阶数
    Ordinary_Least_Squares_t Measure_dot_OLS;//最小二乘提取信号微分
    uint16_t Measure_ddot_OLS_Order;//最小二乘提取信号二阶微分阶数
    Ordinary_Least_Squares_t Measure_ddot_OLS;//最小二乘提取信号二阶微分

    float Disturbance;//观测到的扰动值
    float Output;//扰动补偿输出
    float Last_Disturbance;//上次观测到的扰动值
    float Max_Disturbance;//扰动输出限幅
} LDOB_t;

void LDOB_Init(
    LDOB_t *ldob,
    float max_d,
    float deadband,
    float *c,
    float lpf_rc,
    uint16_t measure_dot_ols_order,
    uint16_t measure_ddot_ols_order);

float LDOB_Calculate(LDOB_t *ldob, float measure, float u);


typedef struct __packed
{
    float Input;     // 系统输出 y
    float u;         // 控制输入 u

    float b;         // 控制增益估计值
    float wo;        // 观测器带宽

    float l1;        // ESO增益
    float l2;

    float z1;        // 状态估计 x1^
    float z2;        // 扰动估计 x2^

    float last_z1;
    float last_z2;

    float last_dz1;
    float last_dz2;

    uint32_t DWT_CNT;
    float dt;

} LESO_t;

void LESO_Init(LESO_t *leso, float b, float wo);
float LESO_Calculate(LESO_t *leso, float measure, float u);

#endif //HORIZON_OBSERVER_H
