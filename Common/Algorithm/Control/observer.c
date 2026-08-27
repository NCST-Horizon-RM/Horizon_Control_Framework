//
// Created by CaoKangqi on 2026/6/19.
//
#include "observer.h"
#include "Horizon_MATH.h"
#include "BSP_DWT.h"
#include "Classic_Control.h"

/*************************LINEAR DISTURBANCE OBSERVER *************************/
void LDOB_Init(
    LDOB_t *ldob,//线性扰动观测器结构体
    float max_d,//扰动观测器输出限幅
    float deadband,//扰动观测器输出死区带宽占比
    float *c,
    float lpf_rc,//扰动观测器低通滤波参数
    uint16_t measure_dot_ols_order,//最小二乘提取信号微分阶数
    uint16_t measure_ddot_ols_order)
{
    ldob->Max_Disturbance = max_d;

    ldob->DeadBand = deadband;

    // 设置线性扰动观测器参数 详见LDOB结构体定义
    // set parameters of linear disturbance observer (see struct definition)
    if (c != NULL && ldob != NULL)
    {
        ldob->c[0] = c[0];
        ldob->c[1] = c[1];
        ldob->c[2] = c[2];
    }
    else
    {
        ldob->c[0] = 0;
        ldob->c[1] = 0;
        ldob->c[2] = 0;
        ldob->Max_Disturbance = 0;
    }

    // 设置Q(s)带宽  Q(s)选用一阶惯性环节
    // set bandwidth of Q(s)    Q(s) is chosen as a first-order low-pass form
    ldob->LPF_RC = lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    ldob->Measure_dot_OLS_Order = measure_dot_ols_order;
    ldob->Measure_ddot_OLS_Order = measure_ddot_ols_order;
    if (measure_dot_ols_order > 2)
        OLS_Init(&ldob->Measure_dot_OLS, measure_dot_ols_order);
    if (measure_ddot_ols_order > 2)
        OLS_Init(&ldob->Measure_ddot_OLS, measure_ddot_ols_order);

    ldob->DWT_CNT = DWT->CYCCNT;

    ldob->Disturbance = 0;
}

float LDOB_Calculate(LDOB_t *ldob, float measure, float u)
{
    ldob->dt = 1; // 离散控制：单位时间固定为 1（差分形式）

    ldob->Measure = measure;

    ldob->u = u;

    // 计算一阶导数
    // calculate first derivative
    if (ldob->Measure_dot_OLS_Order > 2)
        ldob->Measure_dot = OLS_Derivative(&ldob->Measure_dot_OLS, ldob->dt, ldob->Measure);
    else
        ldob->Measure_dot = (ldob->Measure - ldob->Last_Measure) / ldob->dt;

    // 计算二阶导数
    // calculate second derivative
    if (ldob->Measure_ddot_OLS_Order > 2)
        ldob->Measure_ddot = OLS_Derivative(&ldob->Measure_ddot_OLS, ldob->dt, ldob->Measure_dot);
    else
        ldob->Measure_ddot = (ldob->Measure_dot - ldob->Last_Measure_dot) / ldob->dt;

    // 估计总扰动
    // estimate external disturbances and internal disturbances caused by model uncertainties
    ldob->Disturbance = ldob->c[0] * ldob->Measure + ldob->c[1] * ldob->Measure_dot + ldob->c[2] * ldob->Measure_ddot - ldob->u;
    ldob->Disturbance = ldob->Disturbance * ldob->dt / (ldob->LPF_RC + ldob->dt) +
                        ldob->Last_Disturbance * ldob->LPF_RC / (ldob->LPF_RC + ldob->dt);

    ldob->Disturbance = MATH_Limit_float(ldob->Disturbance, -ldob->Max_Disturbance, ldob->Max_Disturbance);

    // 扰动输出死区
    // deadband of disturbance output
    if (abs(ldob->Disturbance) > ldob->DeadBand * ldob->Max_Disturbance)
        ldob->Output = ldob->Disturbance;
    else
        ldob->Output = 0;

    ldob->Last_Measure = ldob->Measure;
    ldob->Last_Measure_dot = ldob->Measure_dot;
    ldob->Last_Disturbance = ldob->Disturbance;

    return ldob->Output;
}

void LESO_Init(LESO_t *leso, float b, float wo)
{
    leso->b = b;
    leso->wo = wo;

    leso->l1 = 2.0f * wo;
    leso->l2 = wo * wo;

    leso->z1 = 0.0f;
    leso->z2 = 0.0f;

    leso->last_dz1 = 0.0f;
    leso->last_dz2 = 0.0f;

    leso->DWT_CNT = DWT->CYCCNT;
}

float LESO_Calculate(LESO_t *leso, float measure, float u)
{
    leso->dt = 1; // 离散控制：单位时间固定为 1（差分形式）

    leso->Input = measure;
    leso->u = u;

    // 估计状态
    leso->z1 += (leso->z2 + leso->b * leso->u - leso->l1 * (leso->z1 - leso->Input)) * leso->dt;
    leso->z2 += (-leso->l2 * (leso->z1 - leso->Input)) * leso->dt;

    // 输出扰动估计
    return leso->z2;
}