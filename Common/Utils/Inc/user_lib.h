//
// Created by CaoKangqi on 2026/1/30.
//

#ifndef HORIZON_USER_LIB_H
#define HORIZON_USER_LIB_H

#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"

enum
{
    CHASSIS_DEBUG = 1,
    GIMBAL_DEBUG,
    INS_DEBUG,
    RC_DEBUG,
    IMU_HEAT_DEBUG,
    SHOOT_DEBUG,
    AIMASSIST_DEBUG,
};

extern uint8_t GlobalDebugMode;

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* math relevant */
/* radian coefficient */
#ifndef RADIAN_COEF
#define RADIAN_COEF 57.295779513f
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif


#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct
{
    float input;        // 输入数据
    float out;          // 输出数据
    float min_value;    // 限幅最小值
    float max_value;    // 限幅最大值
    float frame_period; // 时间间隔
} ramp_function_source_t;

typedef struct __packed
{
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;

/** 快速开方 */
float Sqrt(float x);

/** 斜波函数初始化 */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);
/** 斜波函数计算 */
float ramp_calc(ramp_function_source_t *ramp_source_type, float input);

/** 绝对限制（对称限幅） */
float abs_limit(float num, float Limit);
/** 判断符号位 */
float sign(float value);
/** 浮点死区 */
float float_deadband(float Value, float minValue, float maxValue);
/** int16 死区 */
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
/** int16_t 限幅 */
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
/** 循环限幅 */
float loop_float_constrain(float Input, float minValue, float maxValue);
/** 角度 ° 限幅到 -180 ~ 180 */
float theta_format(float Ang);

/** 弧度格式化为 -PI ~ PI */
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

/** 最小二乘法初始化 */
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
/** 最小二乘法拟合 */
void OLS_Update(Ordinary_Least_Squares_t *OLS, float deltax, float y);
/** 最小二乘法提取信号微分 */
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
/** 最小二乘法平滑信号 */
float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
/** 获取最小二乘法信号微分 */
float Get_OLS_Derivative(Ordinary_Least_Squares_t *OLS);
/** 获取最小二乘法平滑信号 */
float Get_OLS_Smooth(Ordinary_Least_Squares_t *OLS);

#endif //HORIZON_USER_LIB_H
