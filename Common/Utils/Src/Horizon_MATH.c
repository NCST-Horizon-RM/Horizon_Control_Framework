//
// Created by CaoKangqi on 2026/1/25.
//
#include "Horizon_MATH.h"
#include "All_define.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/**
 * @brief   计算 float 类型数据的绝对值（通过清除符号位实现）
 * @param   DATA 需要计算绝对值的 float 数据
 * @return  绝对值
 */
float MATH_ABS_float(float DATA)
{
    uint32_t temp;
    memcpy(&temp, &DATA, 4);
    temp &= 0x7FFFFFFF;
    float result;
    memcpy(&result, &temp, 4);
    return result;
}

/**
 * @brief   float 限幅
 * @param   DATA 需要限幅的 float 数据
 * @param   MIN  下限值
 * @param   MAX  上限值
 * @return  限幅后的值
 */
float MATH_Limit_float(float DATA, float MIN, float MAX)
{
    return (DATA > MAX) ? MAX : ((DATA < MIN) ? MIN : DATA);
}

/**
 * @brief   字节流转浮点数
 * @param   data 待转换的字节流指针
 * @return  转换后的浮点数
 */
float Bytes_To_Float(const uint8_t *data)
{
    float f;
    memcpy(&f, data, 4);
    return f;
}

/**
 * @brief   浮点数转字节流
 * @param   f    待转换的浮点数
 * @param   data 存放结果的字节流指针
 */
void Float_To_Bytes(float f, uint8_t *data)
{
    memcpy(data, &f, 4);
}

/**
 * @brief   int16_t 限幅
 * @param   DATA 需要限幅的 int16_t 数据
 * @param   MIN  下限值
 * @param   MAX  上限值
 * @return  限幅后的值
 */
int16_t MATH_Limit_int16(int16_t DATA, int16_t MIN, int16_t MAX)
{
    return (DATA > MAX) ? MAX : ((DATA < MIN) ? MIN : DATA);
}

/**
 * @brief   置位/复位单个比特位
 * @param   byte     待操作字节的指针
 * @param   position 要设置的位所在位置（0 到 7）
 * @param   value    要设置的值（1=置 1，0=置 0）
 */
void MATH_SETBIT(unsigned char *byte, int position, int value)
{
    unsigned char mask = 1 << position; // 生成一个只有指定位置为 1 的掩码
    if (value)
    {
        *byte |= mask; // 将指定位置设置为 1
    }
    else
    {
        *byte &= ~mask; // 将指定位置设置为 0
    }
}

/**
 * @brief   快速计算 float 平方根倒数（Quake 算法）
 * @param   x 需要计算平方根倒数的 float 数据
 * @return  平方根倒数
 */
float MATH_INV_SQRT_float(float x)
{
    uint32_t i;
    memcpy(&i, &x, 4); // 将 float 位拷贝给 uint32_t
    i = 0x5f3759df - (i >> 1);
    memcpy(&x, &i, 4); // 再拷贝回来
    return x * (1.5f - 0.5f * x * x * x);
}

/**
 * @brief   十六进制到浮点数
 * @param   Byte 指向 uint32_t 数据的指针
 * @param   num  未使用
 * @return  转换后的浮点数
 */
float Hex_To_Float(uint32_t *Byte, int num)
{
    return *((float *)Byte);
}

/**
 * @brief   浮点数到十六进制（float 位模式转 uint32_t）
 * @param   HEX 待转换的浮点数
 * @return  浮点数位模式对应的 uint32_t
 */
uint32_t FloatTohex(float HEX)
{
    uint32_t result;
    memcpy(&result, &HEX, 4);
    return result;
}

/**
 * @brief   浮点数转换为无符号整数
 * @param   x_float 待转换的浮点数
 * @param   x_min   范围最小值
 * @param   x_max   范围最大值
 * @param   bits    目标无符号整数的位数
 * @return  无符号整数结果
 * @details 将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief   无符号整数转换为浮点数
 * @param   x_int  待转换的无符号整数
 * @param   x_min  范围最小值
 * @param   x_max  范围最大值
 * @param   bits   无符号整数的位数
 * @return  浮点数结果
 * @details 将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief   一阶滤波器，适用于电机速度等物理量的平滑处理
 * @param   now            当前原始值
 * @param   last           上一次滤波后的值
 * @param   thresholdValue 突变抑制阈值，超过该值则认为是异常突变，进行特殊处理
 * @return  滤波后的值
 * @note    当输入变化超过阈值时，输出将更倾向于上一次的值，以抑制突变；否则正常进行一阶滤波
 */
int16_t OneFilter1(int16_t now, int16_t last, float thresholdValue)
{
    const float alpha = 0.8f;
    if (abs(now - last) >= thresholdValue)
        return (int16_t)(now * 0.2f + last * 0.8f); // 突变抑制
    else
        return (int16_t)(now * alpha + last * (1.0f - alpha));
}

/**
 * @brief   将角度归一化到 [-π, π] 范围内
 * @param   angle 输入角度（弧度）
 * @return  归一化后的角度（弧度）
 */
float normalize_to_pi(float angle)
{
    angle = fmodf(angle, 2.0f * PI);
    if (angle > PI)
        angle -= 2.0f * PI;
    if (angle < -PI)
        angle += 2.0f * PI;
    return angle;
}
