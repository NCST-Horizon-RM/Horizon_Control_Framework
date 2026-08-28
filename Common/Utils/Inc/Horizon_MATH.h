//
// Created by CaoKangqi on 2026/1/25.
//

#ifndef HORIZON_MATH_H
#define HORIZON_MATH_H

#include <stdint.h>

/** 计算 float 绝对值 */
float MATH_ABS_float(float DATA);
/** 字节流转浮点数 */
float Bytes_To_Float(const uint8_t *data);
/** 浮点数转字节流 */
void Float_To_Bytes(float f, uint8_t *data);
/** float 限幅 */
float MATH_Limit_float(float DATA, float MIN, float MAX);
/** int16_t 限幅 */
int16_t MATH_Limit_int16(int16_t DATA, int16_t MIN, int16_t MAX);
/** 置位/复位单个比特位 */
void MATH_SETBIT(unsigned char *byte, int position, int value);
/** float 快速平方根倒数 */
float MATH_INV_SQRT_float(float DATA);
/** 十六进制转浮点数 */
float Hex_To_Float(uint32_t *Byte, int num);
/** 浮点数转十六进制 */
uint32_t Float_To_Hex(float HEX);
/** 浮点数线性映射为无符号整数 */
int float_to_uint(float x_float, float x_min, float x_max, int bits);
/** 无符号整数线性映射为浮点数 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);
/** 一阶滤波器 */
int16_t OneFilter1(int16_t now, int16_t last, float thresholdValue);
/** 角度归一化到 [-π, π]（弧度） */
float normalize_to_pi(float angle);

#endif //HORIZON_MATH_H
