/**
 * @file       mahony_filter.c
 * @author     CaoKangqi
 * @brief      Mahony姿态滤波算法实现
 * @date       2026/2/11
 * @version    V1.0
 * @note       核心特性：
 * 1. 基于四元数的互补滤波，避免欧拉角万向锁问题
 * 2. 陀螺仪零偏自学习（静态时自动校准）
 * 3. 积分限幅防止姿态发散，偏航角低通平滑
 * 4. 快速逆平方根优化计算效率
 */
#include "mahony_filter.h"
#include "arm_math.h"
#include "cmsis_os2.h"
#include "Horizon_MATH.h"

struct MAHONY_FILTER_t mahony_filter;

/**
 * @brief 优化的逆平方根：在支持FPU的ARM上，硬件除法和开方极快
 */
static float32_t arm_invSqrt(float32_t x) {
    float32_t out;
    arm_status status = arm_sqrt_f32(x, &out);
    if (status != ARM_MATH_SUCCESS || out == 0.0f) return 0.0f;
    return 1.0f / out;
}

/**
 * @brief 根据四元数更新旋转矩阵
 * @param f 指向MAHONY_FILTER_t结构体的指针，包含四元数和旋转矩阵存储区域
 * @return void
 */
void RotationMatrix_update(struct MAHONY_FILTER_t *f)
{
    float32_t q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];

    // 提前计算平方项，减少重复乘法
    float32_t q1q1 = q1 * q1;
    float32_t q2q2 = q2 * q2;
    float32_t q3q3 = q3 * q3;
    float32_t q0q1 = q0 * q1;
    float32_t q0q2 = q0 * q2;
    float32_t q0q3 = q0 * q3;
    float32_t q1q2 = q1 * q2;
    float32_t q1q3 = q1 * q3;
    float32_t q2q3 = q2 * q3;

    f->rMat[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    f->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    f->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    f->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    f->rMat[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    f->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    f->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    f->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    f->rMat[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

/**
 * @brief Mahony滤波算法核心更新函数 (带加速度计低通滤波版)
 * @param f 指向MAHONY_FILTER_t结构体的指针
 * @param gx, gy, gz 陀螺仪原始数据 (rad/s)
 * @param ax, ay, az 加速度计原始数据 (m/s²)
 * @param dt 采样周期 (s)
 */
void mahony_update(struct MAHONY_FILTER_t *f,
                   float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
{
    f->dt = dt;
    float halfT = 0.5f * f->dt;

    // 加速度计一阶低通滤波
    f->acc_lpf.x = (1.0f - f->alpha) * f->acc_lpf.x + f->alpha * ax;
    f->acc_lpf.y = (1.0f - f->alpha) * f->acc_lpf.y + f->alpha * ay;
    f->acc_lpf.z = (1.0f - f->alpha) * f->acc_lpf.z + f->alpha * az;

    // 计算滤波后的加速度模长
    float32_t acc_sum_sq = f->acc_lpf.x * f->acc_lpf.x +
                           f->acc_lpf.y * f->acc_lpf.y +
                           f->acc_lpf.z * f->acc_lpf.z;
    arm_sqrt_f32(acc_sum_sq, &f->acc_norm);

    // 计算陀螺仪模长
    float32_t gyro_sum_sq = gx * gx + gy * gy + gz * gz;
    float32_t gyro_norm;
    arm_sqrt_f32(gyro_sum_sq, &gyro_norm);

    // 静态零偏学习逻辑
    int is_static = (fabsf(f->acc_norm - 9.81f) < 0.05f) && (gyro_norm < 0.015f);
    if (is_static)
    {
        const float learn_rate = 0.006f;
        f->gyro_bias.x = (1 - learn_rate) * f->gyro_bias.x + learn_rate * gx;
        f->gyro_bias.y = (1 - learn_rate) * f->gyro_bias.y + learn_rate * gy;
        f->gyro_bias.z = (1 - learn_rate) * f->gyro_bias.z + learn_rate * gz;
    }

    // 去除陀螺仪零偏
    gx -= f->gyro_bias.x;
    gy -= f->gyro_bias.y;
    gz -= f->gyro_bias.z;

    // 加速度计有效性判定
    int high_dynamic = (fabsf(f->acc_norm - 9.81f) > 1.5f) || (gyro_norm > 1.0f);

    if (!high_dynamic && acc_sum_sq > 0.000001f)
    {
        // 归一化滤波后的加速度计数据
        float32_t norm = arm_invSqrt(acc_sum_sq);
        float ax_n = f->acc_lpf.x * norm;
        float ay_n = f->acc_lpf.y * norm;
        float az_n = f->acc_lpf.z * norm;

        // 叉乘计算误差向量 (重力在机体系投影 f->rMat[2] 与 加速度计测量向量 的误差)
        // ex = ay_meas * rMat[2][2] - az_meas * rMat[2][1]
        // ey = az_meas * rMat[2][0] - ax_meas * rMat[2][2]
        float ex = ay_n * f->rMat[2][2] - az_n * f->rMat[2][1];
        float ey = az_n * f->rMat[2][0] - ax_n * f->rMat[2][2];

        // 积分项累加
        if (gyro_norm < 0.5f)
        {
            f->exInt += f->Ki * ex * dt;
            f->eyInt += f->Ki * ey * dt;
        }
        else
        {
            // 动态较大时可以选择让积分缓慢衰减，或者保持不变
            f->exInt *= 0.99f;
            f->eyInt *= 0.99f;
        }

        // 注入补偿项到角速度
        gx += f->Kp * ex + f->exInt;
        gy += f->Kp * ey + f->eyInt;
    }

    // 四元数积分更新 (Runge-Kutta 1阶)
    float q0 = f->q[0], q1 = f->q[1], q2 = f->q[2], q3 = f->q[3];
    f->q[0] += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    f->q[1] += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    f->q[2] += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    f->q[3] += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    // 四元数归一化
    float q_norm = arm_invSqrt(f->q[0] * f->q[0] + f->q[1] * f->q[1] + f->q[2] * f->q[2] + f->q[3] * f->q[3]);
    f->q[0] *= q_norm; f->q[1] *= q_norm; f->q[2] *= q_norm; f->q[3] *= q_norm;

    // 更新旋转矩阵供下一周期计算及输出使用
    RotationMatrix_update(f);
}

/**
 * @brief 从旋转矩阵解算并输出姿态角（俯仰/横滚/偏航）
 * @param f 指向MAHONY_FILTER_t结构体的指针，存储旋转矩阵和姿态角结果
 * @return void
 */
void mahony_output(struct MAHONY_FILTER_t *f) {
float r20 = f->rMat[2][0];
    if (r20 > 1.0f) r20 = 1.0f;
    if (r20 < -1.0f) r20 = -1.0f;

    float sqrt_val;
    arm_sqrt_f32(1.0f - r20 * r20, &sqrt_val);
    //使用CORDIC优化计算
    f->pitch = -atan2f(r20, sqrt_val) * RAD2DEG;
    f->roll  = atan2f(f->rMat[2][1], f->rMat[2][2]) * RAD2DEG;
    f->yaw   = atan2f(f->rMat[1][0], f->rMat[0][0]) * RAD2DEG;

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

/**
 * @brief Mahony滤波算法初始化函数
 * @param f 指向MAHONY_FILTER_t结构体的指针，待初始化的算法结构体
 * @param Kp 比例增益（姿态校正参数）
 * @param Ki 积分增益（姿态校正参数）
 * @param dt 算法更新周期（单位：秒）
 * @return void
 */
void mahony_init(struct MAHONY_FILTER_t *f, float Kp, float Ki, float alpha,float dt)
{
    f->Kp = Kp;
    f->Ki = Ki;
    f->alpha = alpha;
    f->dt = dt;

    f->q[0] = 1.0f; f->q[1] = 0.0f; f->q[2] = 0.0f; f->q[3] = 0.0f;

    f->acc_lpf.x = 0; f->acc_lpf.y = 0; f->acc_lpf.z = 0;

    f->gyro_bias.x = 0;
    f->gyro_bias.y = 0;
    f->gyro_bias.z = 0;

    f->pitch = 0;
    f->roll = 0;
    f->yaw = 0;
    f->yaw_laps = 0;
    f->YawTotalAngle = 0;

    f->exInt = f->eyInt = 0;

    RotationMatrix_update(f);
    osDelay(1);
}