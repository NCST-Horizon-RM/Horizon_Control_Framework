/**
 * @file    Mat_Fixed.h
 * @brief   固定尺寸 float 矩阵运算封装（基于 CMSIS-DSP arm_math）
 *          编译期定尺寸、栈上存储，运算直接映射 arm_mat_*_f32 优化实现。
 * @note    本头文件只提供运算，不持有全局状态。
 */

#ifndef HORIZON_MAT_FIXED_H
#define HORIZON_MAT_FIXED_H

#include "dsp/matrix_functions.h"

/* 声明固定尺寸矩阵：行主序缓冲区 + CMSIS 描述符。
 * 缓冲区未初始化，使用前须赋值或调用 MATF_EYE / MATF_ZEROS。 */
#define MATF_DECL(name, rows, cols)                          \
    float name##_data[(rows) * (cols)];                      \
    arm_matrix_instance_f32 name = {(rows), (cols), name##_data}

/* 访问第 r 行、第 c 列（从 0 开始），不检查下标。 */
#define MATF_AT(name, r, c) ((name).pData[(r) * (name).numCols + (c)])

/* 运算宏返回 arm_status，尺寸须匹配；乘法和转置使用独立输出缓冲区。 */
#define MATF_MULT(dst, a, b)    arm_mat_mult_f32(&(a), &(b), &(dst))
#define MATF_ADD(dst, a, b)     arm_mat_add_f32(&(a), &(b), &(dst))
#define MATF_SUB(dst, a, b)     arm_mat_sub_f32(&(a), &(b), &(dst))
#define MATF_SCALE(dst, src, s) arm_mat_scale_f32(&(src), (s), &(dst)) /* 乘标量 s */
#define MATF_TRANS(dst, src)    arm_mat_trans_f32(&(src), &(dst))    /* 转置 */
/* 求逆会原地改写 src（成功后变为单位阵），
 * 需要保留原矩阵时请先复制到临时矩阵再求逆。 */
#define MATF_INV(dst, src)      arm_mat_inverse_f32(&(src), &(dst))
/* Cholesky 分解：src = L*L^T，dst 仅下三角有效。 */
#define MATF_CHOLESKY(dst, src) arm_mat_cholesky_f32(&(src), &(dst))

/* 对角线填 1，其余填 0；方阵得到单位阵。 */
#define MATF_EYE(name)                                          \
    do                                                          \
    {                                                           \
        uint16_t _row, _col;                                    \
        for (_row = 0; _row < (name).numRows; _row++)           \
        {                                                       \
            for (_col = 0; _col < (name).numCols; _col++)       \
            {                                                   \
                MATF_AT(name, _row, _col) =                     \
                    (_row == _col) ? 1.0f : 0.0f;               \
            }                                                   \
        }                                                       \
    } while (0)

/* 将全部元素清零。 */
#define MATF_ZEROS(name)                                        \
    do                                                          \
    {                                                           \
        uint16_t _idx;                                          \
        for (_idx = 0; _idx < (name).numRows * (name).numCols; _idx++) \
        {                                                       \
            (name).pData[_idx] = 0.0f;                          \
        }                                                       \
    } while (0)

/**
 * @brief 解对称正定(SPD)方程 A*x = b：Cholesky 分解 + 前代/回代
 * @param x 输出解向量 (n x 1)
 * @param A 系数矩阵 (n x n, SPD)
 * @param b 右端向量 (n x 1)
 * @param L 临时矩阵 (n x n)，存放 Cholesky 下三角 L
 * @param z 临时向量 (n x 1)
 * @return ARM_MATH_SUCCESS 或分解、尺寸、奇异错误。
 * @note 调用方保证指针和尺寸有效，各缓冲区独立；仅成功时使用 x。
 *       失败时 x、L、z 可能已部分更新；L 仅下三角有效。
 */
static inline arm_status matf_solve_spd(arm_matrix_instance_f32 *x,
                                        const arm_matrix_instance_f32 *A,
                                        const arm_matrix_instance_f32 *b,
                                        arm_matrix_instance_f32 *L,
                                        arm_matrix_instance_f32 *z)
{
    uint16_t n = A->numRows;
    float *bd = b->pData;
    float *Ld = L->pData;
    float *zd = z->pData;
    float *xd = x->pData;
    arm_status s;

    /* A = L*L^T，将原方程拆成两次三角方程求解。 */
    s = arm_mat_cholesky_f32(A, L);
    if (s != ARM_MATH_SUCCESS)
    {
        return s;
    }

    /* 前代：L*z = b */
    for (uint16_t i = 0; i < n; i++)
    {
        float sum = bd[i];
        for (uint16_t j = 0; j < i; j++)
        {
            sum -= Ld[i * n + j] * zd[j];
        }
        if (Ld[i * n + i] == 0.0f)
        {
            return ARM_MATH_SINGULAR;
        }
        zd[i] = sum / Ld[i * n + i];
    }

    /* 回代：L^T*x = z */
    for (int16_t i = (int16_t)n - 1; i >= 0; i--)
    {
        float sum = zd[i];
        for (uint16_t j = (uint16_t)(i + 1); j < n; j++)
        {
            sum -= Ld[j * n + i] * xd[j];
        }
        if (Ld[i * n + i] == 0.0f)
        {
            return ARM_MATH_SINGULAR;
        }
        xd[i] = sum / Ld[i * n + i];
    }

    return ARM_MATH_SUCCESS;
}

#endif /* HORIZON_MAT_FIXED_H */
