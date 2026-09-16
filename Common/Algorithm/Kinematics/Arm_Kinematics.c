/**
 * @file    Arm_Kinematics.c
 * @brief   七轴机械臂运动学实现（改进 DH / MDH, Craig 参数化）
 * @ref     Craig, Introduction to Robotics, 3rd ed.
 */

#include "Arm_Kinematics.h"
#include "Mat_Fixed.h"

#include <math.h>
#include <string.h>

/* 检查数组中的 NaN / Inf；指针由调用方提前校验。 */
static int arm_all_finite(const float *values, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
    {
        if (!isfinite(values[i]))
        {
            return 0;
        }
    }
    return 1;
}

/* 检查单关节类型和参数，不判断机械限位。 */
static Arm_Status_t arm_validate_link(const Arm_Link_t *link)
{
    if (link == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    if (link->type != ARM_JOINT_R && link->type != ARM_JOINT_P)
    {
        return ARM_STATUS_INVALID_JOINT_TYPE;
    }
    if (!isfinite(link->alpha) || !isfinite(link->a) ||
        !isfinite(link->theta) || !isfinite(link->d) ||
        !isfinite(link->offset) || !isfinite(link->qmin) || !isfinite(link->qmax))
    {
        return ARM_STATUS_NONFINITE_INPUT;
    }
    return ARM_STATUS_OK;
}

/* 检查有效关节配置，不修改模型。 */
Arm_Status_t Arm_Model_Validate(const Arm_Model_t *m)
{
    if (m == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    /* 必须先检查 n，再访问 links，避免错误配置导致越界。 */
    if (m->n == 0 || m->n > ARM_MAX_JOINTS)
    {
        return ARM_STATUS_INVALID_JOINT_COUNT;
    }
    for (uint8_t i = 0; i < m->n; i++)
    {
        Arm_Status_t status = arm_validate_link(&m->links[i]);
        if (status != ARM_STATUS_OK)
        {
            return status;
        }
    }
    return ARM_STATUS_OK;
}

/* 先确认模型有效，再读取前 n 个关节输入。 */
static Arm_Status_t arm_validate_state(const Arm_Model_t *m, const float *q)
{
    if (q == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    Arm_Status_t status = Arm_Model_Validate(m);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }
    return arm_all_finite(q, m->n) ? ARM_STATUS_OK : ARM_STATUS_NONFINITE_INPUT;
}

/* 3 维向量叉乘 a x b -> out */
static void vec3_cross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

/* 3 维向量减法 a - b -> out */
static void vec3_sub(const float a[3], const float b[3], float out[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

/* MDH 单关节变换（解析展开，Craig）：
 * A_i = Trans(x,a) * Rot(x,alpha) * Rot(z,theta) * Trans(z,d)
 *     [ cT      -sT       0     a     ]
 *     [ sT*ca    cT*ca   -sa   -d*sa  ]
 *     [ sT*sa    cT*sa    ca    d*ca  ]
 *     [ 0        0       0     1     ]
 * 输入已校验；失败时可能改写内部临时缓冲区。 */
static Arm_Status_t arm_link_t(const Arm_Link_t *link, float q, float T[16])
{
    float theta, d;

    /* 按输入 q 求位姿；限位由上层处理，避免 FK 与雅可比不一致。 */
    if (link->type == ARM_JOINT_R)
    {
        theta = link->theta + link->offset + q;
        d = link->d;
    }
    else
    {
        theta = link->theta;
        d = link->d + link->offset + q;
    }

    /* 有限输入相加也可能溢出，先检查再计算三角函数。 */
    if (!isfinite(theta) || !isfinite(d))
    {
        return ARM_STATUS_NUMERIC_ERROR;
    }

    float ct = cosf(theta);
    float st = sinf(theta);
    float ca = cosf(link->alpha);
    float sa = sinf(link->alpha);

    T[0] = ct;
    T[1] = -st;
    T[2] = 0.0f;
    T[3] = link->a;

    T[4] = st * ca;
    T[5] = ct * ca;
    T[6] = -sa;
    T[7] = -d * sa;

    T[8] = st * sa;
    T[9] = ct * sa;
    T[10] = ca;
    T[11] = d * ca;

    T[12] = 0.0f;
    T[13] = 0.0f;
    T[14] = 0.0f;
    T[15] = 1.0f;
    return arm_all_finite(T, 16) ? ARM_STATUS_OK : ARM_STATUS_NUMERIC_ERROR;
}

/* 单关节公开接口：先校验，计算成功后才更新输出。 */
Arm_Status_t Arm_Link_T(const Arm_Link_t *link, float q, float T[16])
{
    if (T == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    Arm_Status_t status = arm_validate_link(link);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }
    if (!isfinite(q))
    {
        return ARM_STATUS_NONFINITE_INPUT;
    }

    float result[16];
    status = arm_link_t(link, q, result);
    if (status == ARM_STATUS_OK)
    {
        memcpy(T, result, sizeof(result));
    }
    return status;
}

/* 从基座开始逐节连乘，得到末端相对基座的位姿。 */
Arm_Status_t Arm_FK(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS], float T[16])
{
    if (T == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    Arm_Status_t status = arm_validate_state(m, q);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }

    MATF_DECL(Ti, 4, 4);    /* 当前关节变换 */
    MATF_DECL(Tacc, 4, 4);  /* 已累积的基座变换 */
    MATF_DECL(Tnext, 4, 4); /* 独立乘法输出，避免覆盖输入 */

    MATF_EYE(Tacc);

    for (uint8_t i = 0; i < m->n; i++)
    {
        status = arm_link_t(&m->links[i], q[i], Ti.pData);
        if (status != ARM_STATUS_OK)
        {
            return status;
        }
        if (MATF_MULT(Tnext, Tacc, Ti) != ARM_MATH_SUCCESS || !arm_all_finite(Tnext.pData, 16))
        {
            return ARM_STATUS_NUMERIC_ERROR;
        }
        memcpy(Tacc.pData, Tnext.pData, 16 * sizeof(float));
    }

    memcpy(T, Tacc.pData, 16 * sizeof(float));
    return ARM_STATUS_OK;
}

/* 根据关节轴和末端位置构造速度映射：[v; omega] = J * q_dot。 */
Arm_Status_t Arm_Jacobian(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS],
                          float J[6 * ARM_MAX_JOINTS])
{
    if (J == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    float Te[16]; /* 末端相对基座的变换 */
    float pe[3];  /* 末端在基座系中的位置 */
    float result[6 * ARM_MAX_JOINTS] = {0}; /* 暂存结果，未使用列保持为零 */
    MATF_DECL(Ti, 4, 4);
    MATF_DECL(Tacc, 4, 4);
    MATF_DECL(Tnext, 4, 4);

    /* FK 同时完成模型和 q 校验；失败时不继续访问 m 或 q。 */
    Arm_Status_t status = Arm_FK(m, q, Te);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }
    pe[0] = Te[3];
    pe[1] = Te[7];
    pe[2] = Te[11];

    MATF_EYE(Tacc);

    for (uint8_t i = 0; i < m->n; i++)
    {
        float z[3]; /* 当前关节轴在基座系中的方向 */
        float p[3]; /* 当前关节坐标系原点 */
        float r[3]; /* 关节原点指向末端的向量 */
        float Jv[3];
        float Jw[3];

        /* MDH 的关节轴为 z_i。先递推到 T_0i；p_i 位于该轴上，
         * 后续 Rot(z,theta) / Trans(z,d) 不改变轴的方向或轴线。 */
        status = arm_link_t(&m->links[i], q[i], Ti.pData);
        if (status != ARM_STATUS_OK)
        {
            return status;
        }
        if (MATF_MULT(Tnext, Tacc, Ti) != ARM_MATH_SUCCESS || !arm_all_finite(Tnext.pData, 16))
        {
            return ARM_STATUS_NUMERIC_ERROR;
        }
        memcpy(Tacc.pData, Tnext.pData, 16 * sizeof(float));

        z[0] = Tacc.pData[2];
        z[1] = Tacc.pData[6];
        z[2] = Tacc.pData[10];
        p[0] = Tacc.pData[3];
        p[1] = Tacc.pData[7];
        p[2] = Tacc.pData[11];

        if (m->links[i].type == ARM_JOINT_R)
        {
            /* J_v = z_i x (p_e - p_i)，J_w = z_i */
            vec3_sub(pe, p, r);
            vec3_cross(z, r, Jv);
            Jw[0] = z[0];
            Jw[1] = z[1];
            Jw[2] = z[2];
        }
        else
        {
            /* J_v = z_i，J_w = 0 */
            Jv[0] = z[0];
            Jv[1] = z[1];
            Jv[2] = z[2];
            Jw[0] = 0.0f;
            Jw[1] = 0.0f;
            Jw[2] = 0.0f;
        }

        result[0 * ARM_MAX_JOINTS + i] = Jv[0];
        result[1 * ARM_MAX_JOINTS + i] = Jv[1];
        result[2 * ARM_MAX_JOINTS + i] = Jv[2];
        result[3 * ARM_MAX_JOINTS + i] = Jw[0];
        result[4 * ARM_MAX_JOINTS + i] = Jw[1];
        result[5 * ARM_MAX_JOINTS + i] = Jw[2];
    }
    if (!arm_all_finite(result, 6 * ARM_MAX_JOINTS))
    {
        return ARM_STATUS_NUMERIC_ERROR;
    }
    memcpy(J, result, sizeof(result));
    return ARM_STATUS_OK;
}

/* 从旋转矩阵提取 yaw、pitch、roll，奇异姿态固定 roll 为零。 */
Arm_Status_t Arm_T2RPY(const float T[16], float rpy[3])
{
    if (T == NULL || rpy == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    if (!arm_all_finite(T, 16))
    {
        return ARM_STATUS_NONFINITE_INPUT;
    }
    float result[3];
    float r00 = T[0];
    float r10 = T[4];
    float r20 = T[8];
    float r21 = T[9];
    float r22 = T[10];
    float cp = sqrtf(r00 * r00 + r10 * r10); /* |cos(pitch)|，用于判断奇异姿态 */
    if (!isfinite(cp))
    {
        return ARM_STATUS_NUMERIC_ERROR;
    }

    result[1] = atan2f(-r20, cp); /* pitch 属于 [-pi/2, pi/2] */
    if (cp > 1.0e-6f)
    {
        result[0] = atan2f(r10, r00); /* yaw */
        result[2] = atan2f(r21, r22); /* roll */
    }
    else
    {
        /* pitch 接近 +/-pi/2 时 yaw、roll 不唯一。
         * 固定 roll = 0，由 R01、R11 保留 yaw - roll / yaw + roll，
         * 使输出仍表示原姿态（误差量级由上述单精度阈值限定）。 */
        result[0] = atan2f(-T[1], T[5]);
        result[2] = 0.0f;
    }
    if (!arm_all_finite(result, 3))
    {
        return ARM_STATUS_NUMERIC_ERROR;
    }
    memcpy(rpy, result, sizeof(result));
    return ARM_STATUS_OK;
}

/* 按 Rz(yaw)*Ry(pitch)*Rx(roll) 构造姿态，并填入位置。 */
Arm_Status_t Arm_RPY2T(const float rpy[3], const float p[3], float T[16])
{
    if (rpy == NULL || p == NULL || T == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    if (!arm_all_finite(rpy, 3) || !arm_all_finite(p, 3))
    {
        return ARM_STATUS_NONFINITE_INPUT;
    }
    float result[16];
    float cy = cosf(rpy[0]);
    float sy = sinf(rpy[0]);
    float cp = cosf(rpy[1]);
    float sp = sinf(rpy[1]);
    float cr = cosf(rpy[2]);
    float sr = sinf(rpy[2]);

    /* R = Rz(yaw) * Ry(pitch) * Rx(roll) */
    result[0] = cy * cp;
    result[1] = cy * sp * sr - sy * cr;
    result[2] = cy * sp * cr + sy * sr;
    result[3] = p[0];

    result[4] = sy * cp;
    result[5] = sy * sp * sr + cy * cr;
    result[6] = sy * sp * cr - cy * sr;
    result[7] = p[1];

    result[8] = -sp;
    result[9] = cp * sr;
    result[10] = cp * cr;
    result[11] = p[2];

    result[12] = 0.0f;
    result[13] = 0.0f;
    result[14] = 0.0f;
    result[15] = 1.0f;
    if (!arm_all_finite(result, 16))
    {
        return ARM_STATUS_NUMERIC_ERROR;
    }
    memcpy(T, result, sizeof(result));
    return ARM_STATUS_OK;
}
