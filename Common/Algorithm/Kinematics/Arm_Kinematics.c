/**
 * @file    Arm_Kinematics.c
 * @brief   七轴机械臂运动学实现（改进 DH / MDH, Craig 参数化）
 * @ref     Craig, Introduction to Robotics, 3rd ed.
 */

#include "Arm_Kinematics.h"
#include "Mat_Fixed.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* 运行模型的私有布局。通过 memcpy 读写不透明存储，避免对齐和严格别名问题。 */
#define ARM_PREPARED_TAG UINT32_C(0x41524D01)

typedef struct
{
    uint32_t tag;
    uint8_t n;
    uint8_t has_tool;
    uint8_t types[ARM_MAX_JOINTS];
} Arm_PreparedHeader_t;

typedef struct
{
    float a;
    float bias; /* 转动关节：theta + offset；移动关节：d + offset */
    float d;    /* 转动关节的固定 d */
    float ca;
    float sa;
    float ct;   /* 移动关节的固定 cos(theta) */
    float st;   /* 移动关节的固定 sin(theta) */
} Arm_PreparedLink_t;

typedef struct
{
    Arm_PreparedHeader_t header;
    Arm_PreparedLink_t links[ARM_MAX_JOINTS];
    float T_world_base[16];
    float T_n_tcp[16];
} Arm_PreparedData_t;

_Static_assert(sizeof(Arm_PreparedData_t) <= sizeof(Arm_PreparedModel_t),
               "Arm_PreparedModel_t storage is too small");

/* 两种模型共用递推和雅可比；指针仅在本次调用期间有效，不作为跨调用缓存。 */
typedef struct
{
    uint8_t n;
    uint8_t types[ARM_MAX_JOINTS];
    const Arm_Link_t *raw_links;
    const uint8_t *prepared_links;
    const void *T_world_base;
    const void *T_n_tcp;
} Arm_Chain_t;

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

/* 可选安装 / 工具变换必须是刚体变换，缩放或反射会破坏几何雅可比的含义。 */
static Arm_Status_t arm_validate_transform(const float *T)
{
    if (T == NULL)
    {
        return ARM_STATUS_OK;
    }
    if (!arm_all_finite(T, 16))
    {
        return ARM_STATUS_NONFINITE_INPUT;
    }
    if (T[12] != 0.0f || T[13] != 0.0f || T[14] != 0.0f || T[15] != 1.0f)
    {
        return ARM_STATUS_INVALID_TRANSFORM;
    }
    const float tolerance = 1.0e-4f;
    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = i; j < 3; j++)
        {
            float dot = T[i] * T[j] + T[4 + i] * T[4 + j] + T[8 + i] * T[8 + j];
            if (!isfinite(dot) || fabsf(dot - (i == j ? 1.0f : 0.0f)) > tolerance)
            {
                return ARM_STATUS_INVALID_TRANSFORM;
            }
        }
    }
    float det = T[0] * (T[5] * T[10] - T[6] * T[9])
              - T[1] * (T[4] * T[10] - T[6] * T[8])
              + T[2] * (T[4] * T[9] - T[5] * T[8]);
    return fabsf(det - 1.0f) <= tolerance ? ARM_STATUS_OK : ARM_STATUS_INVALID_TRANSFORM;
}

/* 检查有效关节和可选坐标变换，不修改模型。 */
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
    Arm_Status_t status = arm_validate_transform(m->T_world_base);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }
    return arm_validate_transform(m->T_n_tcp);
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
static Arm_Status_t arm_mdh_t(float a, float d, float ct, float st,
                              float ca, float sa, float T[16])
{
    T[0] = ct;
    T[1] = -st;
    T[2] = 0.0f;
    T[3] = a;

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

/* 原始模型路径：输入已完整校验，按当前参数计算固定角度。 */
static Arm_Status_t arm_link_t(const Arm_Link_t *link, float q, float T[16])
{
    float theta, d;
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
    if (!isfinite(theta) || !isfinite(d))
    {
        return ARM_STATUS_NUMERIC_ERROR;
    }
    return arm_mdh_t(link->a, d, cosf(theta), sinf(theta),
                     cosf(link->alpha), sinf(link->alpha), T);
}

/* 运行模型路径：仅转动关节的 theta 随 q 变化，移动关节无需计算三角函数。 */
static Arm_Status_t arm_prepared_link_t(const Arm_PreparedLink_t *link, uint8_t type,
                                        float q, float T[16])
{
    float variable = link->bias + q;
    if (!isfinite(variable))
    {
        return ARM_STATUS_NUMERIC_ERROR;
    }
    if (type == ARM_JOINT_R)
    {
        return arm_mdh_t(link->a, link->d, cosf(variable), sinf(variable), link->ca, link->sa, T);
    }
    return arm_mdh_t(link->a, variable, link->ct, link->st, link->ca, link->sa, T);
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

/* 装入可选变换；未配置时使用单位阵。 */
static void arm_load_transform(const void *src, float T[16])
{
    if (src != NULL)
    {
        memcpy(T, src, 16 * sizeof(float));
    }
    else
    {
        memset(T, 0, 16 * sizeof(float));
        T[0] = T[5] = T[10] = T[15] = 1.0f;
    }
}

/* 仅准备阶段访问和校验原始配置；成功后发布不含外部指针的运行模型。 */
Arm_Status_t Arm_Model_Prepare(const Arm_Model_t *m, Arm_PreparedModel_t *prepared)
{
    if (prepared == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    Arm_Status_t status = Arm_Model_Validate(m);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }

    Arm_PreparedData_t result = {0};
    result.header.n = m->n;
    result.header.has_tool = m->T_n_tcp != NULL;
    for (uint8_t i = 0; i < m->n; i++)
    {
        const Arm_Link_t *src = &m->links[i];
        Arm_PreparedLink_t *dst = &result.links[i];
        result.header.types[i] = src->type;
        dst->a = src->a;
        dst->bias = (src->type == ARM_JOINT_R ? src->theta : src->d) + src->offset;
        dst->d = src->d;
        dst->ca = cosf(src->alpha);
        dst->sa = sinf(src->alpha);
        if (src->type == ARM_JOINT_P)
        {
            dst->ct = cosf(src->theta);
            dst->st = sinf(src->theta);
        }
        if (!isfinite(dst->bias) || !isfinite(dst->ca) || !isfinite(dst->sa) ||
            !isfinite(dst->ct) || !isfinite(dst->st))
        {
            return ARM_STATUS_NUMERIC_ERROR;
        }
    }
    arm_load_transform(m->T_world_base, result.T_world_base);
    arm_load_transform(m->T_n_tcp, result.T_n_tcp);
    result.header.tag = ARM_PREPARED_TAG;
    memset(prepared, 0, sizeof(*prepared));
    memcpy(prepared->_opaque, &result, sizeof(result));
    return ARM_STATUS_OK;
}

/* 世界系 -> 模型基座 -> MDH 链 -> TCP。输入已校验；可选保存各关节累计位姿。
 * T 仅成功时写回；T_world_joint 是内部临时缓冲区，失败时可能已部分更新。 */
static Arm_Status_t arm_forward(const Arm_Chain_t *chain, const float *q, float T[16],
                                 float T_world_joint[ARM_MAX_JOINTS][16])
{
    MATF_DECL(Ti, 4, 4);    /* 当前关节变换 */
    MATF_DECL(Tacc, 4, 4);  /* 已累积的世界系变换 */
    MATF_DECL(Tnext, 4, 4); /* 独立乘法输出，避免覆盖输入 */

    arm_load_transform(chain->T_world_base, Tacc.pData);

    for (uint8_t i = 0; i < chain->n; i++)
    {
        Arm_Status_t status;
        if (chain->raw_links != NULL)
        {
            status = arm_link_t(&chain->raw_links[i], q[i], Ti.pData);
        }
        else
        {
            Arm_PreparedLink_t link;
            memcpy(&link, chain->prepared_links + i * sizeof(link), sizeof(link));
            status = arm_prepared_link_t(&link, chain->types[i], q[i], Ti.pData);
        }
        if (status != ARM_STATUS_OK)
        {
            return status;
        }
        if (MATF_MULT(Tnext, Tacc, Ti) != ARM_MATH_SUCCESS || !arm_all_finite(Tnext.pData, 16))
        {
            return ARM_STATUS_NUMERIC_ERROR;
        }
        memcpy(Tacc.pData, Tnext.pData, 16 * sizeof(float));
        if (T_world_joint != NULL)
        {
            memcpy(T_world_joint[i], Tacc.pData, 16 * sizeof(float));
        }
    }

    if (chain->T_n_tcp != NULL)
    {
        memcpy(Ti.pData, chain->T_n_tcp, 16 * sizeof(float));
        if (MATF_MULT(Tnext, Tacc, Ti) != ARM_MATH_SUCCESS || !arm_all_finite(Tnext.pData, 16))
        {
            return ARM_STATUS_NUMERIC_ERROR;
        }
        memcpy(Tacc.pData, Tnext.pData, 16 * sizeof(float));
    }
    memcpy(T, Tacc.pData, 16 * sizeof(float));
    return ARM_STATUS_OK;
}

/* 直接使用同次递推的关节位姿和 TCP 位置；不再计算单关节变换或连乘。
 * 输入已校验，J 是内部临时缓冲区，失败时可能已部分更新。 */
static Arm_Status_t arm_jacobian_from_frames(const Arm_Chain_t *chain, const Arm_Frames_t *frames,
                                             const float T[16], float J[6 * ARM_MAX_JOINTS])
{
    float pe[3] = {T[3], T[7], T[11]};
    memset(J, 0, 6 * ARM_MAX_JOINTS * sizeof(float));

    for (uint8_t i = 0; i < chain->n; i++)
    {
        const float *Ti = frames->T_world_joint[i];
        float z[3]; /* 当前关节轴在世界系中的方向 */
        float p[3]; /* 当前关节坐标系原点在世界系中的位置 */
        float r[3]; /* 关节原点指向 TCP 的向量 */
        float Jv[3];
        float Jw[3];

        /* MDH 的关节轴为 z_i，p_i 位于该轴上，
         * 后续 Rot(z,theta) / Trans(z,d) 不改变轴的方向或轴线。 */
        z[0] = Ti[2];
        z[1] = Ti[6];
        z[2] = Ti[10];
        p[0] = Ti[3];
        p[1] = Ti[7];
        p[2] = Ti[11];

        if (chain->types[i] == ARM_JOINT_R)
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

        J[0 * ARM_MAX_JOINTS + i] = Jv[0];
        J[1 * ARM_MAX_JOINTS + i] = Jv[1];
        J[2 * ARM_MAX_JOINTS + i] = Jv[2];
        J[3 * ARM_MAX_JOINTS + i] = Jw[0];
        J[4 * ARM_MAX_JOINTS + i] = Jw[1];
        J[5 * ARM_MAX_JOINTS + i] = Jw[2];
    }
    return arm_all_finite(J, 6 * ARM_MAX_JOINTS) ? ARM_STATUS_OK : ARM_STATUS_NUMERIC_ERROR;
}

/* 带中间结果的求值路径。临时结果留在栈上，全部成功后再发布给调用方。 */
static Arm_Status_t arm_evaluate_with_frames(const Arm_Chain_t *chain, const float *q,
                                            float *T, float *J, Arm_Frames_t *frames)
{
    Arm_Frames_t frame_result = {0};
    float tcp_result[16];
    float jacobian_result[6 * ARM_MAX_JOINTS];
    Arm_Status_t status = arm_forward(chain, q, tcp_result, frame_result.T_world_joint);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }
    frame_result.n = chain->n;
    if (J != NULL)
    {
        status = arm_jacobian_from_frames(chain, &frame_result, tcp_result, jacobian_result);
        if (status != ARM_STATUS_OK)
        {
            return status;
        }
    }

    if (T != NULL)
    {
        memcpy(T, tcp_result, sizeof(tcp_result));
    }
    if (J != NULL)
    {
        memcpy(J, jacobian_result, sizeof(jacobian_result));
    }
    if (frames != NULL)
    {
        *frames = frame_result;
    }
    return ARM_STATUS_OK;
}

/* 两种模型在各自完成校验后共用输出路径。 */
static Arm_Status_t arm_evaluate(const Arm_Chain_t *chain, const float *q,
                                 float *T, float *J, Arm_Frames_t *frames)
{
    if (J == NULL && frames == NULL)
    {
        /* 仅 FK 时无需保存各关节累计位姿或构造雅可比。 */
        return arm_forward(chain, q, T, NULL);
    }
    return arm_evaluate_with_frames(chain, q, T, J, frames);
}

/* 原始模型入口：每次完整校验，同一份输入只递推一次。 */
Arm_Status_t Arm_Evaluate(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS],
                          float T[16], float J[6 * ARM_MAX_JOINTS], Arm_Frames_t *frames)
{
    if (T == NULL && J == NULL && frames == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    Arm_Status_t status = arm_validate_state(m, q);
    if (status != ARM_STATUS_OK)
    {
        return status;
    }
    Arm_Chain_t chain = {0};
    chain.n = m->n;
    chain.raw_links = m->links;
    chain.T_world_base = m->T_world_base;
    chain.T_n_tcp = m->T_n_tcp;
    for (uint8_t i = 0; i < m->n; i++)
    {
        chain.types[i] = m->links[i].type;
    }
    return arm_evaluate(&chain, q, T, J, frames);
}

/* 运行模型入口：固定参数只在准备时校验，实时阶段不读取原始配置。 */
Arm_Status_t Arm_Evaluate_Prepared(const Arm_PreparedModel_t *prepared,
                                   const float q[ARM_MAX_JOINTS], float T[16],
                                   float J[6 * ARM_MAX_JOINTS], Arm_Frames_t *frames)
{
    if (prepared == NULL || q == NULL || (T == NULL && J == NULL && frames == NULL))
    {
        return ARM_STATUS_NULL_POINTER;
    }
    Arm_PreparedHeader_t header;
    memcpy(&header, prepared->_opaque, sizeof(header));
    if (header.tag != ARM_PREPARED_TAG || header.n == 0 || header.n > ARM_MAX_JOINTS ||
        header.has_tool > 1)
    {
        return ARM_STATUS_MODEL_NOT_PREPARED;
    }
    if (!arm_all_finite(q, header.n))
    {
        return ARM_STATUS_NONFINITE_INPUT;
    }

    Arm_Chain_t chain = {0};
    chain.n = header.n;
    memcpy(chain.types, header.types, sizeof(chain.types));
    chain.prepared_links = prepared->_opaque + offsetof(Arm_PreparedData_t, links);
    chain.T_world_base = prepared->_opaque + offsetof(Arm_PreparedData_t, T_world_base);
    if (header.has_tool)
    {
        chain.T_n_tcp = prepared->_opaque + offsetof(Arm_PreparedData_t, T_n_tcp);
    }
    return arm_evaluate(&chain, q, T, J, frames);
}

/* 保留单独输出接口；同时需要 T 和 J 时应直接调用 Arm_Evaluate。 */
Arm_Status_t Arm_FK(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS], float T[16])
{
    if (T == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    return Arm_Evaluate(m, q, T, NULL, NULL);
}

Arm_Status_t Arm_Jacobian(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS],
                          float J[6 * ARM_MAX_JOINTS])
{
    if (J == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    return Arm_Evaluate(m, q, NULL, J, NULL);
}

/* 从旋转矩阵提取 yaw、pitch、roll，奇异姿态固定 roll 为零。 */
Arm_Status_t Arm_T2YPR(const float T[16], float ypr[3])
{
    if (T == NULL || ypr == NULL)
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
    memcpy(ypr, result, sizeof(result));
    return ARM_STATUS_OK;
}

/* 按 Rz(yaw)*Ry(pitch)*Rx(roll) 构造姿态，并填入位置。 */
Arm_Status_t Arm_YPR2T(const float ypr[3], const float p[3], float T[16])
{
    if (ypr == NULL || p == NULL || T == NULL)
    {
        return ARM_STATUS_NULL_POINTER;
    }
    if (!arm_all_finite(ypr, 3) || !arm_all_finite(p, 3))
    {
        return ARM_STATUS_NONFINITE_INPUT;
    }
    float result[16];
    float cy = cosf(ypr[0]);
    float sy = sinf(ypr[0]);
    float cp = cosf(ypr[1]);
    float sp = sinf(ypr[1]);
    float cr = cosf(ypr[2]);
    float sr = sinf(ypr[2]);

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
