/**
 * @file    Arm_Kinematics.h
 * @brief   最多 7 关节的机械臂运动学（改进 DH / MDH, Craig 参数化）
 *          提供：单关节齐次变换、正运动学、6 x N 几何雅可比、RPY 姿态互转。
 * @note    矩阵运算复用 Mat_Fixed.h（封装 CMSIS-DSP arm_math）。
 *          App 配置模型并传入关节侧 q；转角用 rad，长度用 m，T 均为行主序。
 */

#ifndef HORIZON_ARM_KINEMATICS_H
#define HORIZON_ARM_KINEMATICS_H

#include <stdint.h>

#define ARM_MAX_JOINTS 7 /* 支持的关节数上限 */

/* 关节类型 */
#define ARM_JOINT_R 0 /* 转动关节 Revolute */
#define ARM_JOINT_P 1 /* 移动关节 Prismatic */

/* 所有接口返回状态码；只有 ARM_STATUS_OK 时才可使用本次输出。
 * 失败时不改写输出缓冲区；调用方仍须保证数组长度足够且调用期间输入不被修改。 */
typedef enum
{
    ARM_STATUS_OK = 0,                    /* 成功 */
    ARM_STATUS_NULL_POINTER = -1,         /* 输入或输出指针为空 */
    ARM_STATUS_INVALID_JOINT_COUNT = -2,   /* n 不在 1..7 */
    ARM_STATUS_INVALID_JOINT_TYPE = -3,    /* type 不是 R / P */
    ARM_STATUS_NONFINITE_INPUT = -4, /* 模型参数或计算输入含 NaN / Inf */
    ARM_STATUS_NUMERIC_ERROR = -5    /* 有限输入计算溢出，或矩阵运算失败 */
} Arm_Status_t;

/* 改进 DH (MDH / Craig) 连杆参数。
 * 单关节变换顺序：A_i = Trans(x, a) * Rot(x, alpha) * Rot(z, theta) * Trans(z, d)
 * 第 i 关节的 a、alpha 对应 Craig 的 a_{i-1}、alpha_{i-1}，theta、d 对应 theta_i、d_i。
 * 其中 theta = theta + offset + q（转动关节），d = d + offset + q（移动关节）。
 * qmin/qmax 仅供上层控制或规划限位；本模块按实际输入 q 计算，不截断反馈或试探值。 */
typedef struct
{
    float alpha;  /* 连杆扭转（绕 x 轴），单位 rad */
    float a;      /* 连杆长度（沿 x 轴），单位 m */
    float theta;  /* 关节角常量（叠加 offset、q 前），单位 rad */
    float d;      /* 连杆偏置（沿 z 轴），单位 m */
    float offset; /* 叠加到 q 的零点偏置（rad/m），App 勿重复叠加 */
    float qmin;   /* 关节下限位（rad/m）；qmin >= qmax 表示不限位 */
    float qmax;   /* 关节上限位（rad/m） */
    uint8_t type; /* ARM_JOINT_R / ARM_JOINT_P */
} Arm_Link_t;

/* 串联机械臂模型（最多 7 关节） */
typedef struct
{
    uint8_t n;                          /* 实际关节数（1..ARM_MAX_JOINTS） */
    Arm_Link_t links[ARM_MAX_JOINTS];    /* 按基座到末端排列，仅前 n 项有效 */
} Arm_Model_t;

/* App 初始化或修改模型后调用：检查 n、有效关节的 type 和全部浮点参数。
 * 仅检查 links[0..n-1]；qmin >= qmax 仍表示不限位，两个边界本身须为有限数。
 * 本接口不改写模型；计算接口也会校验，避免漏调本接口或后续参数修改导致越界。 */
Arm_Status_t Arm_Model_Validate(const Arm_Model_t *m);

/* 单关节齐次变换：T(4x4, 行主序 float[16]) = A_i(q) */
Arm_Status_t Arm_Link_T(const Arm_Link_t *link, float q, float T[16]);

/* 正运动学：基座到最后一个 MDH 坐标系，T_0n = A_1 * A_2 * ... * A_n。
 * 输出 T[3]、T[7]、T[11] 为位置；不包含额外的基座安装变换和工具偏移。
 * FK / Jacobian 仅检查和使用 q[0..n-1]；有效 q 超出限位不报错也不截断。 */
Arm_Status_t Arm_FK(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS], float T[16]);

/* 几何雅可比：J(6 x n)，行主序，每行长度 ARM_MAX_JOINTS。
 * 行 0..2 = 末端线速度（m/s），行 3..5 = 末端角速度（rad/s），均在基座坐标系表达。
 * 列 i 对应关节 i；列 n..ARM_MAX_JOINTS-1 清零，输出可作为 6 x ARM_MAX_JOINTS 矩阵使用。 */
Arm_Status_t Arm_Jacobian(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS],
                          float J[6 * ARM_MAX_JOINTS]);

/* 齐次变换 -> RPY 欧拉角 [yaw; pitch; roll]（R = Rz(yaw)*Ry(pitch)*Rx(roll)，单位 rad）。
 * pitch 属于 [-pi/2, pi/2]；|cos(pitch)| <= 1e-6 时固定 roll = 0，返回等价姿态。
 * 检查 T 全部元素的有限性；齐次变换的正交性等几何约束由调用方保证。 */
Arm_Status_t Arm_T2RPY(const float T[16], float rpy[3]);

/* RPY 欧拉角 [yaw; pitch; roll]（rad）+ 位置 p = [x,y,z]（m）-> 齐次变换。 */
Arm_Status_t Arm_RPY2T(const float rpy[3], const float p[3], float T[16]);

#endif /* HORIZON_ARM_KINEMATICS_H */
