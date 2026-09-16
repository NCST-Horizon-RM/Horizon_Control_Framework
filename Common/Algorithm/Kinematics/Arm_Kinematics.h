/**
 * @file    Arm_Kinematics.h
 * @brief   最多 7 关节的机械臂运动学（改进 DH / MDH, Craig 参数化）
 *          提供：单关节齐次变换、含基座 / 工具变换的正运动学和几何雅可比、YPR 姿态互转。
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
    ARM_STATUS_INVALID_JOINT_TYPE = -3,     /* type 不是 R / P */
    ARM_STATUS_NONFINITE_INPUT = -4,       /* 模型参数或计算输入含 NaN / Inf */
    ARM_STATUS_NUMERIC_ERROR = -5,         /* 有限输入计算溢出，或矩阵运算失败 */
    ARM_STATUS_INVALID_TRANSFORM = -6,     /* 基座 / 工具矩阵不是刚体齐次变换 */
    ARM_STATUS_MODEL_NOT_PREPARED = -7     /* 运行模型未成功准备或准备标记无效 */
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

/* 串联机械臂模型（最多 7 关节）。先整体零初始化，再填写配置。
 * 两个可选变换均为行主序 float[16]，NULL 表示单位阵，保留仅 MDH 的行为。
 * 模型借用矩阵、不复制；调用方保证其存储在模型使用期间有效，计算期间不被修改。
 * T_world_base 和 T_n_tcp 均视为固定变换，雅可比只对关节 q 求导。 */
typedef struct
{
    uint8_t n;                          /* 实际关节数（1..ARM_MAX_JOINTS） */
    Arm_Link_t links[ARM_MAX_JOINTS];    /* 按基座到末端排列，仅前 n 项有效 */
    const float *T_world_base;          /* 模型基座在世界系中的位姿（base -> world） */
    const float *T_n_tcp;               /* TCP 在最后一个 MDH 系中的位姿（tcp -> n） */
} Arm_Model_t;

/* 调用方持有的运行模型；先整体零初始化，再调用 Arm_Model_Prepare。
 * 不透明存储仅供模块内部使用，禁止直接读写；运行期通过 const 指针访问。
 * 只允许用 Arm_Model_Prepare 更新，且更新期间不得有并发求值。
 * 内含固定几何参数及基座 / 工具变换的独立快照，不借用原始模型中的指针。
 * 修改原始参数不会自动影响本对象，须检查重新准备成功后再用于控制。
 * 无动态分配或全局实例；存储大小为内部实现预留，不表示公开的数据布局。 */
typedef struct
{
    uint8_t _opaque[160 + 32 * ARM_MAX_JOINTS];
} Arm_PreparedModel_t;

/* 一次求值的中间几何结果，由调用方持有，不是可自动判定有效性的缓存。
 * 下标 i 对应 links[i]：T_world_joint[i] = T_world_base * A_1 * ... * A_{i+1}，
 * 不含工具变换。矩阵第三列 [2,6,10] 是该 MDH 关节轴在世界系中的方向，
 * 平移列 [3,7,11] 是轴上一点；也可用该矩阵变换相应连杆坐标系中的点。
 * 仅前 n 项有效，其余矩阵清零。q、模型参数或基座 / 工具变换改变后须重新求值。 */
typedef struct
{
    uint8_t n;
    float T_world_joint[ARM_MAX_JOINTS][16];
} Arm_Frames_t;

#ifdef __cplusplus
extern "C" {
#endif

/* App 初始化或修改模型后调用：检查 n、有效关节的 type 和全部浮点参数。
 * 仅检查 links[0..n-1]；qmin >= qmax 仍表示不限位，两个边界本身须为有限数。
 * 非 NULL 的基座 / 工具矩阵须全部有限，末行为 [0,0,0,1]；旋转矩阵满足
 * R^T*R = I、det(R) = +1（各元素 / 行列式的绝对误差不超过 1e-4）。
 * 本接口不改写模型；原始模型的计算接口也会校验，避免参数修改导致越界。 */
Arm_Status_t Arm_Model_Validate(const Arm_Model_t *m);

/* 完整校验原始配置、复制固定参数并预计算 alpha 的 sin/cos；
 * 移动关节还预计算固定 theta 的 sin/cos，所有关节预计算常量与 offset 之和。
 * 失败时保留 prepared 的原值，调用方不可把失败当成新配置已生效。
 * prepared 不得与 m 或其借用的矩阵重叠；准备期间原始配置须保持不变。 */
Arm_Status_t Arm_Model_Prepare(const Arm_Model_t *m, Arm_PreparedModel_t *prepared);

/* 单关节齐次变换：T(4x4, 行主序 float[16]) = A_i(q) */
Arm_Status_t Arm_Link_T(const Arm_Link_t *link, float q, float T[16]);

/* 一次逐关节递推，同时得到 TCP 位姿、几何雅可比和可选中间几何结果。
 * T、J 的布局和坐标系与 Arm_FK / Arm_Jacobian 相同；frames 见 Arm_Frames_t。
 * 三个输出均可为 NULL（跳过相应输出），但至少提供一个，否则返回 NULL_POINTER。
 * 每次均按当前 m、q 重新计算，不依赖输入指针或旧 frames；不持有全局静态缓存。
 * 所有请求的输出计算成功后才统一写回，失败时所有输出均保持不变。
 * 输出缓冲区须互不重叠，并且不与 m、q 或模型借用的变换矩阵重叠。
 * 调用期间输入须保持不变；只读取 q[0..n-1]，不截断关节限位。
 * 同时需要 T 和 J 时调用本接口，避免分别调用 FK / Jacobian 重复递推。 */
Arm_Status_t Arm_Evaluate(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS],
                          float T[16], float J[6 * ARM_MAX_JOINTS], Arm_Frames_t *frames);

/* 运行模型的实时求值：输出约定和单次递推与 Arm_Evaluate 相同。
 * prepared 必须来自成功的 Arm_Model_Prepare，此后只能由准备接口更新。
 * 运行期检查准备标记、关节数、q 有限性及计算溢出，不重复校验固定模型参数，
 * 不重复计算固定角度的三角函数；原始模型接口仍保留每次完整校验。
 * prepared 在整个调用期间须保持只读，且不得与任一输出缓冲区重叠。
 * 任一输出可为 NULL，但至少请求一个；失败时所有输出均保持不变。 */
Arm_Status_t Arm_Evaluate_Prepared(const Arm_PreparedModel_t *prepared,
                                   const float q[ARM_MAX_JOINTS], float T[16],
                                   float J[6 * ARM_MAX_JOINTS], Arm_Frames_t *frames);

/* 正运动学：T_world_tcp = T_world_base * A_1 * A_2 * ... * A_n * T_n_tcp。
 * 输出 T[3]、T[7]、T[11] 为 TCP 在世界系中的位置，旋转部分为 TCP 在世界系中的姿态。
 * 未配置基座变换时世界系即模型基座系；未配置工具变换时 TCP 系即最后一个 MDH 系。
 * FK / Jacobian 仅检查和使用 q[0..n-1]；有效 q 超出限位不报错也不截断。 */
Arm_Status_t Arm_FK(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS], float T[16]);

/* 几何雅可比：J(6 x n)，行主序，每行长度 ARM_MAX_JOINTS。
 * [v_tcp; omega_tcp] = J * q_dot，与 Arm_FK 的 TCP 和世界坐标系一致。
 * 行 0..2 = TCP 线速度（m/s），行 3..5 = TCP 角速度（rad/s），均在世界坐标系表达。
 * 角速度不是 YPR 欧拉角变化率；不包含移动基座或可变工具相对运动产生的速度。
 * 逆解的姿态误差应由旋转矩阵或四元数构造并在世界系表达，不能直接使用 YPR 分量差。
 * 列 i 对应关节 i；列 n..ARM_MAX_JOINTS-1 清零，输出可作为 6 x ARM_MAX_JOINTS 矩阵使用。 */
Arm_Status_t Arm_Jacobian(const Arm_Model_t *m, const float q[ARM_MAX_JOINTS],
                          float J[6 * ARM_MAX_JOINTS]);

/* YPR 接口统一使用 ypr[0] = yaw、ypr[1] = pitch、ypr[2] = roll（单位 rad），
 * R = Rz(yaw)*Ry(pitch)*Rx(roll)。欧拉角主要用于显示和人工输入。
 * URDF 的 rpy 数组为 [roll,pitch,yaw]，对接时须重排为 [rpy[2],rpy[1],rpy[0]]。 */

/* 齐次变换 -> YPR 欧拉角 [yaw,pitch,roll]。
 * pitch 属于 [-pi/2, pi/2]；|cos(pitch)| <= 1e-6 时固定 roll = 0，返回等价姿态。
 * 检查 T 全部元素的有限性；齐次变换的正交性等几何约束由调用方保证。 */
Arm_Status_t Arm_T2YPR(const float T[16], float ypr[3]);

/* YPR 欧拉角 [yaw,pitch,roll]（rad）+ 位置 p = [x,y,z]（m）-> 齐次变换。 */
Arm_Status_t Arm_YPR2T(const float ypr[3], const float p[3], float T[16]);

#ifdef __cplusplus
}
#endif

#endif /* HORIZON_ARM_KINEMATICS_H */
