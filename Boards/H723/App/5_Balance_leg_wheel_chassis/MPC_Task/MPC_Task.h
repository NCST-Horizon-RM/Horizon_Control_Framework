//
// Created by qza on 2026/8/29.
// 由 user_class.py 的 Mpc 类逐行移植（保留全部成员，去掉 Le / eps / pri-dua 等未使用项）。
//

#ifndef HORIZON_CONTROL_FRAMEWORK_MPC_H
#define HORIZON_CONTROL_FRAMEWORK_MPC_H

#include "stdint.h"
#include "arm_math.h"
#include "All_define.h"
#include "MPC_define.h"

/* ===== 维数，对应 Python Mpc.__init__ 的 state / input / N ===== */
#define MPC_TS           0.002f
#define state_num        10      /* Python: state                      */
#define input_num         4      /* Python: input                      */
#define coeffs_num       10      /* 特征向量长度 [1,x,y,x2,xy,y2,x3,..]  */
#define prediction_num   10      /* Python: N                          */
#define iteration_count  20      /* Python: for j in range(1)          */

/* ===== 参考轨迹速度限幅（位置目的地模式）=====
 * 每步位移/转角的上限 = 上限速度 * dt，见 MPC_Task.c 的 Error_Calculation()。
 * 目的地很远时，位置参考不会一步迈到位，而是以这个上限速度往前推。 */
#define mpc_ref_v_max     2.0f   /* m/s   平移参考速度上限 */
#define mpc_ref_vw_max    2.0f   /* rad/s 偏航参考角速度上限 */


/* ===== 系数表 =====
 * 数组本体是 *文件作用域* 的 static const（见 MPC_Task.c 里的 mpc_*_coeffs），
 * 链接后落在 FLASH 的 .rodata，不占 RAM；这里只保存指向它们的 CMSIS 矩阵句柄。
 * 数组按行主序展开成 (rows*cols, coeffs_num) 的二维表，运行时左乘特征向量。
 * 尺寸 = rows*cols*coeffs_num：
 *   K      (4,10)  -> 400     P      (10,10) -> 1000
 *   Q_uu   (4,4)   -> 160     A      (10,10) -> 1000
 *   B      (10,4)  -> 400     AmBK_T (10,10) -> 1000
 *   f      (10,1)  -> 100     APf    (10,1)  -> 100
 *   BPf    (4,1)   -> 40      theta / theta_l / theta_r (1,1) -> 10
 */
typedef struct {
    float input[coeffs_num];            /* 特征向量，对应 Python 的 features */

    arm_matrix_instance_f32 K_coeffs_matrix;        /* (40,  10) @ FLASH */
    arm_matrix_instance_f32 P_coeffs_matrix;        /* (100, 10) @ FLASH */
    arm_matrix_instance_f32 Q_uu_coeffs_matrix;     /* (16,  10) @ FLASH */
    arm_matrix_instance_f32 A_coeffs_matrix;        /* (100, 10) @ FLASH */
    arm_matrix_instance_f32 B_coeffs_matrix;        /* (40,  10) @ FLASH */
    arm_matrix_instance_f32 AmBK_T_coeffs_matrix;   /* (100, 10) @ FLASH */
    arm_matrix_instance_f32 f_coeffs_matrix;        /* (10,  10) @ FLASH */
    arm_matrix_instance_f32 APf_coeffs_matrix;      /* (10,  10) @ FLASH */
    arm_matrix_instance_f32 BPf_coeffs_matrix;      /* (4,   10) @ FLASH */
    arm_matrix_instance_f32 theta_coeffs_matrix;    /* (1,   10) @ FLASH */
    arm_matrix_instance_f32 theta_l_coeffs_matrix;  /* (1,   10) @ FLASH */
    arm_matrix_instance_f32 theta_r_coeffs_matrix;  /* (1,   10) @ FLASH */
} MPC_coeffs_t;

typedef struct {
    /* ===== get_matrix() 每个控制周期刷新的时变矩阵 ===== */
    float K[input_num * state_num];         /* self.K        (4,10)  */
    float K_T[state_num * input_num];       /* self.K.T      (10,4)  */
    float P[state_num * state_num];         /* self.P        (10,10) */
    float A[state_num * state_num];         /* self.A        (10,10) */
    float AmBK_T[state_num * state_num];    /* self.AmBK_T   (10,10) */
    float B[state_num * input_num];         /* self.B        (10,4)  */
    float B_T[input_num * state_num];       /* self.B.T      (4,10)  */
    float Q_uu[input_num * input_num];      /* self.Q_uu     (4,4)   */
    float f[state_num];                     /* self.f        (10,)   */
    float APf[state_num];                   /* self.APf      (10,)   */
    float BPf[input_num];                   /* self.BPf      (4,)    */

    /* ===== 权重与标量 ===== */
    float Q[state_num];                     /* self.Q        (对角)   */
    float R[input_num];                     /* self.R        (对角)   */
    float rho;                              /* self.rho              */

    /* ===== 约束（x_min/x_max 目前只保存，clip 仍按 Python 注释保留）===== */
    float u_min[input_num];
    float u_max[input_num];
    float x_min[3];
    float x_max[3];

    /* ===== 腿角拟合输出（get_matrix 的返回量）===== */
    float theta;                            /* Python: theta         */
    float theta_leg_l;                      /* Python: theta_l       */
    float theta_leg_r;                      /* Python: theta_r       */

    /* ===== ADMM 工作区，逐项对应 Python 同名成员 ===== */
    float d[prediction_num - 1][input_num];     /* self.d      */
    float q[prediction_num - 1][state_num];     /* self.q      */
    float r[prediction_num - 1][input_num];     /* self.r      */
    float p[prediction_num][state_num];         /* self.p      */
    float u[prediction_num - 1][input_num];     /* self.u      */
    float x[prediction_num][state_num];         /* self.x      */
    float v[prediction_num][state_num];         /* self.v      */
    float z[prediction_num - 1][input_num];     /* self.z      */
    float g[prediction_num][state_num];         /* self.g      */
    float y[prediction_num - 1][input_num];     /* self.y      */

    /* ===== 参考轨迹与测量，对应 Python 的 x_ref / x_measure / u_ref ===== */
    float x_ref[prediction_num][state_num];         /* self.x_ref     */
    float x_measure[prediction_num][state_num];     /* self.x_measure */
    float u_ref[prediction_num - 1][input_num];     /* self.u_ref     */

    /* ===== 临时向量（原版 temp1..temp8 的精简替身）===== */
    float temp_vec_x[state_num];
    float temp_vec_x2[state_num];
    float temp_vec_x3[state_num];
    float temp_vec_u[input_num];
    float temp_vec_u2[input_num];

    /* ===== CMSIS-DSP 句柄 ===== */
    arm_matrix_instance_f32 K_matrix;
    arm_matrix_instance_f32 K_T_matrix;
    arm_matrix_instance_f32 P_matrix;
    arm_matrix_instance_f32 A_matrix;
    arm_matrix_instance_f32 AmBK_T_matrix;
    arm_matrix_instance_f32 B_matrix;
    arm_matrix_instance_f32 B_T_matrix;
    arm_matrix_instance_f32 Q_uu_matrix;
} MPC_t;

void Coeffs_Init(MPC_coeffs_t* mpc_coeffs);
void Mpc_Init(MPC_coeffs_t* mpc_coeffs, MPC_t* mpc);
void Update_Matrix_Simplify(MPC_coeffs_t* mpc_coeffs, MPC_t* mpc, const float leg_length_l, const float leg_length_r);
void Mpc_Update_Ref(MPC_t* mpc);
void Error_Calculation(MPC_t* mpc,float dt);
void MPC_Admm(MPC_t* mpc);

#endif //HORIZON_CONTROL_FRAMEWORK_MPC_H
