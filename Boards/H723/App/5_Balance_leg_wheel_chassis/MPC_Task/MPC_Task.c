//
// Created by qza on 2026/8/29.
// 由 user_class.py 的 Mpc 类逐行移植，风格与原 MPC_Task.c 保持一致（CMSIS-DSP + float）。
//

#include "MPC_Task.h"
#include "arm_math.h"
#include "VMC_Control.h"
#include "Robot_Cmd.h"
#include "IMU_Task.h"
#include "Chassis_Ctrl.h"


/* ===== 系数表本体：static const → .rodata → FLASH，不占 RAM =====
 * 对应 Python Mpc.__init__ 里的 coeff 入手；*_Coeffs_Def 由 offline 包生成，
 * 见仓库根目录 coeff_c_arrays.txt。CMSIS 只读这些数组，不做写入。
 * 尺寸：K 400 / P 1000 / Q_uu 160 / A 1000 / B 400 / AmBK_T 1000 /
 *       f 100 / APf 100 / BPf 40 / theta(θ,θl,θr) 各 10，合计 4230 个 float。
 */
static const float mpc_k_coeffs[input_num * state_num * coeffs_num] = K_Coeffs_Def;
static const float mpc_p_coeffs[state_num * state_num * coeffs_num] = P_Coeffs_Def;
static const float mpc_q_uu_coeffs[input_num * input_num * coeffs_num] = Quu_inv_Coeffs_Def;
static const float mpc_a_coeffs[state_num * state_num * coeffs_num] = A_Coeffs_Def;
static const float mpc_b_coeffs[state_num * input_num * coeffs_num] = B_Coeffs_Def;
static const float mpc_ambk_t_coeffs[state_num * state_num * coeffs_num] = AmBK_T_Coeffs_Def;
static const float mpc_f_coeffs[state_num * coeffs_num] = f_Coeffs_Def;
static const float mpc_apf_coeffs[state_num * coeffs_num] = APf_Coeffs_Def;
static const float mpc_bpf_coeffs[input_num * coeffs_num] = BPf_Coeffs_Def;
static const float mpc_theta_coeffs[coeffs_num] = theta_Coeffs_Def;
static const float mpc_theta_l_coeffs[coeffs_num] = theta_l_Coeffs_Def;
static const float mpc_theta_r_coeffs[coeffs_num] = theta_r_Coeffs_Def;

/* 系数句柄直接绑定到 FLASH 上的 const 数组，不再往 RAM 拷一份。
 * arm_matrix_instance_f32.pData 是 float32_t* 而数组是 const，故此处显式去 const：
 * CMSIS 的 arm_mat_vec_mult_f32 只读 pData，不会写回，去掉 const 是安全的。 */
void Coeffs_Init(MPC_coeffs_t* mpc_coeffs) {
    memset(mpc_coeffs->input, 0, sizeof(mpc_coeffs->input));

    arm_mat_init_f32(&mpc_coeffs->K_coeffs_matrix,
                     input_num * state_num, coeffs_num, (float32_t *)mpc_k_coeffs);
    arm_mat_init_f32(&mpc_coeffs->P_coeffs_matrix,
                     state_num * state_num, coeffs_num, (float32_t *)mpc_p_coeffs);
    arm_mat_init_f32(&mpc_coeffs->Q_uu_coeffs_matrix,
                     input_num * input_num, coeffs_num, (float32_t *)mpc_q_uu_coeffs);
    arm_mat_init_f32(&mpc_coeffs->A_coeffs_matrix,
                     state_num * state_num, coeffs_num, (float32_t *)mpc_a_coeffs);
    arm_mat_init_f32(&mpc_coeffs->B_coeffs_matrix,
                     state_num * input_num, coeffs_num, (float32_t *)mpc_b_coeffs);
    arm_mat_init_f32(&mpc_coeffs->AmBK_T_coeffs_matrix,
                     state_num * state_num, coeffs_num, (float32_t *)mpc_ambk_t_coeffs);
    arm_mat_init_f32(&mpc_coeffs->f_coeffs_matrix,
                     state_num, coeffs_num, (float32_t *)mpc_f_coeffs);
    arm_mat_init_f32(&mpc_coeffs->APf_coeffs_matrix,
                     state_num, coeffs_num, (float32_t *)mpc_apf_coeffs);
    arm_mat_init_f32(&mpc_coeffs->BPf_coeffs_matrix,
                     input_num, coeffs_num, (float32_t *)mpc_bpf_coeffs);
    arm_mat_init_f32(&mpc_coeffs->theta_coeffs_matrix,
                     1, coeffs_num, (float32_t *)mpc_theta_coeffs);
    arm_mat_init_f32(&mpc_coeffs->theta_l_coeffs_matrix,
                     1, coeffs_num, (float32_t *)mpc_theta_l_coeffs);
    arm_mat_init_f32(&mpc_coeffs->theta_r_coeffs_matrix,
                     1, coeffs_num, (float32_t *)mpc_theta_r_coeffs);
}

/* 对应 Python Mpc.__init__ 的常量部分 */
void Mpc_Init(MPC_coeffs_t* mpc_coeffs, MPC_t* mpc) {
    /* self.Q */
    mpc->Q[0] = 50.0f;
    mpc->Q[1] = 50.0f;
    mpc->Q[2] = 8000.0f;
    mpc->Q[3] = 6000.0f;
    mpc->Q[4] = 6000.0f;
    mpc->Q[5] = 50.0f;
    mpc->Q[6] = 50.0f;
    mpc->Q[7] = 1.0f;
    mpc->Q[8] = 1.0f;
    mpc->Q[9] = 1.0f;

    /* self.R */

    mpc->R[0] = 5.0f;
    mpc->R[1] = 5.0f;
    mpc->R[2] = 10.0f;
    mpc->R[3] = 10.0f;


    /* self.rho */
    mpc->rho = 2.0f;

    /* self.u_min / self.u_max */
    mpc->u_min[0] = -50.0f;
    mpc->u_min[1] = -50.0f;
    mpc->u_min[2] = -6.0f;
    mpc->u_min[3] = -6.0f;
    mpc->u_max[0] = 50.0f;
    mpc->u_max[1] = 50.0f;
    mpc->u_max[2] = 6.0f;
    mpc->u_max[3] = 6.0f;

    /* self.x_min / self.x_max */
    mpc->x_min[0] = -0.5f;
    mpc->x_min[1] = -1.5f;
    mpc->x_min[2] = -1.5f;
    mpc->x_max[0] = 0.5f;
    mpc->x_max[1] = 1.5f;
    mpc->x_max[2] = 1.5f;

    /* 系数表句柄在 Coeffs_Init() 里已直接绑到 FLASH 上的 const 数组，这里不再重复初始化。
     * Chassis_Ctrl.c 的调用顺序是 Coeffs_Init() → Mpc_Init()，所以用到时句柄一定就绪。 */
    (void)mpc_coeffs;

    /* ===== 运行时矩阵句柄 ===== */
    arm_mat_init_f32(&mpc->K_matrix, input_num, state_num, mpc->K);
    arm_mat_init_f32(&mpc->K_T_matrix, state_num, input_num, mpc->K_T);
    arm_mat_init_f32(&mpc->P_matrix, state_num, state_num, mpc->P);
    arm_mat_init_f32(&mpc->A_matrix, state_num, state_num, mpc->A);
    arm_mat_init_f32(&mpc->AmBK_T_matrix, state_num, state_num, mpc->AmBK_T);
    arm_mat_init_f32(&mpc->B_matrix, state_num, input_num, mpc->B);
    arm_mat_init_f32(&mpc->B_T_matrix, input_num, state_num, mpc->B_T);
    arm_mat_init_f32(&mpc->Q_uu_matrix, input_num, input_num, mpc->Q_uu);

    /* ===== 工作区清零 ===== */
    memset(mpc->d, 0, sizeof(mpc->d));
    memset(mpc->q, 0, sizeof(mpc->q));
    memset(mpc->r, 0, sizeof(mpc->r));
    memset(mpc->p, 0, sizeof(mpc->p));
    memset(mpc->u, 0, sizeof(mpc->u));
    memset(mpc->x, 0, sizeof(mpc->x));
    memset(mpc->v, 0, sizeof(mpc->v));
    memset(mpc->z, 0, sizeof(mpc->z));
    memset(mpc->g, 0, sizeof(mpc->g));
    memset(mpc->y, 0, sizeof(mpc->y));
    memset(mpc->x_ref, 0, sizeof(mpc->x_ref));
    memset(mpc->x_measure, 0, sizeof(mpc->x_measure));
    memset(mpc->u_ref, 0, sizeof(mpc->u_ref));
    memset(mpc->temp_vec_x, 0, sizeof(mpc->temp_vec_x));
    memset(mpc->temp_vec_x2, 0, sizeof(mpc->temp_vec_x2));
    memset(mpc->temp_vec_x3, 0, sizeof(mpc->temp_vec_x3));
    memset(mpc->temp_vec_u, 0, sizeof(mpc->temp_vec_u));
    memset(mpc->temp_vec_u2, 0, sizeof(mpc->temp_vec_u2));
}

/* 对应 Python Mpc.get_matrix(leg_lenth_l, leg_lenth_r)：
 * 先按 (l_l, l_r) 造 10 维特征向量，再对每张系数表做 (rows*cols,10)@(10,)，
 * 结果 reshape 回 (rows, cols)。K_T / B_T 一并转置好，供 ADMM 直接用。 */
void Update_Matrix_Simplify(MPC_coeffs_t* mpc_coeffs, MPC_t* mpc, const float leg_length_l, const float leg_length_r) {
    const float x = leg_length_l;
    const float y = leg_length_r;

    const float x2 = x * x;
    const float xy = x * y;
    const float y2 = y * y;
    const float x3 = x * x2;
    const float x2y = x2 * y;
    const float xy2 = x * y2;
    const float y3 = y * y2;

    /* features = [1, x, y, x^2, x*y, y^2, x^3, x^2*y, x*y^2, y^3] */
    mpc_coeffs->input[0] = 1.0f;
    mpc_coeffs->input[1] = x;
    mpc_coeffs->input[2] = y;
    mpc_coeffs->input[3] = x2;
    mpc_coeffs->input[4] = xy;
    mpc_coeffs->input[5] = y2;
    mpc_coeffs->input[6] = x3;
    mpc_coeffs->input[7] = x2y;
    mpc_coeffs->input[8] = xy2;
    mpc_coeffs->input[9] = y3;

    arm_mat_vec_mult_f32(&mpc_coeffs->K_coeffs_matrix, mpc_coeffs->input, mpc->K);
    arm_mat_vec_mult_f32(&mpc_coeffs->P_coeffs_matrix, mpc_coeffs->input, mpc->P);
    arm_mat_vec_mult_f32(&mpc_coeffs->A_coeffs_matrix, mpc_coeffs->input, mpc->A);
    arm_mat_vec_mult_f32(&mpc_coeffs->B_coeffs_matrix, mpc_coeffs->input, mpc->B);
    arm_mat_vec_mult_f32(&mpc_coeffs->Q_uu_coeffs_matrix, mpc_coeffs->input, mpc->Q_uu);
    arm_mat_vec_mult_f32(&mpc_coeffs->AmBK_T_coeffs_matrix, mpc_coeffs->input, mpc->AmBK_T);
    arm_mat_vec_mult_f32(&mpc_coeffs->f_coeffs_matrix, mpc_coeffs->input, mpc->f);
    arm_mat_vec_mult_f32(&mpc_coeffs->APf_coeffs_matrix, mpc_coeffs->input, mpc->APf);
    arm_mat_vec_mult_f32(&mpc_coeffs->BPf_coeffs_matrix, mpc_coeffs->input, mpc->BPf);
    arm_mat_vec_mult_f32(&mpc_coeffs->theta_coeffs_matrix, mpc_coeffs->input, &mpc->theta);
    arm_mat_vec_mult_f32(&mpc_coeffs->theta_l_coeffs_matrix, mpc_coeffs->input, &mpc->theta_leg_l);
    arm_mat_vec_mult_f32(&mpc_coeffs->theta_r_coeffs_matrix, mpc_coeffs->input, &mpc->theta_leg_r);

    arm_mat_trans_f32(&mpc->K_matrix, &mpc->K_T_matrix);
    arm_mat_trans_f32(&mpc->B_matrix, &mpc->B_T_matrix);
}

/* 对应 Python Mpc.updata_ref(self)：原实现就是 pass，这里保留同名占位。 */
void Mpc_Update_Ref(MPC_t* mpc) {
    (void)mpc;
}

/* 对应 Python State.update_state() / State.update_target() 的取数与铺参考轨迹。
 * 保持原函数名，All_Task.c 的调用点不用改。 */
static void Get_State_And_Target(float *measure, float *ref, const float dt) {
    /* self.x = ... （对应 update_state） */
    measure[0] = chassis_ctrl.kalman.outstate.s;
    measure[1] = IMU_Data.yaw * DEG2RAD;
    measure[2] = IMU_Data.pitch * DEG2RAD;
    measure[3] = chassis_ctrl.vmc.leg.left.theta[0];
    measure[4] = chassis_ctrl.vmc.leg.right.theta[0];
    measure[5] = chassis_ctrl.kalman.outstate.dot_s;
    measure[6] = IMU_Data.gyro[2];
    measure[7] = IMU_Data.gyro[1];
    measure[8] = chassis_ctrl.vmc.leg.left.theta[1];
    measure[9] = chassis_ctrl.vmc.leg.right.theta[1];

    /* self.x_target = ... （对应 update_target，腿角用本拍拟合值） */

    ref[0] = chassis_cmd.target_x;//+Planner_VtoX(measure[0],measure[5],chassis_cmd.target_vx);;
    ref[1] = chassis_cmd.target_w;
    ref[2] = chassis_ctrl.mpc.theta;
    ref[3] = chassis_ctrl.mpc.theta_leg_l;
    ref[4] = chassis_ctrl.mpc.theta_leg_r;
    ref[5] = chassis_cmd.target_vx;
    ref[6] = chassis_cmd.target_vw;
    ref[7] = 0.0f;
    ref[8] = 0.0f;
    ref[9] = 0.0f;
}

/* Python 侧这两步写在 leg_control.py 里：
 *   x_ref   = np.tile(x_target, (N, 1))
 *   u_ref   = np.tile(np.zeros(4), (N, 1))
 * 这里落到 self.x_ref / self.x_measure / self.u_ref 成员上。
 * 注意 x_ref[0] 同时充当 ref[0] += v*dt 的累加器，必须常驻（和原版的 mpc->ref 一样）。 */
void Error_Calculation(MPC_t* mpc, const float dt) {
    Get_State_And_Target(mpc->x_measure[0], mpc->x_ref[0], dt);

    /* x_ref = np.tile(x_target, (N, 1)) */
    for (int i = 1; i < prediction_num; i++) {
        for (int k = 0; k < state_num; k++) {
            mpc->x_ref[i][k] = mpc->x_ref[0][k];
        }
    }

    /* u_ref = np.tile(np.zeros(4), (N, 1)) */
    for (int i = 0; i < prediction_num - 1; i++) {
        for (int k = 0; k < input_num; k++) {
            mpc->u_ref[i][k] = 0.0f;
        }
    }
}

/* 对应 Python Mpc.ADMM(x_measure, x_ref, u_ref)：
 * 一次反向递推求 p/d，再一次前向递推求 u/x，然后做 v/z/g/y 的更新。
 * 外层 for 对应 Python 的 "for j in range(1)"，iteration_count = 1。 */
void MPC_Admm(MPC_t* mpc) {
    /* Python: self.x[0] = np.asarray(x_measure).flatten() */
    arm_copy_f32(mpc->x_measure[0], mpc->x[0], state_num);

    for (int j = 0; j < iteration_count; j++) {
        /* self.p[N-1] = -P@x_ref[N-1] - rho*(v[N-1]-g[N-1]) */
        arm_sub_f32(mpc->v[prediction_num - 1], mpc->g[prediction_num - 1], mpc->temp_vec_x, state_num);
        arm_scale_f32(mpc->temp_vec_x, mpc->rho, mpc->temp_vec_x, state_num);
        arm_mat_vec_mult_f32(&mpc->P_matrix, mpc->x_ref[prediction_num - 1], mpc->temp_vec_x2);
        arm_add_f32(mpc->temp_vec_x2, mpc->temp_vec_x, mpc->temp_vec_x2, state_num);
        arm_negate_f32(mpc->temp_vec_x2, mpc->p[prediction_num - 1], state_num);

        /* 反向递推 i = N-2 ... 0 */
        for (int i = prediction_num - 2; i >= 0; i--) {
            /* self.q[i] = -Q*x_ref[i] - rho*(v[i]-g[i]) */
            arm_mult_f32(mpc->Q, mpc->x_ref[i], mpc->temp_vec_x, state_num);
            arm_sub_f32(mpc->v[i], mpc->g[i], mpc->temp_vec_x2, state_num);
            arm_scale_f32(mpc->temp_vec_x2, mpc->rho, mpc->temp_vec_x2, state_num);
            arm_add_f32(mpc->temp_vec_x, mpc->temp_vec_x2, mpc->temp_vec_x, state_num);
            arm_negate_f32(mpc->temp_vec_x, mpc->q[i], state_num);

            /* self.r[i] = -R*u_ref[i] - rho*(z[i]-y[i]) */
            arm_mult_f32(mpc->R, mpc->u_ref[i], mpc->temp_vec_u, input_num);
            arm_sub_f32(mpc->z[i], mpc->y[i], mpc->temp_vec_u2, input_num);
            arm_scale_f32(mpc->temp_vec_u2, mpc->rho, mpc->temp_vec_u2, input_num);
            arm_add_f32(mpc->temp_vec_u, mpc->temp_vec_u2, mpc->temp_vec_u, input_num);
            arm_negate_f32(mpc->temp_vec_u, mpc->r[i], input_num);

            /* self.d[i] = Q_uu @ (B.T@p[i+1] + r[i] + BPf) */
            arm_mat_vec_mult_f32(&mpc->B_T_matrix, mpc->p[i + 1], mpc->temp_vec_u2);
            arm_add_f32(mpc->temp_vec_u2, mpc->r[i], mpc->temp_vec_u2, input_num);
            arm_add_f32(mpc->temp_vec_u2, mpc->BPf, mpc->temp_vec_u2, input_num);
            arm_mat_vec_mult_f32(&mpc->Q_uu_matrix, mpc->temp_vec_u2, mpc->d[i]);

            /* self.p[i] = AmBK_T@p[i+1] + q[i] + APf - K.T@r[i] */
            arm_mat_vec_mult_f32(&mpc->AmBK_T_matrix, mpc->p[i + 1], mpc->temp_vec_x);
            arm_add_f32(mpc->temp_vec_x, mpc->q[i], mpc->temp_vec_x, state_num);
            arm_add_f32(mpc->temp_vec_x, mpc->APf, mpc->temp_vec_x, state_num);
            arm_mat_vec_mult_f32(&mpc->K_T_matrix, mpc->r[i], mpc->temp_vec_x3);
            arm_sub_f32(mpc->temp_vec_x, mpc->temp_vec_x3, mpc->p[i], state_num);
        }

        /* 前向递推：self.u[i] = -K@x[i] - d[i]; self.x[i+1] = A@x[i] + B@u[i] + f */
        for (int i = 0; i < prediction_num - 1; i++) {
            arm_mat_vec_mult_f32(&mpc->K_matrix, mpc->x[i], mpc->temp_vec_u);
            arm_add_f32(mpc->temp_vec_u, mpc->d[i], mpc->temp_vec_u, input_num);
            arm_negate_f32(mpc->temp_vec_u, mpc->u[i], input_num);

            arm_mat_vec_mult_f32(&mpc->A_matrix, mpc->x[i], mpc->temp_vec_x);
            arm_mat_vec_mult_f32(&mpc->B_matrix, mpc->u[i], mpc->temp_vec_x2);
            arm_add_f32(mpc->temp_vec_x, mpc->temp_vec_x2, mpc->temp_vec_x, state_num);
            arm_add_f32(mpc->temp_vec_x, mpc->f, mpc->x[i + 1], state_num);
        }

        /* self.v[i] = self.x[i] + self.g[i]（原实现不做 x 约束裁剪）*/
        for (int i = 0; i < prediction_num; i++) {
            arm_add_f32(mpc->x[i], mpc->g[i], mpc->v[i], state_num);
        }

        /* self.z[i] = np.clip(u[i] + y[i], u_min, u_max)
         * u_min/u_max 前两维一致、后两维一致，按原写法分两段裁。 */
        for (int i = 0; i < prediction_num - 1; i++) {
            arm_add_f32(mpc->u[i], mpc->y[i], mpc->temp_vec_u, input_num);
            arm_clip_f32(&mpc->temp_vec_u[0], &mpc->z[i][0], mpc->u_min[0], mpc->u_max[0], 2);
            arm_clip_f32(&mpc->temp_vec_u[2], &mpc->z[i][2], mpc->u_min[2], mpc->u_max[2], 2);
        }

        /* self.g[i] += self.x[i] - self.v[i] */
        for (int i = 0; i < prediction_num; i++) {
            arm_sub_f32(mpc->x[i], mpc->v[i], mpc->temp_vec_x, state_num);
            arm_add_f32(mpc->g[i], mpc->temp_vec_x, mpc->g[i], state_num);
        }

        /* self.y[i] += self.u[i] - self.z[i] */
        for (int i = 0; i < prediction_num - 1; i++) {
            arm_sub_f32(mpc->u[i], mpc->z[i], mpc->temp_vec_u, input_num);
            arm_add_f32(mpc->y[i], mpc->temp_vec_u, mpc->y[i], input_num);
        }
    }
}
