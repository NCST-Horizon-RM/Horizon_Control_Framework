#include "LQR_Control.h"
#include "k_table_2d.h"
#include "u_eq_table_2d.h"
#include "e3_table.h"
#include "All_define.h"
#include <string.h>

void LQR_Init(LQR_Control_t *lqr)
{
    memset(lqr, 0, sizeof(*lqr));
}

void LQR_SetTarget(LQR_Control_t *lqr, float velocity_mps,
                   float yaw_rate_radps, float pitch_rad,
                   float left_leg_theta_rad, float right_leg_theta_rad,
                   float dt)
{
    if (lqr == NULL) {
        return;
    }
    if (dt > 0.0f && dt < 0.1f) {
        lqr->target.position_m += lqr->target.velocity_mps * dt;
        lqr->target.yaw_rad += lqr->target.yaw_rate_radps * dt;
    }
    lqr->target.velocity_mps = velocity_mps;
    lqr->target.yaw_rate_radps = yaw_rate_radps;
    lqr->target.pitch_rad = pitch_rad;
    lqr->target.left_leg_theta_rad = left_leg_theta_rad;
    lqr->target.right_leg_theta_rad = right_leg_theta_rad;
}

void LQR_Update(LQR_Control_t *lqr, const VMC_Control_t *vmc,
                const Kalman_Observer_t *observer,
                const IMU_Data_t *imu, LESO_Control_t *leso,
                bool leso_learning_enabled)
{
    if (lqr == NULL || vmc == NULL || observer == NULL || imu == NULL ||
        leso == NULL) {
        return;
    }

    /* State order must match the offline linearization and k_table_2d.h. */
    lqr->x[0] = observer->outstate.s;
    lqr->x[1] = imu->YawTotalAngle * DEG2RAD;
    lqr->x[2] = imu->pitch * DEG2RAD;
    lqr->x[3] = vmc->left.theta[0];
    lqr->x[4] = vmc->right.theta[0];
    lqr->x[5] = observer->outstate.dot_s;
    lqr->x[6] = imu->gyro[2];
    lqr->x[7] = imu->gyro[1];
    lqr->x[8] = vmc->left.theta[1];
    lqr->x[9] = vmc->right.theta[1];

    /* Build the reference from commanded motion and equilibrium offsets. */
    memset(lqr->x_ref, 0, sizeof(lqr->x_ref));
    lqr->x_ref[0] = lqr->target.position_m + 3.7f;
    lqr->x_ref[1] = lqr->target.yaw_rad;
    lqr->x_ref[2] = lqr->target.pitch_rad;
    lqr->x_ref[3] = lqr->target.left_leg_theta_rad + e3_eval(vmc->left.length[0]);
    lqr->x_ref[4] = lqr->target.right_leg_theta_rad + e3_eval(vmc->right.length[0]);
    lqr->x_ref[5] = lqr->target.velocity_mps;
    lqr->x_ref[6] = lqr->target.yaw_rate_radps;

    k_table_2d_eval(vmc->left.length[0], vmc->right.length[0], lqr->K);
    ueq_table_2d_eval(vmc->left.length[0], vmc->right.length[0], lqr->u_eq);

    float leso_measurement[LQR_STATE_SIZE];
    memcpy(leso_measurement, lqr->x, sizeof(leso_measurement));
    leso_measurement[3] -= e3_eval(vmc->left.length[0]);
    leso_measurement[4] -= e3_eval(vmc->right.length[0]);
    LESO_Update(leso, leso_measurement,
                vmc->left.length[0], vmc->right.length[0],
                leso_learning_enabled);

    for (int output = 0; output < LQR_OUTPUT_SIZE; output++) {
        float control = lqr->u_eq[output];
        for (int state = 0; state < LQR_STATE_SIZE; state++) {
            control -= lqr->K[output * LQR_STATE_SIZE + state] *
                       (lqr->x[state] - lqr->x_ref[state]);
        }
        if (leso_learning_enabled) {
            control -= LESO_COMPENSATION_SCALE * leso->disturbance[output];
        }
        lqr->u[output] = control;
    }
}
