#include "Step_Climb.h"

#include <math.h>

/* BigEye 导轮圆柱中心、半径、半宽，相对双髋中点，X 前 Y 左 Z 上。 */
static const float guide_wheel_geometry_m[][5] = {
    { 0.2710f, -0.1140f, -0.130585f, 0.020f, 0.005f},
    {-0.2710f, -0.1140f, -0.128585f, 0.020f, 0.005f},
    {-0.2710f,  0.1140f, -0.128585f, 0.020f, 0.005f},
    { 0.2710f,  0.1140f, -0.130585f, 0.020f, 0.005f},
    { 0.1225f,  0.0875f, -0.157585f, 0.015f, 0.003f},
    { 0.1225f,  0.0705f, -0.157585f, 0.015f, 0.003f},
    { 0.1575f,  0.0875f, -0.147585f, 0.015f, 0.003f},
    { 0.1575f,  0.0705f, -0.147585f, 0.015f, 0.003f},
    { 0.1225f, -0.0705f, -0.157585f, 0.015f, 0.003f},
    { 0.1575f, -0.0705f, -0.147585f, 0.015f, 0.003f},
    { 0.1575f, -0.0845f, -0.147585f, 0.015f, 0.003f},
    { 0.1225f, -0.0845f, -0.157585f, 0.015f, 0.003f},
    { 0.0510f,  0.0845f, -0.167585f, 0.015f, 0.003f},
    { 0.0510f,  0.0705f, -0.167585f, 0.015f, 0.003f},
    {-0.0510f,  0.0705f, -0.167585f, 0.015f, 0.003f},
    {-0.0510f,  0.0845f, -0.167585f, 0.015f, 0.003f},
    { 0.0510f, -0.0845f, -0.167585f, 0.015f, 0.003f},
    { 0.0510f, -0.0705f, -0.167585f, 0.015f, 0.003f},
    {-0.0510f, -0.0705f, -0.167585f, 0.015f, 0.003f},
    {-0.0510f, -0.0845f, -0.167585f, 0.015f, 0.003f},
};

static const float gravity_mps2 = 9.81f;
static const float robot_mass_kg = 22.0f; /* 与 H723 当前支撑控制的质量初值一致。 */

typedef struct {
    float x, z;          /* 轮心在水平前向与竖直方向的投影。 */
    float vx, vz;        /* 扣除整机平移后的轮心速度。 */
    float hip_x, hip_z;
    float jx[2], jz[2];  /* 前髋、后髋对轮心位置的雅可比。 */
} StepWheelPose_t;

typedef struct {
    float forward_accel;
    float deck;         /* 导轮轮廓最低点的竖直投影。 */
    StepWheelPose_t wheel[2];
} StepObservation_t;

static float clip(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

static float dot3(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void StepClimb_Reset(StepClimb_t *step)
{
    *step = (StepClimb_t){.support = 1.0f, .wheel_load = {0.5f, 0.5f}};
}

static void finish_step(StepClimb_t *step, StepClimb_Result_t result)
{
    step->phase = STEP_IDLE;
    step->result = result;
    step->cooldown_s = 0.45f;
    step->wheel_odometry_valid = false;
    step->torque = (SelfRighting_Torque_t){0};
    step->approach_s = 0.0f;
    step->impact_s = 0.0f;
    step->unload_s = 0.0f;
    step->confirm_s = 0.0f;
}

/* 将 IMU 比力和腿部力矩投影到同一方向，估算整机支撑和驱动轮承重。 */
static bool estimate_step_support(StepClimb_t *step, const StepClimb_Input_t *input,
                    const SelfRighting_Config_t *geometry, StepObservation_t *observation, float dt)
{
    const float *q = input->quat_wxyz;
    const float norm = dot3(q+1, q+1) + q[0]*q[0];
    if (!(norm > 0.25f && norm < 4.0f)) {
        return false;
    }
    /* 从四元数求世界竖直方向，再把车头轴投影到水平面。 */
    const float k = 2.0f / norm;
    const float up[3] = {k*(q[1]*q[3]-q[0]*q[2]), k*(q[2]*q[3]+q[0]*q[1]),
                         1.0f-k*(q[1]*q[1]+q[2]*q[2])};
    const float forward_norm = sqrtf(fmaxf(1.0f-up[0]*up[0], 0.0f));
    if (forward_norm < 0.15f) {
        return false;
    }
    const float forward[3] = {forward_norm, -up[0]*up[1]/forward_norm, -up[0]*up[2]/forward_norm};
    observation->forward_accel = dot3(forward, input->accel_mps2);
    const float body_support = dot3(up, input->accel_mps2) / gravity_mps2;
    if (!isfinite(observation->forward_accel) || !isfinite(body_support)) {
        return false;
    }
    /* 导轮包络提供轮心抬升的参考高度，实际承重由力矩和比力确认。 */
    observation->deck = INFINITY;
    const float radius_projection = sqrtf(up[0]*up[0] + up[2]*up[2]);
    for (unsigned i = 0; i < sizeof(guide_wheel_geometry_m)/sizeof(guide_wheel_geometry_m[0]); ++i) {
        observation->deck = fminf(observation->deck, dot3(guide_wheel_geometry_m[i], up)
                         - guide_wheel_geometry_m[i][3]*radius_projection - guide_wheel_geometry_m[i][4]*fabsf(up[1]));
    }

    const float alpha = dt / (0.012f + dt);
    float load[2];
    for (unsigned side = 0; side < 2; ++side) {
        StepWheelPose_t *wheel = &observation->wheel[side];
        const VMC_LegSite_t *leg = &input->vmc->left;
        const float (*jt)[2] = input->vmc->JRM_l;
        if (side == 1U) {
            leg = &input->vmc->right;
            jt = input->vmc->JRM_r;
        }
        const VMC_AirSpring_t *spring = &input->vmc->spring[side];
        const float c = cosf(leg->phi), s = sinf(leg->phi), length = leg->length[0];
        const float *hip = geometry->hip_origin_m[side];
        const float center[3] = {hip[0]+length*c, hip[1]+geometry->wheel_lateral_m[side], hip[2]-length*s};
        wheel->x = dot3(forward, center);
        wheel->z = dot3(up, center);
        wheel->hip_x = wheel->x - length*(forward[0]*c-forward[2]*s);
        wheel->hip_z = wheel->z - length*(up[0]*c-up[2]*s);
        const float *g = input->gyro_radps;
        const float spin[3] = {g[1]*center[2]-g[2]*center[1],
                               g[2]*center[0]-g[0]*center[2],
                               g[0]*center[1]-g[1]*center[0]};
        const float body_theta_rate = leg->theta[1] - g[1];
        const float rel_x = c*leg->length[1] - length*s*body_theta_rate;
        const float rel_z = -s*leg->length[1] - length*c*body_theta_rate;
        wheel->vx = dot3(forward, spin) + forward[0]*rel_x + forward[2]*rel_z;
        wheel->vz = dot3(up, spin) + up[0]*rel_x + up[2]*rel_z;
        if (!isfinite(wheel->x) || !isfinite(wheel->z) ||
            !isfinite(wheel->vx) || !isfinite(wheel->vz)) {
            return false;
        }
        for (unsigned joint = 0; joint < 2; ++joint) {
            const float dx = c*jt[joint][0] - length*s*jt[joint][1];
            const float dz = -s*jt[joint][0] - length*c*jt[joint][1];
            if (!isfinite(dx) || !isfinite(dz) || fabsf(dx) > 2.0f || fabsf(dz) > 2.0f) {
                return false;
            }
            wheel->jx[joint] = forward[0]*dx + forward[2]*dz;
            wheel->jz[joint] = up[0]*dx + up[2]*dz;
        }

        /* 正则化反解 J^T F = tau，向下推轮子的分力对应轮地向上反力。 */
        const float *jx = wheel->jx, *jz = wheel->jz;
        const float xx = jx[0]*jx[0] + jx[1]*jx[1] + 0.0004f;
        const float xz = jx[0]*jz[0] + jx[1]*jz[1];
        const float zz = jz[0]*jz[0] + jz[1]*jz[1] + 0.0004f;
        const float tf = input->hip_torque_nm[side] + spring->torque_front_nm;
        const float tb = input->hip_torque_nm[side+2] + spring->torque_back_nm;
        const float fz = (xx*(jz[0]*tf+jz[1]*tb) - xz*(jx[0]*tf+jx[1]*tb)) / (xx*zz-xz*xz);
        if (!isfinite(fz)) {
            return false;
        }
        load[side] = clip(-fz / (robot_mass_kg*gravity_mps2), 0.0f, 1.0f);
    }
    /* 两侧轮载都有效才一起更新，避免单侧错误污染本拍触发条件。 */
    step->support += alpha*(clip(body_support, 0.0f, 2.0f)-step->support);
    for (unsigned i = 0; i < 2; ++i) {
        step->wheel_load[i] += alpha*(load[i]-step->wheel_load[i]);
    }
    return true;
}

void StepClimb_Update(StepClimb_t *step, const StepClimb_Input_t *input,
                      const SelfRighting_Config_t *geometry, float dt)
{
    step->cooldown_s = fmaxf(0.0f, step->cooldown_s-dt);
    if (step->phase == STEP_IDLE && (!input->armed || step->cooldown_s > 0.0f)) {
        step->approach_s = 0.0f;
        step->impact_s = 0.0f;
        step->unload_s = 0.0f;
        step->confirm_s = 0.0f;
        return;
    }
    StepObservation_t observation;
    if (!input->feedback_valid || !isfinite(input->speed_mps) ||
        !isfinite(input->roll_rad) || !isfinite(input->pitch_rad) || !estimate_step_support(step, input, geometry, &observation, dt)) {
        finish_step(step, STEP_BAD_FEEDBACK);
        return;
    }
    const float total_load = step->wheel_load[0]+step->wheel_load[1];
    const float min_load = fminf(step->wheel_load[0], step->wheel_load[1]);
    /* 整机有支撑、驱动轮却卸载时，剩余支撑可能来自导轮或车体。 */
    const float body_support = step->support - total_load;
    const float wheel_bottom = fminf(observation.wheel[0].z, observation.wheel[1].z)-geometry->wheel_radius_m;
    if (step->phase == STEP_IDLE) {
        /* 依次确认有载荷地接近台阶、受到反向冲击、机身托住且驱动轮卸载。 */
        step->approach_s = fmaxf(0.0f, step->approach_s-dt);
        step->impact_s = fmaxf(0.0f, step->impact_s-dt);
        step->unload_s = fmaxf(0.0f, step->unload_s-dt);
        if (step->impact_s == 0.0f && fabsf(input->speed_mps) > 0.35f &&
            total_load > 0.65f && step->support > 0.65f) {
            step->direction = copysignf(1.0f, input->speed_mps);
            step->approach_s = 0.12f;
        }
        if (step->approach_s > 0.0f && step->impact_s == 0.0f) {
            if (step->support < 0.55f) {
                step->unload_s = 0.12f;
            }
            if (step->direction * observation.forward_accel < -7.0f) {
                step->impact_s = 0.09f;
            }
        }
        const bool total_load_low = total_load < 0.60f;
        const bool one_wheel_unloaded = min_load < 0.12f && body_support > 0.55f;
        const bool body_caught_after_unload =
            step->unload_s > 0.0f && step->support > 1.15f && body_support > 0.43f;
        const bool wheel_unload_detected =
            total_load_low || one_wheel_unloaded || body_caught_after_unload;

        const bool body_pose_valid = fabsf(input->roll_rad) < 0.30f && fabsf(input->pitch_rad) < 0.50f;
        const bool wheels_below_guides = observation.deck - wheel_bottom > 0.065f;
        const bool step_detected = step->impact_s > 0.0f && body_pose_valid && wheels_below_guides &&
                                   step->support > 0.60f && wheel_unload_detected && body_support > 0.33f;
        if (step_detected) {
            step->confirm_s += dt;
        } else {
            step->confirm_s = 0.0f;
        }
        if (step->confirm_s < 0.008f) {
            return;
        }

        step->phase = STEP_LIFT;
        step->result = STEP_NONE;
        step->elapsed_s = 0.0f;
        step->landed_s = 0.0f;
        step->lift_travel_m = 0.0f;
        step->lift_x[0] = observation.wheel[0].x;
        step->lift_x[1] = observation.wheel[1].x;
    }

    step->elapsed_s += dt;
    if (fabsf(input->roll_rad) > 0.65f || fabsf(input->pitch_rad) > 0.85f) {
        finish_step(step, STEP_UNSTABLE);
        return;
    }
    if (step->elapsed_s > 0.95f) {
        finish_step(step, STEP_TIMEOUT);
        return;
    }
    if (step->phase == STEP_CAPTURE) {
        /* 进入捕获后由平衡环控制，轮速重新可用并稳定后才退出台阶模式。 */
        step->capture_s += dt;
        step->wheel_odometry_valid = min_load > 0.12f && step->support > 0.50f;
        if (step->capture_s >= 0.08f && step->wheel_odometry_valid && fabsf(input->pitch_rad) < 0.14f) {
            finish_step(step, STEP_COMPLETE);
        } else if (step->capture_s >= 0.32f) {
            finish_step(step, STEP_TIMEOUT);
        }
        return;
    }

    const float goal_x = -0.025f*step->direction;
    const float horizontal_error = fmaxf(fabsf(observation.wheel[0].x-goal_x), fabsf(observation.wheel[1].x-goal_x));
    /* 两个判定按顺序执行，已满足条件时允许在同一拍跨过中间阶段。 */
    if (step->phase == STEP_LIFT && wheel_bottom - observation.deck >= 0.023f) {
        step->phase = STEP_SWING;
    }
    if (step->phase == STEP_SWING && horizontal_error < 0.12f) {
        step->phase = STEP_PLANT;
    }
    /* 同时确认承重、轮心速度和姿态，防止轮子刚碰到台阶就交接。 */
    const bool planted = step->phase == STEP_PLANT &&
                         min_load > 0.17f && step->support > 0.65f &&
                         fmaxf(fabsf(observation.wheel[0].vz), fabsf(observation.wheel[1].vz)) < 0.45f &&
                         fabsf(input->vmc->left.theta[0]) < 0.43f &&
                         fabsf(input->vmc->right.theta[0]) < 0.43f &&
                         fabsf(input->pitch_rad) < 0.30f;
    if (planted) {
        step->landed_s += dt;
    } else {
        step->landed_s = 0.0f;
    }
    if (step->landed_s >= 0.020f && fabsf(input->gyro_radps[1]) < 2.0f) {
        step->phase = STEP_CAPTURE;
        step->capture_s = 0.0f;
        step->wheel_odometry_valid = true;
        return;
    }

    /* 抬轮先留出越阶高度，回摆时逐步降低，放轮时再主动压向台面。 */
    float clearance = 0.053f;
    if (step->phase == STEP_LIFT) {
        const float speed = 0.6f*clip(input->speed_mps*step->direction, 0.0f, 1.6f)*step->direction;
        step->lift_travel_m = clip(step->lift_travel_m+speed*dt, -0.16f, 0.16f);
    } else if (step->phase == STEP_SWING) {
        clearance = clip(0.053f-fmaxf(0.0f, 0.16f-horizontal_error)*0.65f, 0.008f, 0.053f);
    } else {
        clearance = -0.042f;
    }
    /* 用轮心位置误差生成速度参考，再把跟踪力映射到髋关节。 */
    const float target_z = observation.deck + geometry->wheel_radius_m + clearance;
    const float min_length = geometry->min_length_m + 0.003f;
    const float max_length = geometry->swing_length_m - 0.005f;
    for (unsigned side = 0; side < 2; ++side) {
        const StepWheelPose_t *wheel = &observation.wheel[side];
        float target_x = goal_x;
        if (step->phase == STEP_LIFT) {
            target_x = step->lift_x[side] - step->lift_travel_m;
        }
        float dx = target_x - wheel->hip_x;
        const float dz = target_z - wheel->hip_z;
        if (step->phase == STEP_LIFT && fabsf(dz) < min_length) {
            /* 抬轮时从来向一侧绕开最短腿长的不可达区域。 */
            const float min_dx = sqrtf(fmaxf(min_length*min_length - dz*dz, 0.0f));
            dx = -step->direction * fmaxf(-step->direction*dx, min_dx);
        }

        const float length = sqrtf(dx*dx + dz*dz);
        const float scale = clip(length, min_length, max_length) / fmaxf(length, 0.001f);
        const float target_vx = clip(18.0f*(wheel->hip_x + dx*scale - wheel->x), -2.2f, 2.2f);
        const float target_vz = clip(18.0f*(wheel->hip_z + dz*scale - wheel->z), -0.45f, 2.8f);
        const float force_x = 135.0f * (target_vx - wheel->vx);
        const float force_z = 140.0f * (target_vz - wheel->vz);
        const VMC_AirSpring_t *spring = &input->vmc->spring[side];
        const float limit = geometry->hip_limit_nm;
        const float front_torque = wheel->jx[0]*force_x + wheel->jz[0]*force_z - spring->torque_front_nm;
        const float rear_torque = wheel->jx[1]*force_x + wheel->jz[1]*force_z - spring->torque_back_nm;
        step->torque.hip[side] = clip(front_torque, -limit, limit);
        step->torque.hip[side + 2] = clip(rear_torque, -limit, limit);
    }
}
