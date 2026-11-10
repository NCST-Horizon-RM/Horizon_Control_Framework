#include "Self_Righting.h"

#include <math.h>
#include <string.h>

#define SR_PI 3.14159265358979323846f
#define SR_BLEND_S 0.040f         /* 力矩偏差衰减时间。 */
#define SR_LENGTH_BLEND_S 0.080f  /* 从实际腿长过渡到捕获腿长的时间。 */
#define SR_COOLDOWN_S 0.200f      /* 交接结束后屏蔽重复触发的时间。 */

typedef struct {
    float wheel_xz[2];           /* 轮心相对髋轴的水平和竖直位移，单位米。 */
    float wheel_speed;           /* 轮心速度模长，单位米每秒。 */
    float wheel_velocity_xz[2];  /* 机身转动和腿关节运动产生的轮心速度。 */
    float jacobian_xz[2][2];     /* 行为水平和竖直方向，列为前髋和后髋。 */
    float body_xz[2];            /* 机体系下轮心相对髋轴的前向和竖直位移。 */
    float length;
    float theta;                 /* 虚拟腿角，轮心竖直向下时为零。 */
    float rest;                  /* 机身贴地时髋轴距地面的高度。 */
    float clearance;             /* 轮底低于机身最低点的距离，轮底悬高时为负。 */
} RecoveryLeg_t;

typedef struct {
    float up[3];          /* 世界竖直方向在机体系中的表示。 */
    float forward[3];     /* 规划前向在机体系中的表示。 */
    float speed;          /* 机身角速度模长，单位弧度每秒。 */
    float pitch_rate;
    float q[4], dq[4];    /* 换算到统一方向的髋关节角与角速度。 */
    float max_angle, max_foot_speed;
    bool can_align_on_body;
    RecoveryLeg_t leg[2];
} RecoveryObservation_t;

static float clamp(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

/* 五次曲线使轨迹的起止速度和加速度均为零。 */
static float quintic_blend(float u)
{
    u = clamp(u, 0.0f, 1.0f);
    return u*u*u*(10.0f + u*(-15.0f + 6.0f*u));
}

static float dot3(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static bool finite_array(const float *x, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(x[i])) {
            return false;
        }
    }
    return true;
}

void SelfRighting_DefaultConfig(SelfRighting_Config_t *cfg)
{
    if (cfg == NULL) {
        return;
    }
    *cfg = (SelfRighting_Config_t){
        .min_length_m = VMC_LEG_MIN_LENGTH_M,
        .swing_length_m = VMC_LEG_MAX_LENGTH_M,
        .capture_length_m = 0.150f,
        .wheel_radius_m = 0.060f,
        .wheel_effective_mass_kg = 0.353f,
        .timeout_s = 10.0f
    };
}

void SelfRighting_Reset(SelfRighting_t *recovery)
{
    if (recovery == NULL) {
        return;
    }
    const SelfRighting_Config_t cfg = recovery->cfg;
    const bool configured = recovery->configured;
    memset(recovery, 0, sizeof(*recovery));
    recovery->cfg = cfg;
    recovery->configured = configured;
    recovery->forward[0] = 1.0f;
    recovery->phase = SR_WATCH;
}

bool SelfRighting_Init(SelfRighting_t *recovery, const SelfRighting_Config_t *cfg)
{
    if (recovery == NULL) {
        return false;
    }
    memset(recovery, 0, sizeof(*recovery));
    if (cfg == NULL || cfg->support_points_m == NULL || cfg->support_count < 3 ||
        !isfinite(cfg->min_length_m) || !isfinite(cfg->swing_length_m) ||
        !isfinite(cfg->capture_length_m) || !isfinite(cfg->wheel_radius_m) ||
        !isfinite(cfg->wheel_effective_mass_kg) || cfg->wheel_effective_mass_kg < 0.0f ||
        !isfinite(cfg->hip_limit_nm) || !isfinite(cfg->wheel_limit_nm) ||
        !isfinite(cfg->timeout_s) || cfg->timeout_s <= 0.0f ||
        cfg->min_length_m < VMC_LEG_MIN_LENGTH_M ||
        cfg->swing_length_m > VMC_LEG_MAX_LENGTH_M ||
        cfg->capture_length_m < cfg->min_length_m ||
        cfg->capture_length_m >= cfg->swing_length_m ||
        cfg->wheel_radius_m <= 0.0f || cfg->hip_limit_nm <= 0.0f ||
        cfg->wheel_limit_nm <= 0.0f || !finite_array(cfg->wheel_lateral_m, 2)) {
        return false;
    }
    for (size_t i = 0; i < cfg->support_count; ++i) {
        if (!finite_array(cfg->support_points_m[i], 3)) {
            return false;
        }
    }
    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(cfg->hip_min_rad[i]) || !isfinite(cfg->hip_max_rad[i]) ||
            cfg->hip_min_rad[i] >= cfg->hip_max_rad[i]) {
            return false;
        }
    }
    if (!finite_array(cfg->hip_origin_m[0], 3) ||
        !finite_array(cfg->hip_origin_m[1], 3)) {
        return false;
    }
    recovery->cfg = *cfg;
    recovery->configured = true;
    SelfRighting_Reset(recovery);
    return true;
}

static bool observe_body_and_legs(SelfRighting_t *recovery, const SelfRighting_Input_t *input, RecoveryObservation_t *observation)
{
    const float *quat = input->quat_wxyz;
    const float norm = quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
    if (!(norm > 0.25f && norm < 4.0f)) {
        return false;
    }
    /* 直接用四元数建立投影基，避免倒扣时通过欧拉角判断朝向。 */
    const float k = 2.0f/norm;
    const float w = quat[0], x = quat[1], y = quat[2], z = quat[3];
    const float r0[3] = {1.0f-k*(y*y+z*z), k*(x*y-w*z), k*(x*z+w*y)};
    const float r1[3] = {k*(x*y+w*z), 1.0f-k*(x*x+z*z), k*(y*z-w*x)};
    observation->up[0] = k*(x*z-w*y);
    observation->up[1] = k*(y*z+w*x);
    observation->up[2] = 1.0f-k*(x*x+y*y);
    /* 车头近竖直时水平投影不可靠。翻身期间也沿用进入前的规划方向。 */
    const float forward_norm = sqrtf(r0[0]*r0[0]+r1[0]*r1[0]);
    const bool flipping = recovery->phase >= SR_FLIP_PLACE && recovery->phase <= SR_FLIP_BRAKE;
    if (forward_norm > 0.15f && !flipping) {
        recovery->forward[0] = r0[0]/forward_norm;
        recovery->forward[1] = r1[0]/forward_norm;
    }
    for (unsigned j = 0; j < 3; ++j) {
        observation->forward[j] = recovery->forward[0]*r0[j] + recovery->forward[1]*r1[j];
    }
    const float *g = input->gyro_radps;
    const float roll_proj = sqrtf(observation->up[1]*observation->up[1] + observation->up[2]*observation->up[2]);
    observation->pitch_rate = g[1];
    if (roll_proj > 0.10f) {
        observation->pitch_rate = (observation->up[2]*g[1] - observation->up[1]*g[2]) / roll_proj;
    }
    observation->speed = sqrtf(dot3(g, g));
    /* 将机身轮廓投影到竖直方向，得到当前姿态下的几何支撑高度。 */
    float bottom = INFINITY;
    for (size_t i = 0; i < recovery->cfg.support_count; ++i) {
        bottom = fminf(bottom, dot3(recovery->cfg.support_points_m[i], observation->up));
    }
    for (unsigned i = 0; i < 4; ++i) {
        /* 反馈顺序为左前、右前、左后、右后，右侧先取反。 */
        float sign = 1.0f;
        if (i % 2U == 1U) {
            sign = -1.0f;
        }
        observation->q[i] = sign*input->motor_pos_rad[i];
        observation->dq[i] = sign*input->motor_vel_radps[i];
        if (observation->q[i] < recovery->cfg.hip_min_rad[i] || observation->q[i] > recovery->cfg.hip_max_rad[i]) {
            return false;
        }
    }
    for (unsigned i = 0; i < 2; ++i) {
        RecoveryLeg_t *leg = &observation->leg[i];
        const VMC_LegSite_t *site = &input->vmc->left;
        const float (*jt)[2] = input->vmc->JRM_l;
        if (i == 1U) {
            site = &input->vmc->right;
            jt = input->vmc->JRM_r;
        }
        const VMC_AirSpring_t *spring = &input->vmc->spring[i];
        leg->length = site->length[0];
        if (!isfinite(leg->length) || leg->length < 0.03f || !isfinite(site->phi) ||
            !finite_array(jt[0], 2) || !finite_array(jt[1], 2) ||
            !isfinite(spring->torque_front_nm) || !isfinite(spring->torque_back_nm)) {
            return false;
        }
        const float local[3] = {leg->length*cosf(site->phi), recovery->cfg.wheel_lateral_m[i],
                                -leg->length*sinf(site->phi)};
        leg->body_xz[0] = local[0];
        leg->body_xz[1] = local[2];
        leg->wheel_xz[0] = dot3(observation->forward, local);
        leg->wheel_xz[1] = dot3(observation->up, local);
        leg->theta = atan2f(-leg->wheel_xz[0], -leg->wheel_xz[1]);
        leg->rest = -bottom + dot3(observation->up, recovery->cfg.hip_origin_m[i]);
        leg->clearance = recovery->cfg.wheel_radius_m - leg->wheel_xz[1] - leg->rest;
        /* 先计入机身角速度与轮心位置的叉积，再叠加髋关节运动。 */
        float velocity[3] = {g[1]*local[2]-g[2]*local[1],
                             g[2]*local[0]-g[0]*local[2],
                             g[0]*local[1]-g[1]*local[0]};
        for (unsigned j = 0; j < 2; ++j) {
            const float dx = local[0]/leg->length*jt[j][0] + local[2]*jt[j][1];
            const float dz = local[2]/leg->length*jt[j][0] - local[0]*jt[j][1];
            leg->jacobian_xz[0][j] = observation->forward[0]*dx + observation->forward[2]*dz;
            leg->jacobian_xz[1][j] = observation->up[0]*dx + observation->up[2]*dz;
            if (!isfinite(dx) || !isfinite(dz) || fabsf(dx) > 2.0f || fabsf(dz) > 2.0f) {
                return false;
            }
            velocity[0] += dx*observation->dq[i+2*j];
            velocity[2] += dz*observation->dq[i+2*j];
        }
        leg->wheel_velocity_xz[0] = dot3(observation->forward, velocity);
        leg->wheel_velocity_xz[1] = dot3(observation->up, velocity);
        observation->max_angle = fmaxf(observation->max_angle, fabsf(leg->theta));
        const float vx = leg->wheel_velocity_xz[0];
        const float vz = leg->wheel_velocity_xz[1];
        leg->wheel_speed = sqrtf(vx*vx + vz*vz);
        observation->max_foot_speed = fmaxf(observation->max_foot_speed, leg->wheel_speed);
        if (recovery->angle_valid) {
            recovery->theta[i] += VMC_WrapAngle(leg->theta - recovery->previous_theta[i]);
        } else {
            recovery->theta[i] = leg->theta;
        }
        recovery->previous_theta[i] = leg->theta;
    }
    recovery->angle_valid = true;
    /* 腿短且轮在髋轴下方时，可借直立机身支撑对正。 */
    observation->can_align_on_body = observation->up[2] > 0.90f &&
           fmaxf(observation->leg[0].length, observation->leg[1].length) < recovery->cfg.min_length_m+0.042f &&
           fmaxf(observation->leg[0].wheel_xz[1], observation->leg[1].wheel_xz[1]) < -0.025f;
    return true;
}

/* 每次换阶段都重新开始计时和卡滞检测。 */
static void enter_phase(SelfRighting_t *recovery, SelfRighting_Phase_t phase)
{
    recovery->phase = phase;
    recovery->t = 0.0f;
    recovery->no_progress = 0.0f;
    recovery->best = INFINITY;
}

/* 误差至少降低指定幅度才算有进展，过滤关节测量抖动。 */
static bool progress_stalled(SelfRighting_t *recovery, float error, float step, float window, float dt)
{
    if (error < recovery->best-step) {
        recovery->best = error;
        recovery->no_progress = 0.0f;
    } else {
        recovery->no_progress += dt;
    }
    return recovery->no_progress >= window;
}

static void map_wheel_force_to_hip_torque(const SelfRighting_Input_t *input, const RecoveryLeg_t *leg, unsigned i,
                          float fx, float fz, SelfRighting_Torque_t *out)
{
    const VMC_AirSpring_t *spring = &input->vmc->spring[i];
    out->hip[i] = leg->jacobian_xz[0][0]*fx + leg->jacobian_xz[1][0]*fz +
                  input->leg_gravity_nm[i] - spring->torque_front_nm;
    out->hip[i+2] = leg->jacobian_xz[0][1]*fx + leg->jacobian_xz[1][1]*fz +
                    input->leg_gravity_nm[i+2] - spring->torque_back_nm;
}

static void apply_joint_damping(const SelfRighting_Input_t *input, const RecoveryObservation_t *observation, SelfRighting_Torque_t *out)
{
    *out = (SelfRighting_Torque_t){0};
    for (unsigned i = 0; i < 2; ++i) {
        map_wheel_force_to_hip_torque(input, &observation->leg[i], i, 0.0f, 0.0f, out);
    }
    for (unsigned i = 0; i < 4; ++i) {
        out->hip[i] -= 2.0f*observation->dq[i];
    }
}

static bool track_wheel_pose(SelfRighting_t *recovery, const SelfRighting_Input_t *input,
                        const RecoveryObservation_t *observation, unsigned side, const float target[2],
                        float kp, float kd, float dt, SelfRighting_Torque_t *out)
{
    const float a = observation->forward[0], b = observation->forward[2], c = observation->up[0], d = observation->up[2];
    /* 把水平和竖直目标投影回腿平面，接近奇异姿态时偏向当前轮心。 */
    const float dy = recovery->cfg.wheel_lateral_m[side];
    const float x = target[0] - observation->forward[1]*dy, z = target[1] - observation->up[1]*dy;
    const float damping = 0.0025f;
    const float m00 = a*a+c*c+damping, m01 = a*b+c*d, m11 = b*b+d*d+damping;
    const float det = m00*m11-m01*m01;
    const float bx = a*x+c*z+damping*observation->leg[side].body_xz[0];
    const float bz = b*x+d*z+damping*observation->leg[side].body_xz[1];
    const float local_x = (m11*bx-m01*bz)/det, local_z = (m00*bz-m01*bx)/det;
    const float length = clamp(sqrtf(local_x*local_x+local_z*local_z),
                              recovery->cfg.min_length_m, recovery->cfg.swing_length_m);
    float q[2];
    if (!VMC_InverseKinematicsContinuous(input->vmc, length, atan2f(-local_x, -local_z), 0.0f,
                                         recovery->ref_q[side], recovery->ref_q[side+2], &q[0], &q[1])) {
        return false;
    }
    for (unsigned j = 0; j < 2; ++j) {
        const unsigned i = side+2*j;
        if (q[j] < recovery->cfg.hip_min_rad[i] || q[j] > recovery->cfg.hip_max_rad[i]) {
            return false;
        }
    }
    const VMC_AirSpring_t *spring = &input->vmc->spring[side];
    /* 连续关节参考从上一拍推进，避免逆解选圈变化造成力矩突变。 */
    const float spring_torque_nm[2] = {spring->torque_front_nm, spring->torque_back_nm};
    for (unsigned j = 0; j < 2; ++j) {
        const unsigned i = side+2*j;
        const float rate = clamp((q[j]-recovery->ref_q[i])/dt, -8.0f, 8.0f);
        recovery->ref_q[i] += rate*dt;
        out->hip[i] = kp*clamp(recovery->ref_q[i]-observation->q[i], -0.55f, 0.55f)
                      + kd*(rate-observation->dq[i]) + input->leg_gravity_nm[i] - spring_torque_nm[j];
    }
    return true;
}

static float reachable_ground_x(const SelfRighting_t *recovery, const RecoveryLeg_t *leg, float x)
{
    const float length = recovery->cfg.min_length_m + 0.002f;
    const float offset = sqrtf(fmaxf(0.0f, length*length-leg->wheel_xz[1]*leg->wheel_xz[1]));
    return copysignf(fmaxf(fabsf(x), offset), x);
}

static void hip_torque_bounds(const SelfRighting_t *recovery, unsigned i, float *lo, float *hi)
{
    const float limit = recovery->cfg.hip_limit_nm;
    /* 在机械限位前减小向外力矩，保留向内回退力矩。 */
    *lo = -limit*clamp((recovery->current_q[i]-recovery->cfg.hip_min_rad[i])/0.03f, 0.0f, 1.0f);
    *hi = limit*clamp((recovery->cfg.hip_max_rad[i]-recovery->current_q[i])/0.03f, 0.0f, 1.0f);
}

static void pull_wheel_along_ground(const SelfRighting_t *recovery, const SelfRighting_Input_t *input,
                          const RecoveryLeg_t *leg, unsigned i, float x, float vx, SelfRighting_Torque_t *out)
{
    const float error = reachable_ground_x(recovery, leg, x)-leg->wheel_xz[0];
    const float extra = copysignf(700.0f*fmaxf(0.0f, fabsf(error)-0.035f), error);
    float fx = clamp(600.0f*error + extra + 45.0f*(vx-leg->wheel_velocity_xz[0]), -110.0f, 110.0f);
    if (leg->length < recovery->cfg.min_length_m && fx*leg->wheel_xz[0] < 0.0f) {
        fx = copysignf(fminf(20.0f, 1500.0f*(recovery->cfg.min_length_m-leg->length)), leg->wheel_xz[0]);
    }
    /* 先给气弹簧补偿和贴地预压留力矩，再按两髋共同余量缩放收轮力。 */
    map_wheel_force_to_hip_torque(input, leg, i, 0.0f, -3.0f, out);
    float scale = 1.0f;
    for (unsigned j = 0; j < 2; ++j) {
        const unsigned k = i+2*j;
        const float bias = out->hip[k], tracking = leg->jacobian_xz[0][j]*fx;
        float lo, hi;
        hip_torque_bounds(recovery, k, &lo, &hi);
        if (bias < lo || bias > hi) {
            /* 单独补偿已超出能力，保留原合成力矩，由最终限幅处理。 */
            scale = 1.0f;
            break;
        }
        if (tracking > hi-bias) {
            scale = fminf(scale, (hi-bias)/tracking);
        }
        if (tracking < lo-bias) {
            scale = fminf(scale, (lo-bias)/tracking);
        }
    }
    for (unsigned j = 0; j < 2; ++j) {
        out->hip[i+2*j] += leg->jacobian_xz[0][j]*(fx*scale);
    }
    out->wheel[i] = 0.0f;
}

static void save_trajectory_start(SelfRighting_t *recovery, const RecoveryObservation_t *observation)
{
    for (unsigned i = 0; i < 2; ++i) {
        memcpy(recovery->starts[i], observation->leg[i].wheel_xz, sizeof(recovery->starts[i]));
        recovery->start_length[i] = observation->leg[i].length;
    }
    memcpy(recovery->ref_q, observation->q, sizeof(recovery->ref_q));
}

typedef enum {
    RETRACT_ON_GROUND,
    ALIGN_ON_BODY,
    ALIGN_ON_TILTED_BODY,
} RetractionRoute_t;

/* 以当前轮心为起点，按支撑方式进入沿地收轮或机身对正。 */
static void start_retraction(SelfRighting_t *recovery,
                             const RecoveryObservation_t *observation,
                             RetractionRoute_t route)
{
    save_trajectory_start(recovery, observation);
    if (route != RETRACT_ON_GROUND) {
        recovery->support = true;
        recovery->support_tried = true;
        recovery->tilted = route == ALIGN_ON_TILTED_BODY;
        recovery->damp_pitch = recovery->tilted && fabsf(observation->up[1]) < 0.22f;
        enter_phase(recovery, SR_ROLL_IN);
        return;
    }

    recovery->support = false;
    float distance = 0.0f;
    for (unsigned i = 0; i < 2; ++i) {
        const RecoveryLeg_t *leg = &observation->leg[i];
        const float vertical = fmaxf(0.015f, leg->rest-recovery->cfg.wheel_radius_m);
        const float length = recovery->cfg.min_length_m;
        const float x = sqrtf(fmaxf(0.0f, length*length-vertical*vertical));
        const bool positive = (recovery->route[i] == SR_GROUND_SHORT || recovery->retries > 0U) &&
                              leg->wheel_xz[0] > 0.0f;
        recovery->roll_target[i] = -fmaxf(0.015f, x);
        if (positive) {
            recovery->roll_target[i] = -recovery->roll_target[i];
        }
        distance = fmaxf(distance, fabsf(recovery->roll_target[i]-leg->wheel_xz[0]));
    }
    recovery->roll_duration = fmaxf(0.25f, distance/0.60f);
    enter_phase(recovery, SR_ROLL_IN);
}

static void retry_or_block(SelfRighting_t *recovery, SelfRighting_Reason_t reason, const RecoveryObservation_t *observation)
{
    recovery->reason = reason;
    if (recovery->retries >= 2U) {
        memcpy(recovery->blocked_up, observation->up, sizeof(recovery->blocked_up));
        memcpy(recovery->blocked_q, observation->q, sizeof(recovery->blocked_q));
        enter_phase(recovery, SR_BLOCKED);
        return;
    }

    ++recovery->retries;
    if (recovery->phase == SR_ROLL_IN && !recovery->support && reason == SR_NO_PROGRESS) {
        start_retraction(recovery, observation, RETRACT_ON_GROUND);
        return;
    }
    recovery->angle_valid = false;
    enter_phase(recovery, SR_SETTLE);
}

static bool update_retraction(SelfRighting_t *recovery, const SelfRighting_Input_t *input,
                     const RecoveryObservation_t *observation, float dt, SelfRighting_Output_t *out)
{
    const float upright_cos = observation->up[2];
    bool ready;
    if (recovery->support) {
        /* 借机身支撑对正时，倾斜姿态需要更长的轨迹和等待时间。 */
        float align_duration_s = 0.26f;
        float align_timeout_s = 0.58f;
        float upright_threshold = 0.90f;
        if (recovery->tilted) {
            align_duration_s = 0.35f;
            align_timeout_s = 1.05f;
            upright_threshold = 0.92f;
        }

        float position_gain = 90.0f;
        float velocity_gain = 3.0f;
        if (recovery->damp_pitch) {
            position_gain = 55.0f;
            velocity_gain = 3.5f;
        }

        const float b = quintic_blend(recovery->t / align_duration_s);
        for (unsigned i = 0; i < 2; ++i) {
            float x = clamp(recovery->starts[i][0], -0.038f, 0.038f);
            float z = -0.147f;
            if (recovery->tilted) {
                float extension_below_body_m = 0.09f;
                if (recovery->damp_pitch) {
                    x = -0.26f*observation->up[0] + 0.08f*observation->pitch_rate;
                    extension_below_body_m = 0.035f;
                } else {
                    x = -0.38f*observation->up[0];
                }
                x = clamp(x, -0.17f, 0.17f);
                z = fminf(z, recovery->cfg.wheel_radius_m - observation->leg[i].rest - extension_below_body_m);
            }
            const float target[2] = {recovery->starts[i][0]+b*(x-recovery->starts[i][0]),
                                     recovery->starts[i][1]+b*(z-recovery->starts[i][1])};
            if (!track_wheel_pose(recovery, input, observation, i, target,
                                  position_gain, velocity_gain, dt, &out->direct)) {
                return false;
            }
        }
        ready = upright_cos > upright_threshold && observation->max_angle < 0.65f &&
                observation->speed < 1.2f;
        if (recovery->damp_pitch) {
            const bool loaded = recovery->contact[0] && recovery->contact[1] && fminf(recovery->fn[0], recovery->fn[1]) > 30.0f;
            const bool near_floor = fminf(observation->leg[0].clearance, observation->leg[1].clearance) > -0.008f;
            ready = ready && observation->max_angle < 0.48f && fabsf(observation->pitch_rate) < 0.70f && (loaded || near_floor);
        }
        if (!ready && recovery->t > align_timeout_s) {
            retry_or_block(recovery, SR_NO_PROGRESS, observation);
        }
    } else {
        const float u = clamp(recovery->t/recovery->roll_duration, 0.0f, 1.0f);
        const float b = quintic_blend(u);
        const float rate = 30.0f*u*u*(1.0f-u)*(1.0f-u)/recovery->roll_duration;
        float error = 0.0f;
        for (unsigned i = 0; i < 2; ++i) {
            const RecoveryLeg_t *leg = &observation->leg[i];
            if (observation->speed < 1.2f && upright_cos > 0.55f && leg->wheel_xz[1] > 0.022f &&
                fabsf(leg->theta) > 1.65f && leg->wheel_speed < 0.25f) {
                retry_or_block(recovery, SR_LEFT_ROLL_PATH, observation);
                return true;
            }
            const float dx = recovery->roll_target[i]-recovery->starts[i][0];
            pull_wheel_along_ground(recovery, input, leg, i, recovery->starts[i][0]+b*dx, rate*dx, &out->direct);
            error = fmaxf(error, fabsf(leg->wheel_xz[0]-reachable_ground_x(recovery, leg, recovery->roll_target[i])));
        }
        const bool slow = observation->max_foot_speed < 0.30f;
        const bool short_legs = fmaxf(observation->leg[0].length, observation->leg[1].length) <
                               recovery->cfg.min_length_m+0.014f && observation->max_angle < 0.78f;
        const bool done = short_legs || (error < 0.025f && observation->max_angle < 0.90f);
        ready = upright_cos > 0.90f && observation->speed < 1.2f && slow && done;
        if (!ready && upright_cos <= 0.90f && observation->speed < 1.5f && slow && done) {
            start_retraction(recovery, observation, ALIGN_ON_TILTED_BODY);
        } else if (!ready && recovery->t > recovery->roll_duration &&
                   (progress_stalled(recovery, error, 0.005f, 0.35f, dt) || recovery->t > recovery->roll_duration+1.0f)) {
            if (observation->speed < 1.0f && slow && observation->can_align_on_body) {
                start_retraction(recovery, observation, ALIGN_ON_BODY);
            } else if (upright_cos > 0.68f && observation->speed < 1.2f && slow && observation->max_angle < 1.8f &&
                       fminf(observation->leg[0].wheel_xz[1], observation->leg[1].wheel_xz[1]) < -0.06f) {
                start_retraction(recovery, observation, ALIGN_ON_TILTED_BODY);
            } else {
                retry_or_block(recovery, SR_NO_PROGRESS, observation);
            }
        }
    }

    /* 两种路线共用同一段平衡交接逻辑。 */
    if (ready) {
        recovery->capture_prev = input->applied;
        recovery->offset_valid = false;
        recovery->capture_length[0] = observation->leg[0].length;
        recovery->capture_length[1] = observation->leg[1].length;
        recovery->reason = SR_OK;
        out->handoff = true;
        enter_phase(recovery, SR_RISE);
    }
    return true;
}

static void limit_output_torque(const SelfRighting_t *recovery, SelfRighting_Torque_t *out)
{
    for (unsigned i = 0; i < 4; ++i) {
        float lo, hi;
        hip_torque_bounds(recovery, i, &lo, &hi);
        out->hip[i] = clamp(out->hip[i], lo, hi);
    }
    for (unsigned i = 0; i < 2; ++i) {
        out->wheel[i] = clamp(out->wheel[i], -recovery->cfg.wheel_limit_nm, recovery->cfg.wheel_limit_nm);
    }
}

SelfRighting_Output_t SelfRighting_Update(SelfRighting_t *recovery,
                                         const SelfRighting_Input_t *input, float dt)
{
    SelfRighting_Output_t out = {0};
    if (recovery == NULL || input == NULL) {
        out.phase = SR_FAULT;
        out.reason = SR_BAD_INPUT;
        out.active = true;
        return out;
    }
    if (!input->armed) {
        SelfRighting_Reset(recovery);
        return out;
    }
    RecoveryObservation_t observation = {0};
    if (recovery->phase != SR_FAULT) {
        /* 先检查输入，再更新姿态和腿部观测，故障锁定期间不读取反馈。 */
        const bool feedback_valid = recovery->configured &&
            input->feedback_valid && input->vmc != NULL && input->vmc->derivative_initialized &&
            isfinite(dt) && dt > 0.0f && dt <= 0.02f &&
            finite_array(input->quat_wxyz, 4) && finite_array(input->gyro_radps, 3) &&
            finite_array(input->accel_mps2, 3) && finite_array(input->motor_pos_rad, 4) &&
            finite_array(input->motor_vel_radps, 4) && finite_array(input->wheel_vel_radps, 2) &&
            finite_array(input->applied.hip, 4) && finite_array(input->applied.wheel, 2) &&
            finite_array(input->leg_gravity_nm, 4);
        if (!feedback_valid || !observe_body_and_legs(recovery, input, &observation)) {
            recovery->reason = SR_BAD_INPUT;
            enter_phase(recovery, SR_FAULT);
        }
    }
    if (recovery->phase == SR_FAULT) {
        out.phase = SR_FAULT;
        out.reason = recovery->reason;
        out.active = true;
        return out;
    }
    memcpy(recovery->current_q, observation.q, sizeof(recovery->current_q));
    recovery->cooldown = fmaxf(0.0f, recovery->cooldown-dt);
    const float upright_cos = observation.up[2];  /* 机身 Z 轴与世界竖直方向的夹角余弦。 */
    const bool large_leg_angle = !input->step_protected && observation.max_angle > 0.95f;
    const bool abnormal = upright_cos < 0.55f || large_leg_angle || fabsf(observation.up[0]) > 0.70f;
    if (recovery->phase == SR_WATCH) {
        if (abnormal && !input->jump_protected && recovery->cooldown == 0.0f) {
            recovery->quiet += dt;
        } else {
            recovery->quiet = 0.0f;
        }
        float debounce = 0.030f;
        if (upright_cos < 0.30f || (large_leg_angle && observation.max_angle > 1.2f)) {
            debounce = 0.010f;
        }
        if (recovery->quiet < debounce) {
            out.use_balance = recovery->cooldown > 0.0f;
            return out;
        }
        recovery->retries = 0U;
        recovery->total = 0.0f;
        recovery->reason = SR_OK;
        recovery->support_tried = false;
        recovery->load_valid = false;
        memset(recovery->fn, 0, sizeof(recovery->fn));
        memset(recovery->contact, 0, sizeof(recovery->contact));
        memset(recovery->contact_time, 0, sizeof(recovery->contact_time));
        enter_phase(recovery, SR_SETTLE);
    }
    recovery->t += dt;
    recovery->total += dt;
    /* 由实发髋力矩估计轮载，并以滞回阈值确认接地。 */
    for (unsigned i = 0; i < 2; ++i) {
        const RecoveryLeg_t *leg = &observation.leg[i];
        const float *jx = leg->jacobian_xz[0], *jz = leg->jacobian_xz[1];
        const float axx = jx[0]*jx[0] + jx[1]*jx[1] + 0.000225f;
        const float axz = jx[0]*jz[0] + jx[1]*jz[1];
        const float azz = jz[0]*jz[0] + jz[1]*jz[1] + 0.000225f;
        const VMC_AirSpring_t *spring = &input->vmc->spring[i];
        const float tf = input->applied.hip[i] + spring->torque_front_nm - input->leg_gravity_nm[i];
        const float tb = input->applied.hip[i+2] + spring->torque_back_nm - input->leg_gravity_nm[i+2];
        const float fz = (axx*(jz[0]*tf+jz[1]*tb) - axz*(jx[0]*tf+jx[1]*tb))/(axx*azz-axz*axz);
        float rel_acc = 0.0f;
        if (recovery->load_valid) {
            rel_acc = (leg->wheel_velocity_xz[1] - recovery->last_vz[i]) / dt;
        }
        recovery->last_vz[i] = leg->wheel_velocity_xz[1];
        const float confidence = clamp((leg->clearance+0.05f)/0.03f, 0.0f, 1.0f);
        const float inertia = recovery->cfg.wheel_effective_mass_kg*
                              clamp(dot3(observation.up, input->accel_mps2)+rel_acc, -60.0f, 60.0f);
        const float fn = clamp(-fz + inertia, 0.0f, 350.0f)*confidence;
        recovery->fn[i] += (dt/(dt+0.004667f))*(fn-recovery->fn[i]);
        /* 接地与离地用不同阈值，持续满足条件才翻转接地状态。 */
        bool contact_changed;
        if (recovery->contact[i]) {
            contact_changed = recovery->fn[i] < 8.0f;
        } else {
            contact_changed = recovery->fn[i] > 30.0f;
        }
        if (contact_changed) {
            recovery->contact_time[i] += dt;
        } else {
            recovery->contact_time[i] = 0.0f;
        }
        if (recovery->contact_time[i] >= 0.010f) {
            recovery->contact[i] = !recovery->contact[i];
            recovery->contact_time[i] = 0.0f;
        }
    }
    recovery->load_valid = true;
    /* 全局超时和二次翻倒先于当前阶段处理，本拍不再推进旧轨迹。 */
    bool reachable = true;
    if (recovery->total > recovery->cfg.timeout_s) {
        recovery->reason = SR_TIMEOUT;
        enter_phase(recovery, SR_FAULT);
    } else if ((recovery->phase == SR_REPOSITION || recovery->phase == SR_ROLL_IN || recovery->phase == SR_RISE) &&
               upright_cos < 0.30f) {
        recovery->retries = 0U;
        recovery->support_tried = false;
        recovery->reason = SR_OVERTURNED;
        recovery->angle_valid = false;
        enter_phase(recovery, SR_SETTLE);
    } else {
        switch (recovery->phase) {
        case SR_SETTLE:
            apply_joint_damping(input, &observation, &out.direct);
            if (!(observation.speed < 1.2f)) {
                if (recovery->t > 1.5f) {
                    retry_or_block(recovery, SR_NO_PROGRESS, &observation);
                }
                break;
            }
            if (upright_cos < 0.30f) {
                /* 倒扣时先把轮摆到能向下撑地的一侧，再执行翻身。 */
                save_trajectory_start(recovery, &observation);
                recovery->flip_sign = 1.0f;
                if (observation.up[0] > 0.0f) {
                    recovery->flip_sign = -1.0f;
                }
                if (recovery->retries & 1U) {
                    recovery->flip_sign = -recovery->flip_sign;
                }
                const float angle = atan2f(-recovery->flip_sign*recovery->cfg.swing_length_m, 0.010f);
                float distance = 0.0f;
                float error = 0.0f;
                for (unsigned i = 0; i < 2; ++i) {
                    recovery->start_angle[i] = recovery->theta[i];
                    recovery->goal_angle[i] = VMC_UnwrapAngleNear(angle, recovery->theta[i]);
                    distance = fmaxf(distance, fabsf(recovery->goal_angle[i]-recovery->start_angle[i]));
                    error = fmaxf(error, fabsf(observation.leg[i].length-recovery->cfg.swing_length_m));
                }
                recovery->duration[0] = fmaxf(0.525f, distance/2.4f);
                recovery->grow[0] = 0.20f;
                if (error < 0.005f) {
                    recovery->grow[0] = 0.0f;
                }
                enter_phase(recovery, SR_FLIP_PLACE);
            } else if (!recovery->support_tried && observation.can_align_on_body) {
                start_retraction(recovery, &observation, ALIGN_ON_BODY);
            } else {
                /* 给两条腿分别选路，再生成角度和时长。 */
                save_trajectory_start(recovery, &observation);
                recovery->route_length = fmaxf(recovery->cfg.min_length_m, recovery->cfg.swing_length_m-0.018f*recovery->retries);
                for (unsigned i = 0; i < 2; ++i) {
                    const RecoveryLeg_t *leg = &observation.leg[i];
                    const float z_limit = fminf(0.30f, recovery->route_length);
                    const float z = clamp(recovery->cfg.wheel_radius_m-leg->rest, -z_limit, -0.020f);
                    const float x = -sqrtf(fmaxf(0.0001f, recovery->route_length*recovery->route_length-z*z));
                    const float angle = atan2f(-x, -z);
                    recovery->park[i][0] = x;
                    recovery->park[i][1] = z;
                    const bool direct = leg->wheel_xz[1] < -0.035f && fabsf(leg->theta) < 0.85f && leg->length < 0.27f;
                    const bool rear_floor = observation.up[2] > 0.90f && leg->wheel_xz[0] < -0.10f &&
                        leg->wheel_xz[1] < -0.035f && leg->length < 0.35f && fabsf(leg->clearance) < 0.025f;
                    recovery->start_angle[i] = recovery->theta[i];
                    if (direct || rear_floor) {
                        recovery->route[i] = SR_GROUND_SHORT;
                        recovery->goal_angle[i] = recovery->theta[i];
                    } else if (leg->wheel_xz[0] < -0.10f && leg->wheel_xz[1] < 0.04f) {
                        recovery->route[i] = SR_REAR_LOWER;
                        recovery->goal_angle[i] = VMC_UnwrapAngleNear(angle, recovery->theta[i]);
                    } else {
                        recovery->route[i] = SR_UPPER_ARC;
                        recovery->goal_angle[i] = VMC_SelectAngleTarget(recovery->theta[i], angle, VMC_ANGLE_PATH_NEGATIVE);
                        if (recovery->goal_angle[i] >= recovery->theta[i]-0.05f) {
                            recovery->goal_angle[i] -= 2.0f*SR_PI;
                        }
                    }
                    recovery->duration[i] = fmaxf(0.375f, fabsf(recovery->goal_angle[i]-recovery->start_angle[i])/(4.4f/1.5f));
                    recovery->grow[i] = 0.20f;
                    if (fabsf(leg->length - recovery->route_length) < 0.005f) {
                        recovery->grow[i] = 0.0f;
                    }
                    const float dx = leg->wheel_xz[0] - x;
                    const float dz = leg->wheel_xz[1] - z;
                    recovery->done[i] = recovery->route[i] == SR_GROUND_SHORT ||
                        (recovery->route[i] == SR_REAR_LOWER && sqrtf(dx*dx + dz*dz) < 0.025f && leg->wheel_speed < 0.4f);
                }
                enter_phase(recovery, SR_REPOSITION);
            }
            break;
        case SR_FLIP_PLACE:
        case SR_FLIP_PUSH:
        case SR_FLIP_BRAKE: {
            /* 摆轮、撑地、制动在对应阶段生成力矩与切换条件。 */
            const float up_rate = observation.up[0]*input->gyro_radps[1] - observation.up[1]*input->gyro_radps[0];
            if (recovery->phase != SR_FLIP_BRAKE && upright_cos > 0.45f &&
                (recovery->phase == SR_FLIP_PLACE || observation.speed < 9.0f)) {
                recovery->angle_valid = false;
                enter_phase(recovery, SR_SETTLE);
                break;
            }
            if (recovery->phase == SR_FLIP_PUSH && (upright_cos > 0.45f || (upright_cos > 0.12f && up_rate > 10.0f))) {
                save_trajectory_start(recovery, &observation);
                enter_phase(recovery, SR_FLIP_BRAKE);
                apply_joint_damping(input, &observation, &out.direct);
                break;
            }
            float error = 0.0f;
            for (unsigned i = 0; i < 2; ++i) {
                const RecoveryLeg_t *leg = &observation.leg[i];
                if (recovery->phase == SR_FLIP_PLACE) {
                    float extension_blend = 1.0f;
                    if (recovery->grow[0] > 0.0f) {
                        extension_blend = quintic_blend(recovery->t / recovery->grow[0]);
                    }
                    const float length = recovery->start_length[i] +
                        (recovery->cfg.swing_length_m - recovery->start_length[i])*extension_blend;
                    const float angle = recovery->start_angle[i] + (recovery->goal_angle[i]-recovery->start_angle[i])*
                        quintic_blend((recovery->t-recovery->grow[0])/recovery->duration[0]);
                    const float target[2] = {-length*sinf(angle), -length*cosf(angle)};
                    if (!track_wheel_pose(recovery, input, &observation, i, target, 60.0f, 2.5f, dt, &out.direct)) {
                        reachable = false;
                        break;
                    }
                    const float dx = leg->wheel_xz[0] + recovery->cfg.swing_length_m*sinf(recovery->goal_angle[i]);
                    const float dz = leg->wheel_xz[1] + recovery->cfg.swing_length_m*cosf(recovery->goal_angle[i]);
                    error = fmaxf(error, sqrtf(dx*dx + dz*dz));
                } else if (recovery->phase == SR_FLIP_PUSH) {
                    /* 机身翻离倒扣位置后，根据翻转速度减小撑地力。 */
                    float scale = 1.0f;
                    if (upright_cos > -0.55f) {
                        scale = clamp((15.0f - up_rate)/6.0f, 0.25f, 1.0f);
                    }
                    map_wheel_force_to_hip_torque(input, leg, i, -recovery->flip_sign*35.0f*scale,
                                 -scale*fminf(260.0f, 65.0f+120.0f*recovery->t), &out.direct);
                    out.direct.hip[i] -= 0.5f*observation.dq[i];
                    out.direct.hip[i+2] -= 0.5f*observation.dq[i+2];
                } else {
                    const float start = recovery->start_length[i];
                    const float length = start+(fminf(start, 0.225f)-start)*quintic_blend(recovery->t/0.13f);
                    const float lateral = recovery->cfg.wheel_lateral_m[i];
                    const float dx = observation.forward[1]*lateral, dz = observation.up[1]*lateral;
                    const float target[2] = {dx + (leg->wheel_xz[0]-dx)*length/leg->length,
                                             dz + (leg->wheel_xz[1]-dz)*length/leg->length};
                    if (!track_wheel_pose(recovery, input, &observation, i, target, 70.0f, 3.0f, dt, &out.direct)) {
                        reachable = false;
                        break;
                    }
                }
                out.direct.wheel[i] = clamp(-0.15f*input->wheel_vel_radps[i], -1.5f, 1.5f);
            }
            if (!reachable) {
                break;
            }
            if (recovery->phase == SR_FLIP_PLACE) {
                const float plan_time = recovery->grow[0]+recovery->duration[0];
                if (error < 0.060f && observation.max_foot_speed < 0.5f) {
                    enter_phase(recovery, SR_FLIP_PUSH);
                } else if (recovery->t > plan_time && (progress_stalled(recovery, error, 0.006f, 0.35f, dt) ||
                                                recovery->t > plan_time+1.0f)) {
                    retry_or_block(recovery, SR_NO_PROGRESS, &observation);
                }
            } else if (recovery->phase == SR_FLIP_PUSH) {
                if (recovery->t > 0.65f && (progress_stalled(recovery, 1.0f-upright_cos, 0.02f, 0.5f, dt) || recovery->t > 2.5f))
                    retry_or_block(recovery, SR_NO_PROGRESS, &observation);
            } else if ((upright_cos > 0.80f && observation.speed < 2.6f) || (upright_cos > 0.55f && observation.speed < 1.6f) ||
                       (recovery->t > 0.65f && upright_cos > 0.45f)) {
                recovery->angle_valid = false;
                enter_phase(recovery, SR_SETTLE);
            } else if (recovery->t > 0.65f) {
                retry_or_block(recovery, SR_NO_PROGRESS, &observation);
            }
            break;
        }
        case SR_REPOSITION: {
            /* 两腿独立跟踪绕行路线，两侧到位后开始沿地收轮。 */
            float error = 0.0f, plan_time = 0.0f;
            for (unsigned i = 0; i < 2; ++i) {
                const RecoveryLeg_t *leg = &observation.leg[i];
                if (recovery->done[i]) {
                    float x = recovery->park[i][0];
                    if (recovery->route[i] == SR_GROUND_SHORT) {
                        x = recovery->starts[i][0];
                    }
                    pull_wheel_along_ground(recovery, input, leg, i, x, 0.0f, &out.direct);
                    continue;
                }
                const float u = quintic_blend((recovery->t-recovery->grow[i])/recovery->duration[i]);
                float grow = 1.0f;
                if (recovery->grow[i] > 0.0f) {
                    grow = quintic_blend(recovery->t / recovery->grow[i]);
                }
                const float length = recovery->start_length[i]+(recovery->route_length-recovery->start_length[i])*grow;
                const float angle = recovery->start_angle[i]+(recovery->goal_angle[i]-recovery->start_angle[i])*u;
                const float target[2] = {-length*sinf(angle), -length*cosf(angle)};
                if (!track_wheel_pose(recovery, input, &observation, i, target, 90.0f, 3.0f, dt, &out.direct)) {
                    reachable = false;
                    break;
                }
                const float dx = leg->wheel_xz[0] - recovery->park[i][0];
                const float dz = leg->wheel_xz[1] - recovery->park[i][1];
                const float position_error = sqrtf(dx*dx + dz*dz);
                const float angle_error = fabsf(recovery->theta[i]-recovery->goal_angle[i]);
                const bool parked = angle_error < 0.08f && position_error < 0.030f;
                const bool can_roll = u > 0.75f && angle_error < 0.24f && leg->wheel_xz[0] < -0.10f &&
                    leg->wheel_xz[1] < -0.035f && (leg->wheel_xz[1] < -0.10f || leg->clearance > -0.035f);
                if (leg->wheel_speed < 0.45f && (parked || can_roll)) {
                    recovery->done[i] = true;
                    if (!parked) {
                        memcpy(recovery->park[i], leg->wheel_xz, sizeof(recovery->park[i]));
                    }
                } else {
                    error = fmaxf(error, position_error);
                    plan_time = fmaxf(plan_time, recovery->grow[i]+recovery->duration[i]);
                }
            }
            if (!reachable) {
                break;
            }
            if (recovery->done[0] && recovery->done[1]) {
                start_retraction(recovery, &observation, RETRACT_ON_GROUND);
            } else if (recovery->t > plan_time && (progress_stalled(recovery, error, 0.005f, 0.35f, dt) ||
                                            recovery->t > plan_time+1.0f)) {
                retry_or_block(recovery, SR_NO_PROGRESS, &observation);
            }
            break;
        }
        case SR_ROLL_IN:
            reachable = update_retraction(recovery, input, &observation, dt, &out);
            break;
        case SR_RISE: {
            const float min_clearance = fminf(observation.leg[0].clearance, observation.leg[1].clearance);
            const float max_clearance = fmaxf(observation.leg[0].clearance, observation.leg[1].clearance);

            /* 两轮高差较大时，仅靠轮底几何高度无法确认支撑，改用双轮载荷。 */
            const bool wheels_loaded = recovery->contact[0] && recovery->contact[1] &&
                                       fminf(recovery->fn[0], recovery->fn[1]) > 40.0f;
            const bool stable_on_split_level = wheels_loaded &&
                upright_cos > 0.98f && observation.max_angle < 0.28f && observation.speed < 0.65f &&
                min_clearance > -0.015f && max_clearance - min_clearance > 0.045f;
            const bool stable_on_level = upright_cos > 0.94f && fabsf(observation.up[0]) < 0.14f &&
                observation.max_angle < 0.40f && observation.speed < 1.0f && min_clearance > 0.012f;
            if (recovery->t >= SR_LENGTH_BLEND_S && (stable_on_split_level || stable_on_level)) {
                recovery->cooldown = SR_COOLDOWN_S;
                recovery->quiet = 0.0f;
                enter_phase(recovery, SR_WATCH);
                out.released = true;
                break;
            }

            /* 先给平衡环建立支撑的时间，再判定姿态异常或卡滞。 */
            if (recovery->t > SR_LENGTH_BLEND_S + 0.10f) {
                bool should_retry = abnormal;
                if (!should_retry) {
                    const float rise_error = fmaxf(0.0f, 0.012f - min_clearance) +
                                             0.12f*observation.max_angle + 0.10f*(1.0f - upright_cos);
                    /* 卡滞检测会更新计时，即使本拍也满足总时限，仍先执行它。 */
                    should_retry = progress_stalled(recovery, rise_error, 0.003f, 0.65f, dt) ||
                                   recovery->t > 2.0f;
                }
                if (should_retry) {
                    retry_or_block(recovery, SR_NO_PROGRESS, &observation);
                }
            }
            break;
        }
        case SR_BLOCKED: {
            const float diff[3] = {observation.up[0]-recovery->blocked_up[0], observation.up[1]-recovery->blocked_up[1],
                                   observation.up[2]-recovery->blocked_up[2]};
            /* 重试耗尽后保持阻尼，等机身或关节位置明显改变再尝试。 */
            bool moved = dot3(diff, diff) > 0.18f*0.18f;
            for (unsigned i = 0; i < 4 && !moved; ++i) {
                const float angle_change = VMC_WrapAngle(observation.q[i] - recovery->blocked_q[i]);
                moved = fabsf(angle_change) > 0.18f;
            }
            apply_joint_damping(input, &observation, &out.direct);
            if ((upright_cos < 0.30f && recovery->blocked_up[2] >= 0.30f) ||
                (moved && observation.speed < 1.2f && recovery->t > 0.20f)) {
                recovery->retries = 0U;
                recovery->support_tried = false;
                recovery->reason = SR_OK;
                recovery->angle_valid = false;
                enter_phase(recovery, SR_SETTLE);
            } else if (!recovery->support_tried && observation.speed < 1.0f && observation.max_foot_speed < 0.15f && observation.can_align_on_body) {
                start_retraction(recovery, &observation, ALIGN_ON_BODY);
            }
            break;
        }
        default:
            break;
        }
    }
    if (!reachable) {
        retry_or_block(recovery, SR_UNREACHABLE, &observation);
    }
    if (recovery->phase == SR_SETTLE || recovery->phase == SR_BLOCKED) {
        apply_joint_damping(input, &observation, &out.direct);
    } else if (recovery->phase == SR_FAULT || recovery->phase == SR_RISE || out.released) {
        out.direct = (SelfRighting_Torque_t){0};
    }
    if (!finite_array(out.direct.hip, 4) || !finite_array(out.direct.wheel, 2)) {
        enter_phase(recovery, SR_FAULT);
        recovery->reason = SR_BAD_INPUT;
        out = (SelfRighting_Output_t){0};
    }
    limit_output_torque(recovery, &out.direct);
    out.phase = recovery->phase;
    out.reason = recovery->reason;
    out.active = recovery->phase != SR_WATCH;
    out.use_balance = recovery->phase == SR_RISE || out.released;
    if (out.use_balance) {
        float b = 1.0f;
        if (!out.released) {
            b = quintic_blend(recovery->t / SR_LENGTH_BLEND_S);
        }
        for (unsigned i = 0; i < 2; ++i)
            out.length_ref_m[i] = recovery->capture_length[i] + b*(recovery->cfg.capture_length_m-recovery->capture_length[i]);
    }
    for (unsigned side = 0; side < 2; ++side) {
        if (recovery->contact[side]) {
            out.normal_force_n[side] = recovery->fn[side];
        } else {
            out.normal_force_n[side] = 0.0f;
        }
    }
    return out;
}

SelfRighting_Torque_t SelfRighting_MixBalance(SelfRighting_t *recovery,
                                             const SelfRighting_Torque_t *balance,
                                             const SelfRighting_Torque_t *applied, float dt)
{
    SelfRighting_Torque_t out = {0};
    if (recovery == NULL || balance == NULL || applied == NULL || !recovery->configured ||
        recovery->phase == SR_FAULT || !isfinite(dt) || dt <= 0.0f || dt > 0.02f ||
        !finite_array(balance->hip, 4) || !finite_array(balance->wheel, 2) ||
        !finite_array(applied->hip, 4) || !finite_array(applied->wheel, 2)) {
        return out;
    }
    out = *balance;
    if (recovery->phase == SR_RISE) {
        for (unsigned i = 0; i < 2; ++i) {
            if (!recovery->contact[i]) {
                out.wheel[i] = 0.0f;
            }
        }
        if (!recovery->offset_valid) {
            for (unsigned i = 0; i < 4; ++i) {
                recovery->offset.hip[i] = recovery->capture_prev.hip[i]-out.hip[i];
            }
            for (unsigned i = 0; i < 2; ++i) {
                recovery->offset.wheel[i] = recovery->capture_prev.wheel[i]-out.wheel[i];
            }
            recovery->offset_valid = true;
        }
        const float blend = 1.0f-quintic_blend(recovery->t/SR_BLEND_S);
        for (unsigned i = 0; i < 4; ++i) {
            out.hip[i] += blend*recovery->offset.hip[i];
        }
        for (unsigned i = 0; i < 2; ++i) {
            out.wheel[i] += blend*recovery->offset.wheel[i];
        }
    }
    /* 交接和冷却期间限制每周期力矩变化，保持与实际下发值连续。 */
    if (recovery->phase != SR_RISE && !(recovery->cooldown > 0.0f)) {
        return out;
    }

    limit_output_torque(recovery, &out);
    const float max_hip_delta_nm = 1000.0f * dt;
    const float max_wheel_delta_nm = 250.0f * dt;
    for (unsigned i = 0; i < 4; ++i) {
        const float delta_nm = out.hip[i] - applied->hip[i];
        out.hip[i] = applied->hip[i] + clamp(delta_nm, -max_hip_delta_nm, max_hip_delta_nm);
    }
    for (unsigned i = 0; i < 2; ++i) {
        const float delta_nm = out.wheel[i] - applied->wheel[i];
        out.wheel[i] = applied->wheel[i] + clamp(delta_nm, -max_wheel_delta_nm, max_wheel_delta_nm);
    }
    limit_output_torque(recovery, &out);
    return out;
}
