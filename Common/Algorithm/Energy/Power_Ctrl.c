#include "Power_Ctrl.h"
#include <math.h>
#include <stddef.h>

// M3508物理模型
const Power_Motor_Model_t MODEL_M3508 = {
    .k1 = 1.5756e-02f, .k2 = 1.94e-01f,
    .k3 = 1.9202e-05f, .k4 = 1.15f,
    .current_convert = 20.0f / 16384.0f
};
// M6020物理模型
const Power_Motor_Model_t MODEL_M6020 = {
    .k1 = 0.751f,      .k2 = 2.5f,
    .k3 = 2.1e-5f,     .k4 = 1.15f,
    .current_convert = 3.0f / 16384.0f
};
// 如果有其他电机模型就在下面加，记得头文件要extern

void Power_Ctrl_Init(Power_Ctrl_t *ctrl) {
    ctrl->total_pred_power = 0.0f;
}

/**
 * @brief 用当前实际转速和候选电流预测单电机输入功率。
 * @param[in] node 电机状态与物理模型。
 * @param[in] current_cmd 候选电流指令，单位 raw。
 * @return 输入功率，单位 W；制动时模型结果可能为负。
 */
static inline float predict_motor_power(const Power_Node_t *node, float current_cmd) {
    float speed_rad = node->state->speed_rpm * POWER_RPM_TO_RAD;
    float current_amp = current_cmd * node->model->current_convert;
    return node->model->k1 * speed_rad * current_amp +
           node->model->k2 * current_amp * current_amp +
           node->model->k3 * speed_rad * speed_rad + node->model->k4;
}


/**
 * @brief 先合成各轮电流，再计算总功率，保留两种运动分量的交叉影响。
 * @param[in] nodes 电机分量数组。
 * @param[in] count 电机数量。
 * @param[in] translation_scale 平移保留比例，范围 0～1。
 * @param[in] rotation_scale 旋转保留比例，范围 0～1。
 * @return 各轮非负预测功率之和，单位 W；数值溢出时返回无穷大。
 */
static float motion_power(const Power_Motion_Node_t *nodes, uint8_t count,
                          float translation_scale, float rotation_scale)
{
    float power = 0.0f;
    for (uint8_t index = 0; index < count; index++) {
        float command = translation_scale * nodes[index].translation_cmd +
                        rotation_scale * nodes[index].rotation_cmd;
        float motor_power = predict_motor_power(&nodes[index].motor, command);
        if (!isfinite(motor_power)) return INFINITY;
        /* 不假设制动能量一定能被母线回收，因此负功率不抵扣其他轮耗电。 */
        power += fmaxf(motor_power, 0.0f);
    }
    return power;
}

/** @brief 检查合成电流是否有限且未超过各轮 raw 上限，不单独截断轮电流。 */
static bool motion_current_valid(const Power_Motion_Node_t *nodes, uint8_t count,
                                 float translation_scale, float rotation_scale)
{
    for (uint8_t index = 0; index < count; index++) {
        float command = translation_scale * nodes[index].translation_cmd +
                        rotation_scale * nodes[index].rotation_cmd;
        if (!isfinite(command) || fabsf(command) > nodes[index].max_cmd) return false;
    }
    return true;
}

/**
 * @brief 固定一种运动比例，仅改变另一种比例，计算该候选点功率。
 * @param[in] nodes 电机分量数组。
 * @param[in] count 电机数量。
 * @param[in] vary_rotation true 表示搜索旋转比例，false 表示搜索平移比例。
 * @param[in] fixed_scale 不参与搜索的另一种运动比例。
 * @param[in] scale 当前参与搜索的候选比例。
 * @return 候选点的总预测功率，单位 W。
 */
static float motion_segment_power(const Power_Motion_Node_t *nodes, uint8_t count,
                                  bool vary_rotation, float fixed_scale, float scale)
{
    return motion_power(nodes, count, vary_rotation ? fixed_scale : scale,
                        vary_rotation ? scale : fixed_scale);
}

/**
 * @brief 在一条固定运动比例的搜索线上，寻找尽可能大的可执行比例。
 * @param[in] nodes 电机分量数组，模型参数须已通过校验。
 * @param[in] count 电机数量。
 * @param[in] allowed_limit 总功率预算，单位 W。
 * @param[in] vary_rotation true 搜索旋转，false 搜索平移。
 * @param[in] fixed_scale 另一种运动的固定保留比例。
 * @param[out] scale 成功时写入可执行比例；失败时保持原值。
 * @return true 表示候选比例同时满足总功率和每轮电流上限。
 * @note 功率沿搜索线为凸函数，但制动时不一定单调，不能直接从零二分。
 */
static bool motion_segment_solve(const Power_Motion_Node_t *nodes, uint8_t count,
                                 float allowed_limit, bool vary_rotation,
                                 float fixed_scale, float *scale)
{
    float lower = 0.0f;
    float upper = 1.0f;
    /* 步骤一：电流 = base + slope × 比例；将各轮电流约束取交集。 */
    for (uint8_t index = 0; index < count; index++) {
        float base = fixed_scale * (vary_rotation ? nodes[index].translation_cmd : nodes[index].rotation_cmd);
        float slope = vary_rotation ? nodes[index].rotation_cmd : nodes[index].translation_cmd;
        if (slope == 0.0f) {
            if (fabsf(base) > nodes[index].max_cmd) return false;
            continue;
        }
        float bound_first = (-nodes[index].max_cmd - base) / slope;
        float bound_second = (nodes[index].max_cmd - base) / slope;
        lower = fmaxf(lower, fminf(bound_first, bound_second));
        upper = fminf(upper, fmaxf(bound_first, bound_second));
    }
    if (lower > upper) return false;

    float feasible = upper;
    /* 步骤二：先尝试区间上界；若超功率，寻找一个已知可行的起点。 */
    if (motion_segment_power(nodes, count, vary_rotation, fixed_scale, upper) > allowed_limit) {
        if (motion_segment_power(nodes, count, vary_rotation, fixed_scale, lower) <= allowed_limit) {
            feasible = lower;
        } else {
            /* 两端都超功率时，内部仍可能因制动而可行，用三分搜索功率最小点。 */
            float search_lower = lower;
            float search_upper = upper;
            for (uint8_t iteration = 0; iteration < 32; iteration++) {
                float first = search_lower + (search_upper - search_lower) / 3.0f;
                float second = search_upper - (search_upper - search_lower) / 3.0f;
                float first_power = motion_segment_power(nodes, count, vary_rotation, fixed_scale, first);
                float second_power = motion_segment_power(nodes, count, vary_rotation, fixed_scale, second);
                if (first_power <= second_power) search_upper = second;
                else search_lower = first;
            }
            feasible = (search_lower + search_upper) * 0.5f;
            if (motion_segment_power(nodes, count, vary_rotation, fixed_scale, feasible) > allowed_limit) {
                return false;
            }
        }
        /* 步骤三：从已知可行点向右二分，只保留满足预算的候选值。 */
        for (uint8_t iteration = 0; iteration < 24; iteration++) {
            float candidate = (feasible + upper) * 0.5f;
            if (motion_segment_power(nodes, count, vary_rotation, fixed_scale, candidate) <= allowed_limit) {
                feasible = candidate;
            } else {
                upper = candidate;
            }
        }
    }

    /* 步骤四：重新检查硬约束，消除浮点舍入使边界电流略微越限的情况。 */
    for (uint8_t attempt = 0; attempt < 8; attempt++) {
        float translation = vary_rotation ? fixed_scale : feasible;
        float rotation = vary_rotation ? feasible : fixed_scale;
        if (motion_current_valid(nodes, count, translation, rotation) &&
            motion_power(nodes, count, translation, rotation) <= allowed_limit) {
            *scale = feasible;
            return true;
        }
        feasible = nextafterf(feasible, lower);
    }
    return false;
}

/**
 * @brief 使用默认平移优先策略分配功率。
 * @param[in,out] ctrl 功率控制实例，记录最终预测功率。
 * @param[in] allowed_limit 本周期总功率预算，单位 W，必须非负。
 * @param[in,out] nodes 电机分量数组，结果写入 motor.state->limited_cmd。
 * @param[in] node_count 电机数量，必须大于零。
 * @param[out] result 比例、功率和状态，不能为空。
 * @return 分配状态，与 result->status 一致。
 */
Power_Motion_Status_t Power_Ctrl_Allocate_Motion(Power_Ctrl_t *ctrl,
                                                float allowed_limit,
                                                Power_Motion_Node_t *nodes,
                                                uint8_t node_count,
                                                Power_Motion_Result_t *result)
{
    return Power_Ctrl_Allocate_Motion_With_Priority(ctrl, allowed_limit, nodes, node_count,
                                                   POWER_PRIORITY_TRANSLATION, result);
}

/**
 * @brief 按指定运动优先级，在总功率和各轮电流约束下分配输出。
 * @param[in,out] ctrl 功率控制实例。
 * @param[in] allowed_limit 本周期总功率预算，单位 W。
 * @param[in,out] nodes 电机数组，每轮提供实测 RPM 和两个 raw 电流分量。
 * @param[in] node_count 电机数量，必须大于零。
 * @param[in] priority 优先保留平移或旋转；两者仍受硬件上限约束。
 * @param[out] result 输出保留比例、预测功率和状态，不能为空。
 * @return 分配状态；仅 OK 和 LIMITED 表示找到了满足约束的输出。
 * @note 使用有界分阶段搜索，不保证二维全局最优。未找到可行组合时输出
 *       零电流，但当前转速对应的损耗仍可能超过预算；不得将此状态视为限功成功。
 * @note 不使用负功率抵扣其他轮耗电，不自动修改上层速度目标。
 */
Power_Motion_Status_t Power_Ctrl_Allocate_Motion_With_Priority(
    Power_Ctrl_t *ctrl, float allowed_limit, Power_Motion_Node_t *nodes,
    uint8_t node_count, Power_Motion_Priority_t priority, Power_Motion_Result_t *result)
{
    /* 先清空可访问的电流输出，避免参数错误时继续使用上周期结果。 */
    if (ctrl != NULL) ctrl->total_pred_power = 0.0f;
    if (nodes != NULL) {
        for (uint8_t index = 0; index < node_count; index++) {
            if (nodes[index].motor.state != NULL) nodes[index].motor.state->limited_cmd = 0.0f;
        }
    }
    if (result == NULL) return POWER_MOTION_INVALID_INPUT;
    *result = (Power_Motion_Result_t){.status = POWER_MOTION_INVALID_INPUT};
    if (ctrl == NULL || nodes == NULL || node_count == 0 ||
        (priority != POWER_PRIORITY_TRANSLATION && priority != POWER_PRIORITY_ROTATION) ||
        !isfinite(allowed_limit) || allowed_limit < 0.0f) return result->status;

    /* 模型损耗系数非负是凸搜索的前提；同一个状态对象不能重复参与分配。 */
    for (uint8_t index = 0; index < node_count; index++) {
        const Power_Motion_Node_t *node = &nodes[index];
        const Power_Motor_Model_t *model = node->motor.model;
        if (node->motor.state == NULL || model == NULL ||
            !isfinite(node->motor.state->speed_rpm) ||
            !isfinite(node->translation_cmd) || !isfinite(node->rotation_cmd) ||
            !isfinite(node->translation_cmd + node->rotation_cmd) ||
            !isfinite(node->max_cmd) || node->max_cmd < 0.0f ||
            !isfinite(model->k1) || model->k1 < 0.0f ||
            !isfinite(model->k2) || model->k2 < 0.0f ||
            !isfinite(model->k3) || model->k3 < 0.0f ||
            !isfinite(model->k4) || model->k4 < 0.0f ||
            !isfinite(model->current_convert) || model->current_convert <= 0.0f) return result->status;
        for (uint8_t previous = 0; previous < index; previous++) {
            if (nodes[previous].motor.state == node->motor.state) return result->status;
        }
    }

    /* 检查四个边界组合，提前拦截超出浮点表达范围的输入。 */
    result->requested_power = motion_power(nodes, node_count, 1.0f, 1.0f);
    if (!isfinite(result->requested_power) ||
        !isfinite(motion_power(nodes, node_count, 1.0f, 0.0f)) ||
        !isfinite(motion_power(nodes, node_count, 0.0f, 1.0f)) ||
        !isfinite(motion_power(nodes, node_count, 0.0f, 0.0f))) return result->status;

    float translation = 1.0f;
    float rotation = 1.0f;
    bool translation_first = priority == POWER_PRIORITY_TRANSLATION;
    /* 用指针绑定优先/次优先比例，平移优先和旋转优先共用同一套搜索流程。 */
    float *primary_scale = translation_first ? &translation : &rotation;
    float *secondary_scale = translation_first ? &rotation : &translation;
    if (result->requested_power <= allowed_limit &&
        motion_current_valid(nodes, node_count, translation, rotation)) {
        /* 完整请求可执行，无需压缩任何运动分量。 */
        result->status = POWER_MOTION_OK;
    } else if (motion_segment_solve(nodes, node_count, allowed_limit, translation_first, 1.0f, secondary_scale)) {
        /* 优先分量保留 100%，只削减次优先分量。 */
        result->status = POWER_MOTION_LIMITED;
    } else if (motion_segment_solve(nodes, node_count, allowed_limit, !translation_first, 0.0f, primary_scale)) {
        /* 优先分量也无法全额执行：先确定其可行比例，再尝试补回次优先分量。 */
        *secondary_scale = 0.0f;
        motion_segment_solve(nodes, node_count, allowed_limit, translation_first, *primary_scale, secondary_scale);
        result->status = POWER_MOTION_LIMITED;
    } else {
        /* 搜索未找到可行组合；零电流仍可能存在超预算的速度相关损耗。 */
        translation = 0.0f;
        rotation = 0.0f;
        result->status = POWER_MOTION_NO_FEASIBLE_CANDIDATE;
    }

    /* 两种比例作用于整套电流分量，不对单个轮子再次独立截断。 */
    result->translation_scale = translation;
    result->rotation_scale = rotation;
    result->allocated_power = motion_power(nodes, node_count, translation, rotation);
    ctrl->total_pred_power = result->allocated_power;
    for (uint8_t index = 0; index < node_count; index++) {
        nodes[index].motor.state->original_cmd = nodes[index].translation_cmd + nodes[index].rotation_cmd;
        nodes[index].motor.state->limited_cmd = translation * nodes[index].translation_cmd +
                                               rotation * nodes[index].rotation_cmd;
    }
    return result->status;
}
