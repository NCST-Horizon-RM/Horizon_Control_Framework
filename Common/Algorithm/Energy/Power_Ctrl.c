#include "Power_Ctrl.h"
#include <math.h>

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

// 预测单电机功率
static inline float predict_motor_power(const Power_Node_t *node, float I_cmd) {
    float w = node->state->speed_rpm * POWER_RPM_TO_RAD;
    float I = I_cmd * node->model->current_convert;
    return (node->model->k1 * w * I) + (node->model->k2 * I * I) +
           (node->model->k3 * w * w) + node->model->k4;
}

// 解算单个 Group 的削减比例
static void solve_motor_group(Power_Group_t *group, float P_limit) {
    float A = 0.0f, B = 0.0f, C_base = 0.0f;

    for (uint8_t i = 0; i < group->node_count; i++) {
        Power_Node_t *n = &group->nodes[i];
        float w = n->state->speed_rpm * POWER_RPM_TO_RAD;
        float I = n->state->original_cmd * n->model->current_convert;

        A += n->model->k2 * I * I;
        B += n->model->k1 * w * I;
        C_base += n->model->k3 * w * w + n->model->k4;
    }

    if (A + B + C_base <= P_limit) {
        for(uint8_t i=0; i < group->node_count; i++) {
            group->nodes[i].state->limited_cmd = group->nodes[i].state->original_cmd;
        }
        return;
    }
    float C = C_base - P_limit;
    float scale = 1.0f;

    if (A > 1e-6f) {
        float delta = B * B - 4.0f * A * C;
        if (delta >= 0) scale = (-B + sqrtf(delta)) / (2.0f * A);
        else scale = 0.0f;
    } else if (B > 1e-6f) {
        scale = -C / B;
    } else {
        scale = 0.0f;
    }
    scale = (scale > 1.0f) ? 1.0f : ((scale < 0.0f) ? 0.0f : scale);
    for (uint8_t i = 0; i < group->node_count; i++) {
        group->nodes[i].state->limited_cmd = group->nodes[i].state->original_cmd * scale;
    }
}

/**
 * @brief 功率控制算法
 * @param ctrl          算法实例
 * @param allowed_limit 当前裁判系统允许的最大绝对功率 (W)
 * @param cur_buffer    当前的裁判系统剩余缓冲能量 (J)
 * @param groups        功率控制组数组。排列规则：下标越小，优先级越低
 * 例如：groups[0] 放驱动轮(最先被砍)，groups[1] 放舵轮(最后被砍)
 * @param group_count   总共有多少个组
 */
void Power_Ctrl_Calculate(Power_Ctrl_t *ctrl,
                          float allowed_limit,
                          Power_Group_t *groups,
                          uint8_t group_count)
{
    float actual_limit = allowed_limit;

    // 计算每个组的预测总功率需求
    float group_pred_power[group_count];
    float total_pred_power = 0.0f;

    for (uint8_t g = 0; g < group_count; g++) {
        group_pred_power[g] = 0.0f;
        for (uint8_t i = 0; i < groups[g].node_count; i++) {
            float p = predict_motor_power(&groups[g].nodes[i], groups[g].nodes[i].state->original_cmd);
            if (p < 0.0f) p = 0.0f;
            group_pred_power[g] += p;
        }
        total_pred_power += group_pred_power[g];
    }
    // 优先级阶梯分配算法
    if (total_pred_power > actual_limit) {
        float remaining_limit = actual_limit;
        // 倒序遍历（下标越大的组，优先级越高，越先拿到功率配额）
        for (int8_t g = group_count - 1; g >= 0; g--) {
            if (remaining_limit >= group_pred_power[g]) {
                // 配额充足，全额输出
                for(uint8_t i=0; i < groups[g].node_count; i++) {
                    groups[g].nodes[i].state->limited_cmd = groups[g].nodes[i].state->original_cmd;
                }
                remaining_limit -= group_pred_power[g];
            } else {
                // 配额不足，削减该组
                if (remaining_limit > 0.0f) {
                    solve_motor_group(&groups[g], remaining_limit);
                    remaining_limit = 0.0f; // 配额用光
                } else {
                    // 配额已经枯竭，更低优先级的组直接输出 0
                    for(uint8_t i=0; i < groups[g].node_count; i++) {
                        groups[g].nodes[i].state->limited_cmd = 0.0f;
                    }
                }
            }
        }
    } else {
        // 总功率未超，全部满输出
        for (uint8_t g = 0; g < group_count; g++) {
            for(uint8_t i=0; i < groups[g].node_count; i++) {
                groups[g].nodes[i].state->limited_cmd = groups[g].nodes[i].state->original_cmd;
            }
        }
    }
    // 重新计算并记录限制后的最终功率消耗预测
    ctrl->total_pred_power = 0.0f;
    for (uint8_t g = 0; g < group_count; g++) {
        for (uint8_t i = 0; i < groups[g].node_count; i++) {
            ctrl->total_pred_power += predict_motor_power(&groups[g].nodes[i], groups[g].nodes[i].state->limited_cmd);
        }
    }
}