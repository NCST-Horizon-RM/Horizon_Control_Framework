#ifndef HORIZON_POWER_CTRL_H
#define HORIZON_POWER_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#define POWER_RPM_TO_RAD (2.0f * 3.14159265f / 60.0f)

// 电机物理模型参数
typedef struct {
    float k1, k2, k3, k4;
    float current_convert; // PID 输出到实际转矩电流的转换系数
} Power_Motor_Model_t;

extern const Power_Motor_Model_t MODEL_M3508;
extern const Power_Motor_Model_t MODEL_M6020;

// 单电机状态
typedef struct {
    float speed_rpm;       // 当前实时转速 (RPM)
    float original_cmd;    // 原始输入的 PID 电流控制项
    float limited_cmd;     // 缩放限制后的最终输出电流
} Motor_Power_State_t;

// 功率计算
typedef struct {
    Motor_Power_State_t *state;       // 指向对应的电机状态变量
    const Power_Motor_Model_t *model; // 指向该电机的物理模型
} Power_Node_t;

// 功率限制组：同组内的电机按照相同的比例一起被限制
typedef struct {
    Power_Node_t *nodes; // 该组包含的电机节点数组
    uint8_t node_count;  // 该组内的电机数量
} Power_Group_t;

// 控制器主体
typedef struct {
    float total_pred_power;   // 解算后预测的总功率
} Power_Ctrl_t;

void Power_Ctrl_Init(Power_Ctrl_t *ctrl);

/**
 * @brief 功率控制算法
 * @param ctrl          算法实例
 * @param allowed_limit 允许的最大功率 (W)
 * @param groups        功率控制组数组。排列规则：下标越小，优先级越低！
 * 例如：groups[0] 放驱动轮(最先被砍)，groups[1] 放舵轮(最后被砍)
 * @param group_count   总共有多少个组
 */
void Power_Ctrl_Calculate(Power_Ctrl_t *ctrl,
                          float allowed_limit,
                          Power_Group_t *groups,
                          uint8_t group_count);

#endif // HORIZON_POWER_CTRL_H