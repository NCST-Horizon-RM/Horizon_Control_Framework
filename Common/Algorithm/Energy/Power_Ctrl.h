#ifndef HORIZON_POWER_CTRL_H
#define HORIZON_POWER_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#define POWER_RPM_TO_RAD (2.0f * 3.14159265f / 60.0f)

/** @brief 电机输入功率模型，电流使用 A，角速度使用 rad/s，功率使用 W。 */
typedef struct {
    float k1;              /**< 转速与电流乘积项系数，描述机械做功。 */
    float k2;              /**< 电流平方项系数，描述铜耗，必须非负。 */
    float k3;              /**< 转速平方项系数，描述速度相关损耗。 */
    float k4;              /**< 单电机固定损耗，单位 W。 */
    float current_convert; /**< 原始电流指令 raw 转换为 A 的系数。 */
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

// 控制器主体
typedef struct {
    float total_pred_power;   // 解算后预测的总功率
} Power_Ctrl_t;

/** @brief 同一真实电机的两个运动电流分量，均在本周期实际转速下评估。 */
typedef struct {
    Power_Node_t motor;     /**< 电机状态与模型，节点间不得共用状态对象。 */
    float translation_cmd; /**< 平移产生的有符号电流指令，单位 raw。 */
    float rotation_cmd;    /**< 旋转产生的有符号电流指令，单位 raw。 */
    float max_cmd;         /**< 合成电流绝对值上限，单位 raw，必须非负。 */
} Power_Motion_Node_t;

/** @brief 同一底盘的运动分量优先级，不是电机组之间的优先级。 */
typedef enum {
    POWER_PRIORITY_TRANSLATION = 0, /**< 优先保留平移，适合小陀螺移动。 */
    POWER_PRIORITY_ROTATION         /**< 优先保留旋转，适合底盘跟随。 */
} Power_Motion_Priority_t;

typedef enum {
    POWER_MOTION_OK = 0,                 /**< 原始请求可全部执行。 */
    POWER_MOTION_LIMITED,                /**< 已找到满足约束的缩放组合。 */
    POWER_MOTION_NO_FEASIBLE_CANDIDATE,   /**< 分阶段搜索未找到可行组合，输出零电流。 */
    POWER_MOTION_INVALID_INPUT           /**< 参数无效，可访问的输出电流清零。 */
} Power_Motion_Status_t;

/** @brief 分配结果；比例为 0～1，两个功率字段的单位均为 W。 */
typedef struct {
    float translation_scale;     /**< 所有轮共用的平移保留比例。 */
    float rotation_scale;        /**< 所有轮共用的旋转保留比例。 */
    float requested_power;       /**< 两种分量均保留 100% 时的预测功率。 */
    float allocated_power;       /**< 最终输出电流对应的预测功率。 */
    Power_Motion_Status_t status; /**< 分配状态，调用方须检查后再发送电流。 */
} Power_Motion_Result_t;


Power_Motion_Status_t Power_Ctrl_Allocate_Motion(Power_Ctrl_t *ctrl,
                                                float allowed_limit,
                                                Power_Motion_Node_t *nodes,
                                                uint8_t node_count,
                                                Power_Motion_Result_t *result);


Power_Motion_Status_t Power_Ctrl_Allocate_Motion_With_Priority(
    Power_Ctrl_t *ctrl, float allowed_limit, Power_Motion_Node_t *nodes,
    uint8_t node_count, Power_Motion_Priority_t priority, Power_Motion_Result_t *result);

/** @brief 初始化功率统计；ctrl 必须指向有效实例。 */
void Power_Ctrl_Init(Power_Ctrl_t *ctrl);

#endif // HORIZON_POWER_CTRL_H
