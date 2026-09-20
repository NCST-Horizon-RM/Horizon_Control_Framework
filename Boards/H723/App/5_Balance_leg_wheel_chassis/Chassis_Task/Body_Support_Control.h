#ifndef BALANCE_LEG_WHEEL_BODY_SUPPORT_CONTROL_H
#define BALANCE_LEG_WHEEL_BODY_SUPPORT_CONTROL_H

#include "IMU_Task.h"
#include "VMC_Control.h"

/**
 * @brief 机身高度与 ROLL 支撑控制状态。
 *
 * 该结构体同时保存控制器内部状态和对外输出。所有长度使用 m，
 * 速度使用 m/s，力使用 N，力矩使用 N·m，角度使用 rad。
 */
typedef struct {
    float body_height_m;                  /**< 左右虚拟腿竖直高度的平均值。 */
    float body_height_rate_mps;           /**< 机身平均高度变化率。 */
    float limited_height_target_m;        /**< 经过机构最大高度约束后的机身高度目标。 */
    float lateral_acceleration_mps2;      /**< 低通滤波后的转弯向心加速度。 */
    float roll_lean_target_rad;           /**< 根据向心加速度生成的主动内倾目标。 */
    float effective_roll_target_rad;      /**< 用户 ROLL 目标与主动内倾目标之和。 */
    float total_vertical_force_n;         /**< 限幅和斜率限制后左右腿实际竖直力之和。 */
    float roll_moment_nm;                 /**< 左右腿竖直力差实际产生的 ROLL 力矩。 */
    float left_leg_force_n;               /**< 输出给左腿 VMC 的轴向力指令。 */
    float right_leg_force_n;              /**< 输出给右腿 VMC 的轴向力指令。 */
    float last_left_leg_force_n;          /**< 左腿上一周期轴向力，用于力变化率限制。 */
    float last_right_leg_force_n;         /**< 右腿上一周期轴向力，用于力变化率限制。 */
} Body_Support_Control_t;

/**
 * @brief 初始化机身支撑控制器。
 * @param support 控制器实例。
 */
void Body_Support_Init(Body_Support_Control_t *support);

/**
 * @brief 清零高度、ROLL 和输出力内部状态。
 * @param support 控制器实例。
 */
void Body_Support_Reset(Body_Support_Control_t *support);

/**
 * @brief 更新机身平均高度、ROLL 稳定和左右腿轴向支撑力。
 *
 * @param support 控制器实例。
 * @param vmc VMC 正运动学以及腿长、腿角和速度数据。
 * @param imu IMU 姿态与角速度数据。
 * @param target_height_m 用户要求的机身竖直高度目标，单位 m。
 * @param target_roll_rad 用户要求的机身 ROLL 目标，单位 rad。
 * @param forward_speed_mps 底盘前进速度，单位 m/s。
 * @param dt 控制周期，单位 s。
 */
void Body_Support_Update(Body_Support_Control_t *support,
                         const VMC_Control_t *vmc,
                         const IMU_Data_t *imu,
                         float target_height_m,
                         float target_roll_rad,
                         float forward_speed_mps,
                         float dt);

#endif
