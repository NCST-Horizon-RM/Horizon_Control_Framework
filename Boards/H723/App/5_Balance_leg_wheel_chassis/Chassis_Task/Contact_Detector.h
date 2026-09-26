#ifndef BALANCE_LEG_WHEEL_CONTACT_DETECTOR_H
#define BALANCE_LEG_WHEEL_CONTACT_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "VMC_Control.h"

/** 接地概率达到此值时，开始确认由离地切换为接地。 */
#define CONTACT_DETECT_ENTER_THRESHOLD 0.550f
/** 接地概率降至此值时，开始确认由接地切换为离地。 */
#define CONTACT_DETECT_EXIT_THRESHOLD 0.45f
/** 连续满足切换条件的调用次数；实际确认时间等于该值乘以调用周期。 */
#define CONTACT_DETECT_CONFIRM_SAMPLES 8U

/** 单条腿的检测状态；左右腿须分别创建实例。 */
typedef struct {
    float contact_probability; /**< 本次有效输入得到的接地概率，范围 [0, 1]。 */
    uint16_t confirm_count;    /**< 当前目标状态连续满足阈值的采样次数。 */
    bool is_contact;           /**< 当前迟滞输出：true 接地，false 离地。 */
} Contact_Detector_t;

/** 初始化检测器；initial_contact 应按启动时已知的接地状态设置。 */
void Contact_Detector_Init(Contact_Detector_t *detector, bool initial_contact);

/**
 * 用腿部 VMC 观测量和 IMU Z 轴加速度更新接地状态。
 * leg 的支持力/力矩须先由 VMC_TorqueToForce 更新；accel_z_mps2
 * 须与训练 CSV 的“重力加速度”列使用相同的坐标方向和 m/s² 单位。
 * 每个控制周期调用一次；输入无效时保持原状态并清空确认计数。
 * 返回迟滞后的接地状态；detector 为 NULL 时返回 false。
 */
bool Contact_Detector_Update(Contact_Detector_t *detector,
                             const VMC_LegSite_t *leg,
                             float accel_z_mps2);

#endif
