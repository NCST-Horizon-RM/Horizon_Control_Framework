/* 曹康祺于 2026 年 6 月 20 日创建。 */

#ifndef H7_FRAMEWORK_CHASSIS_CTRL_H
#define H7_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "IMU_Task.h"
#include <math.h>
#include <string.h>
#include "All_define.h"
#include "Body_Support_Control.h"
#include "Classic_Control.h"
#include "Kalman_Observer.h"
#include "LQR_Control.h"
#include "Power_Ctrl.h"
#include "Robot_Cmd.h"
#include "Self_Righting.h"
#include "Step_Climb.h"
#include "System_State.h"

typedef enum {
    CTRL_PREPARE = 0,
    CTRL_STAND,
    CTRL_FALL_HOLD = 3  /* 拨回安全挡解除 */
} Chassis_Control_Mode_t;

extern Chassis_Control_Mode_t MODE;

typedef struct {
    VMC_Control_t vmc;
    Kalman_Observer_t odometry;
    LQR_Control_t lqr;
    LESO_Control_t leso;
    Body_Support_Control_t support;
    PID_t joint_pos[4], joint_vel[4];
    SelfRighting_t recovery;
    StepClimb_t step;
    SelfRighting_Torque_t last_applied;
    float capture_yaw_rad, spin_yaw_rad;
    bool stand_initialized, output_enabled;
} Chassis_Ctrl_t;

/* 初始化估计器、控制器和电机零位，输出从准备阶段开始。 */
uint8_t Chassis_Control_Init(void);

/* 由底盘控制任务周期调用，dt 为距上次调用的时间，单位秒。 */
void Chassis_Control_Task(const Chassis_Motor_Group_t *wheel_motors,
                          const Leg_Motor_Group_t *leg_feedback, const IMU_Data_t *imu, float dt);

#endif
