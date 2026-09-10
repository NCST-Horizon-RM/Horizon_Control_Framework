//
// Created by CaoKangqi on 2026/1/27.
//

#ifndef G4_FRAMEWORK_IMU_TASK_H
#define G4_FRAMEWORK_IMU_TASK_H

#include "main.h"
#include "tim.h"
#include "Classic_Control.h"

typedef enum
{
    TEMP_INIT = 0,   // 温控状态初始化，初始化变量、清零 PID、启动加热相关外设
    TEMP_PID_CTRL,   // PID 控制加热阶段
    TEMP_STABLE,     // 温度稳定状态
    GYRO_CALIB,      // 陀螺仪校准阶段
    FUSION_RUN,      // 姿态融合正常运行状态
    ERROR_STATE      // 错误状态（如温控失败、校准异常等）
} IMU_CTRL_STATE_e;

typedef enum
{
    VQF = 0,    // VQF算法
    MAHONY,     // Mahony算法
    EKF         // EKF算法
} IMU_Fusion_Algo_e;

typedef struct
{
    uint8_t temp_reached;      // 到达目标温度
    uint8_t temp_stable;       // 温度稳定
    uint8_t gyro_calib_done;   // 陀螺仪零漂完成
    uint8_t fusion_enabled;    // 融合算法使能
} IMU_CTRL_FLAG_t;

typedef struct __attribute__((aligned(8)))
{
    float gyro_correct[3];
    float accel_bias[3];     //加速度计零偏标定
    float accel_correct[3];
    float accel_scale[3];    //加速度计尺度因子标定
    float gyro[3];
    float accel[3];
    float temp;

    float q[4];

    float pitch;
    float roll;
    float yaw;
    float YawTotalAngle;
}IMU_Data_t;

extern IMU_Data_t IMU_Data;
extern IMU_CTRL_STATE_e imu_ctrl_state;
extern IMU_CTRL_FLAG_t imu_ctrl_flag;

void Set_Heat_Power(float pwm);
void IMU_Temp_PID_Init(void);
void IMU_Update_Task(IMU_Data_t *IMU,float dt_s);
void IMU_Gyro_Zero_Calibration_Task(IMU_Data_t *IMU);
void IMU_Status_Check(IMU_Data_t *IMU);

void IMU_Fusion_Init(IMU_Fusion_Algo_e algo, float dt);
void IMU_Fusion_Update(IMU_Data_t *IMU, float dt);
#endif //G4_FRAMEWORK_IMU_TASK_H