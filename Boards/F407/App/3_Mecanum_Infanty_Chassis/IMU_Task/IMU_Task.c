/**
 * @file    IMU_Task.c
 * @author  CaoKangqi
 * @date    2026/01/27
 * @brief   IMU温控与校准任务
 */

#include "IMU_Task.h"
#include <math.h>

#include "BMI088.h"
#include "BSP_TIM.h"
#include "mahony_filter.h"
#include "Message_Center.h"
#include "QuaternionEKF.h"
#include "System_State.h"
#include "VQF_filter.h"

#define IMU_TARGET_TEMP        40.0f     // 目标温度 (℃)
#define TEMP_STABLE_ERR        0.5f     // 稳定判据误差
#define TEMP_STABLE_TIME_MS    1500      // 稳定持续时间 (ms)
#define GYRO_CALIB_SAMPLES     1000      // 陀螺仪采样样本数
#define HEATER_PWM_MAX         1000.0f
typedef struct {
    float kp;
    float ki;
    float kd;
} PID_Params_t;

BSP_PWM_t imu_heater_pwm = {&htim10,  TIM_CHANNEL_1, PWM_CHANNEL_NORMAL};
IMU_CTRL_STATE_e imu_ctrl_state = TEMP_INIT;// 当前控制状态
IMU_CTRL_FLAG_t  imu_ctrl_flag  = {0};// 控制状态标志
PID_t imu_temp;
FuzzyRule_t fuzzy_rule_temp;
IMU_Data_t IMU_Data = {
    .accel_bias = {0.0f, 0.0f, 0.0f},
    .accel_scale = {1.0f, 1.0f, 1.0f}
};

static const PID_Params_t base_pid = {400.0f, 0.45f, 0.0f};
static PID_Params_t current_pid;
static uint32_t temp_stable_tick = 0;// 温度稳定计时起点
static uint16_t imu_pid_cnt      = 0;//PID控制计数器，用于10ms分频执行PID计算
static uint16_t gyro_calib_cnt   = 0;//陀螺仪校准计数
static float heater_pwm_out   = 0;// 当前加热片PWM输出值
static Publisher_t* imu_pub = NULL;
static IMU_Fusion_Algo_e current_fusion_algo = VQF;
/**
 * @brief 设置加热片PWM输出
 * @param pwm 目标PWM值 (0.0f - HEATER_PWM_MAX)
 * @note 该函数会自动进行限幅保护，确保PWM值在安全范围内
 */
void Set_Heater_PWM(float pwm) {
    // 限幅保护
    pwm = (pwm < 0.0f) ? 0.0f : (pwm > HEATER_PWM_MAX) ? HEATER_PWM_MAX : pwm;
    BSP_PWM_Set_Compare(&imu_heater_pwm, pwm);
}
/**
 * @brief 初始化PID结构体与模糊规则
 * @note  仅在系统启动或状态机复位时调用一次
 */
void IMU_Temp_Control_Init(void)
{
    BSP_PWM_Start(&imu_heater_pwm);
    // 初始化PID控制器
    PID_Init(&imu_temp,1000.0f,250.0f,(float*)&base_pid,
             7.5f,0.0f,0.0f,0.0f,0,
             Trapezoid_Intergral |ChangingIntegrationRate |
             Derivative_On_Measurement |DerivativeFilter |
             Integral_Limit |OutputFilter);
    imu_temp.FuzzyRule = NULL;
    // 初始化模糊规则参数
    Fuzzy_Rule_Init(&fuzzy_rule_temp, NULL, NULL, NULL,
        -75.0f, -0.35f, 0.0f, // Kp, Ki, Kd Ratios
        1.5f,0.125f);
    current_pid = base_pid;
}
/**
 * @brief IMU数据更新与控制状态机执行函数
 * @note 该函数在每次IMU数据更新后调用，负责执行温控PID计算、状态转换和陀螺仪校准等核心逻辑
 */
void IMU_Update_Task(IMU_Data_t *IMU,float dt_s)
{
    float now_temp = IMU->temp;
    BMI088_Read_Fast(IMU->gyro, IMU->accel, &IMU->temp);
    IMU_Status_Check(IMU);// 监测IMU数据，若不正常则进入错误状态
    if (imu_ctrl_state != TEMP_INIT)
    {
        if (++imu_pid_cnt >= 10) // 10ms 分频执行
        {
            imu_pid_cnt = 0;
            float temp_err = IMU_TARGET_TEMP - now_temp;
            if (temp_err > 10.0f)
            {
                heater_pwm_out = HEATER_PWM_MAX;
                PID_Clear(&imu_temp);
            }
            else
            {
                Fuzzy_Rule_Implementation(&fuzzy_rule_temp, now_temp, IMU_TARGET_TEMP);
                current_pid.kp = base_pid.kp + (fuzzy_rule_temp.KpFuzzy * fuzzy_rule_temp.KpRatio);
                current_pid.ki = base_pid.ki + (fuzzy_rule_temp.KiFuzzy * fuzzy_rule_temp.KiRatio);
                current_pid.kd = base_pid.kd + (fuzzy_rule_temp.KdFuzzy * fuzzy_rule_temp.KdRatio);
                float t_kpid[3] = {current_pid.kp, current_pid.ki, current_pid.kd};
                PID_set(&imu_temp, t_kpid);
                heater_pwm_out = PID_Calculate(&imu_temp, now_temp, IMU_TARGET_TEMP);
            }
            Set_Heater_PWM(heater_pwm_out);
        }
    }

    switch (imu_ctrl_state)
    {
        case TEMP_INIT:
            IMU_Temp_Control_Init();
            //姿态滤波算法初始化，目前有三种算法，VQF、Mahony、EKF
            //VQF姿态解算，Yaw漂移较低，，p/r收敛速度大于EKF慢于Mahony，抗扰能力较强，综合效果最好，建议用这个
            //Mahony姿态解算，Yaw漂移较低，p/r收敛速度最快，但是抗扰能力差，波形噪声大
            //EKF姿态解算，目前这一套不好用，Yaw漂移严重，p/r收敛速度慢且无法收敛到正确角度，待优化，不建议用
            IMU_Fusion_Init(VQF, dt_s);
            System_State_Report(ID_IMU,STATUS_INIT);
#ifdef DEBUG_MODE
            //DEBUG模式，不跳过状态
            imu_ctrl_state = TEMP_PID_CTRL;
#endif
#ifdef RELEASE_MODE
            //Release模式，直接跳到零漂校准，节省时间
            imu_ctrl_state = GYRO_CALIB;
#endif
            break;
        case TEMP_PID_CTRL:
            if (fabsf(now_temp - IMU_TARGET_TEMP) < TEMP_STABLE_ERR)
            {
                imu_ctrl_flag.temp_reached = 1;
                temp_stable_tick = HAL_GetTick();
                imu_ctrl_state = TEMP_STABLE;
            }
            break;
        case TEMP_STABLE:
            if (fabsf(now_temp - IMU_TARGET_TEMP) < TEMP_STABLE_ERR){
                if (HAL_GetTick() - temp_stable_tick > TEMP_STABLE_TIME_MS){
                    imu_ctrl_flag.temp_stable = 1;
                    imu_ctrl_state = GYRO_CALIB;
                }
            }
            else{
                imu_ctrl_state = TEMP_PID_CTRL;
            }
            break;
        case GYRO_CALIB:
            System_State_Report(ID_IMU,STATUS_PREPARING);
            IMU_Gyro_Zero_Calibration_Task(IMU);
            if (imu_ctrl_flag.gyro_calib_done){
                imu_ctrl_flag.gyro_calib_done = 0;
                gyro_calib_cnt = 0;
                IMU->accel_correct[0]=0;
                IMU->accel_correct[1]=0;
                IMU->accel_correct[2]=0;
                imu_ctrl_flag.gyro_calib_done = 0;
                gyro_calib_cnt = 0;
                imu_ctrl_state = FUSION_RUN;
            }
            break;
        case FUSION_RUN:
            System_State_Report(ID_IMU,STATUS_RUN);
            //旋转矩阵切换
            const float AXIS_MAP[3][3] = {
                {1.0f, 0.0f, 0.0f}, // Logical X = + Physical X
                {0.0f, 1.0f, 0.0f}, // Logical Y = + Physical Y
                {0.0f, 0.0f, 1.0f}  // Logical Z = + Physical Z
            };
            float gyro_phy[3];
            float accel_phy[3];
            for (int i = 0; i < 3; i++) {
                gyro_phy[i]  = IMU->gyro[i] - IMU->gyro_correct[i];
                accel_phy[i] = (IMU->accel[i] - IMU->accel_bias[i]) * IMU->accel_scale[i];
            }
            for (int i = 0; i < 3; i++) {
                IMU->gyro[i]  = 0.0f;
                IMU->accel[i] = 0.0f;
                for (int j = 0; j < 3; j++) {
                    IMU->gyro[i]  += AXIS_MAP[i][j] * gyro_phy[j];
                    IMU->accel[i] += AXIS_MAP[i][j] * accel_phy[j];
                }
            }

            IMU_Fusion_Update(IMU, dt_s);
            //注册IMU Topic
            if (imu_pub == NULL) {
                imu_pub = PubRegister("imu_data", IMU, sizeof(IMU));
            }
            //发布IMU数据
            PubPushMessage(imu_pub, IMU);
            imu_ctrl_flag.fusion_enabled = 1;
            break;
        case ERROR_STATE:
            System_State_Report(ID_IMU,STATUS_ERROR);
            if (BMI088_Init() == 1) // 尝试重新初始化IMU，成功则认为错误已恢复
            {
                imu_ctrl_state = TEMP_INIT; // 成功则回到初始状态
                break;
            }
            Set_Heater_PWM(0); // 关闭加热片
            break;
        default:
            break;
    }
}

/**
 * @brief 陀螺仪零偏校准任务
 * @note  该函数在GYRO_CALIB状态下被周期调用，累计采样数据进行平均，完成后设置校准完成标志
 */
void IMU_Gyro_Zero_Calibration_Task(IMU_Data_t *IMU)
{
    static float gyro_sq_sum[3] = {0};
    static float accel_sq_sum[3] = {0};

    if (imu_ctrl_flag.gyro_calib_done) return;
    // 累加数据与平方和
    for (int i = 0; i < 3; i++)
    {
        IMU->gyro_correct[i]  += IMU->gyro[i];
        IMU->accel_correct[i] += IMU->accel[i];

        gyro_sq_sum[i]  += IMU->gyro[i]  * IMU->gyro[i];
        accel_sq_sum[i] += IMU->accel[i] * IMU->accel[i];
    }
    gyro_calib_cnt++;
    // 采样未完成，直接返回等待下次调用
    if (gyro_calib_cnt < GYRO_CALIB_SAMPLES) return;
    // 到达采样数量，开始计算方差
    const float div = 1.0f / (float)GYRO_CALIB_SAMPLES;
    uint8_t is_stable = 1;
    for (int i = 0; i < 3; i++)
    {
        // 计算当前均值
        float mean_g = IMU->gyro_correct[i] * div;
        float mean_a = IMU->accel_correct[i] * div;
        // 方差公式：Var = E(x²) - (E(x))²
        float gyro_var  = (gyro_sq_sum[i] * div) - (mean_g * mean_g);
        float accel_var = (accel_sq_sum[i] * div) - (mean_a * mean_a);
        // 判定阈值，如果超过阈值，认为数据不稳定，需重新采集
        if (gyro_var > 0.005f || accel_var > 0.005f)
        {
            is_stable = 0;
            break;
        }
    }
    if (is_stable)
    {
        // 判定稳定：计算最终均值并结束校准
        for (int i = 0; i < 3; i++)
        {
            IMU->gyro_correct[i] *= div;
            IMU->accel_correct[i] *= div;
        }
        imu_ctrl_flag.gyro_calib_done = 1;
    }
    else
    {
        // 判定不稳定：清零累加器，下一周期自动重新开始
        for (int i = 0; i < 3; i++)
        {
            IMU->gyro_correct[i] = 0.0f;
            IMU->accel_correct[i] = 0.0f;
            gyro_sq_sum[i] = 0.0f;
            accel_sq_sum[i] = 0.0f;
        }
    }
    gyro_calib_cnt = 0; // 重置计数器
}

/**
 * @brief IMU数据状态检查，包含静态零值检测、数据卡死检测和温度边界保护
 * @note  该函数在每次IMU数据更新后调用，若检测到异常则将状态机切换到ERROR_STATE
 */
void IMU_Status_Check(IMU_Data_t *IMU) {
    static float last_sum = 0;
    static uint16_t stuck_cnt = 0;
    static uint16_t zero_cnt = 0;

    // 静态零值检测，判断加速度或陀螺仪是否全为0
    if ((fabsf(IMU->accel[0]) < 1e-6f && fabsf(IMU->accel[1]) < 1e-6f && fabsf(IMU->accel[2]) < 1e-6f)
     || (fabsf(IMU->gyro[0]) < 1e-6f && fabsf(IMU->gyro[1]) < 1e-6f && fabsf(IMU->gyro[2]) < 1e-6f))
    {
        if (++zero_cnt >= 4) { // 连续4个周期全为0才判定为异常
            imu_ctrl_state = ERROR_STATE;
        }
    } else {
        zero_cnt = 0; // 只要有数据不为 0，立即重置计数器
    }
    // 数据卡死检测
    // 将六轴数据求和，若连续 100 次采样完全一致，判定为传感器内部逻辑死锁
    float sum = 0;
    for(int i=0; i<3; i++) {
        sum += IMU->accel[i] + IMU->gyro[i];
    }

    if (fabsf(sum - last_sum) < 1e-7f) {
        if (++stuck_cnt > 100) {
            imu_ctrl_state = ERROR_STATE;
        }
    } else {
        stuck_cnt = 0;
        last_sum = sum;
    }
    // 温度边界保护
    if (IMU->temp > 50.0f || IMU->temp < 0.0f) {
        imu_ctrl_state = ERROR_STATE;
    }
}

/**
 * @brief 统一的姿态解算初始化接口
 * @param algo 选择的算法枚举
 * @param dt 采样周期(s)
 */
void IMU_Fusion_Init(IMU_Fusion_Algo_e algo, float dt)
{
    current_fusion_algo = algo;

    switch (current_fusion_algo)
    {
        case VQF:
            vqf_init(&vqf_filter, 0.001f);
            break;

        case MAHONY:
            mahony_init(&mahony_filter, 2.0f, 0.01f, 0.9f, dt);
            break;

        case EKF:
            IMU_QuaternionEKF_Init(10, dt, 10000000, 1, 0.001f, 0);
            break;

        default:
            break;
    }
}

/**
 * @brief 统一的姿态解算更新接口
 * @param IMU  IMU数据结构体指针
 * @param dt   采样周期(s)
 */
void IMU_Fusion_Update(IMU_Data_t *IMU, float dt)
{
    switch (current_fusion_algo)
    {
        case VQF:
            vqf_update(&vqf_filter,
                       IMU->gyro[0], IMU->gyro[1], IMU->gyro[2],
                       IMU->accel[0], IMU->accel[1], IMU->accel[2]);
            vqf_output(&vqf_filter);

            IMU->q[0] = vqf_filter.q[0];
            IMU->q[1] = vqf_filter.q[1];
            IMU->q[2] = vqf_filter.q[2];
            IMU->q[3] = vqf_filter.q[3];
            IMU->pitch = vqf_filter.pitch;
            IMU->roll  = vqf_filter.roll;
            IMU->yaw   = vqf_filter.yaw;
            IMU->YawTotalAngle = vqf_filter.YawTotalAngle;
            break;

        case MAHONY:
            mahony_update(&mahony_filter,
                          IMU->gyro[0], IMU->gyro[1], IMU->gyro[2],
                          IMU->accel[0], IMU->accel[1], IMU->accel[2], dt);
            mahony_output(&mahony_filter);

            IMU->q[0] = mahony_filter.q[0];
            IMU->q[1] = mahony_filter.q[1];
            IMU->q[2] = mahony_filter.q[2];
            IMU->q[3] = mahony_filter.q[3];
            IMU->pitch = mahony_filter.pitch;
            IMU->roll  = mahony_filter.roll;
            IMU->yaw   = mahony_filter.yaw;
            IMU->YawTotalAngle = mahony_filter.YawTotalAngle;
            break;

        case EKF:
            IMU_QuaternionEKF_Update(IMU->gyro[0], IMU->gyro[1], IMU->gyro[2],
                                     IMU->accel[0], IMU->accel[1], IMU->accel[2]);

            IMU->q[0] = QEKF_INS.q[0];
            IMU->q[1] = QEKF_INS.q[1];
            IMU->q[2] = QEKF_INS.q[2];
            IMU->q[3] = QEKF_INS.q[3];
            IMU->pitch = QEKF_INS.Pitch;
            IMU->roll  = QEKF_INS.Roll;
            IMU->yaw   = QEKF_INS.Yaw;
            IMU->YawTotalAngle = QEKF_INS.YawTotalAngle;
            break;

        default:
            break;
    }
}