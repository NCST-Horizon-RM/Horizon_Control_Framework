//
// Created by CaoKangqi on 2026/2/14.
//
#ifndef HORIZON_DM_MOTOR_H
#define HORIZON_DM_MOTOR_H

#include "BSP_CAN.h"
#include "Horizon_MATH.h"
#include "Offline_Detector.h"


// --- 模式偏移地址 ---
#define MIT_MODE      0x000
#define POS_MODE      0x100
#define SPEED_MODE    0x200

// --- 映射范围参数 ---
#define P_MIN   -12.5f
#define P_MAX    12.5f
#define V_MIN   -30.0f
#define V_MAX    30.0f
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -10.0f
#define T_MAX    10.0f

typedef struct {
    Offline_Check_t offline;
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;

    int16_t Angle_last;
    int16_t Angle_now;
    int16_t Speed_last;
    int16_t Speed_now;
    int16_t current;
    int8_t temperature;
    int32_t Angle_Infinite;
    int16_t Laps;
    float ralativeAngle;
} DM_MOTOR_DATA_Typedef;

typedef enum {
    DM_CMD_MOTOR_MODE    = 0xfc,//电机使能
    DM_CMD_RESET_MODE    = 0xfd,//电机失能
    DM_CMD_ZERO_POSITION = 0xfe,//将当前编码器位置设置为零位
    DM_CMD_CLEAR_ERROR   = 0xfb,//清除错误状态
} DMMotor_Mode_e;

// 反馈解算
void DM_Standard_Resolve(void* instance, uint8_t *rx_data);
void DM_1to4_Resolve(void* instance, uint8_t* rx_data);

// 控制发送
void Motor_Mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what);
void MIT_Ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void Pos_Speed_Ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);
void Speed_Ctrl(hcan_t* hcan, uint16_t motor_id, float vel);
void DM_Motor_Send(hcan_t* hcan, uint16_t master_id, float m1_cur, float m2_cur, float m3_cur, float m4_cur);
#endif //HORIZON_DM_MOTOR_H