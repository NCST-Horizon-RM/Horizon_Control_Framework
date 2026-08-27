//
// Created by CaoKangqi on 2026/2/14.
//
#include "DM_Motor.h"
#include <string.h>
#include "All_define.h"

/**
 * @brief 达妙电机数据解析函数
 * @param motor   电机结构体指针
 * @param rx_data 接收到的8字节数据数组
 * @note
 */
void DM_Standard_Resolve(void* instance, uint8_t *rx_data)
{
    if (instance == NULL || rx_data == NULL) return;

    DM_MOTOR_DATA_Typedef* DATA = instance;

    DATA->id = (rx_data[0]) & 0x0F;
    DATA->state = (rx_data[0]) >> 4;
    DATA->p_int = (rx_data[1] << 8) | rx_data[2];
    DATA->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    DATA->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];

    DATA->pos = uint_to_float(DATA->p_int, P_MIN, P_MAX, 16);
    DATA->vel = uint_to_float(DATA->v_int, V_MIN, V_MAX, 12);
    DATA->tor = uint_to_float(DATA->t_int, T_MIN, T_MAX, 12);

    DATA->Tmos = (float)(rx_data[6]);
    DATA->Tcoil = (float)(rx_data[7]);
    DATA->offline.last_feed_tick = HAL_GetTick();
}

/**
 * @brief 达妙电机一拖四模式数据解析函数
 * @param motor   电机结构体指针
 * @param rx_data 接收到的8字节数据数组
 * @note
 */
void DM_1to4_Resolve(void* instance, uint8_t* rx_data)
{
    if (instance == NULL || rx_data == NULL) return;

    DM_MOTOR_DATA_Typedef* DATA = instance;

    DATA->Angle_last = DATA->Angle_now;
    DATA->Angle_now = (int16_t)((rx_data[0] << 8) | rx_data[1]);

    int16_t spd_raw = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    int16_t cur_raw = (int16_t)((rx_data[4] << 8) | rx_data[5]);

    int16_t angleError = DATA->Angle_now - INIT_ANGLE;
    if (angleError > 4096)       angleError -= 8192;
    else if (angleError < -4096) angleError += 8192;

    DATA->ralativeAngle = angleError * 0.043945f; // 360.0f / 8192.0f

    int16_t diff = DATA->Angle_now - DATA->Angle_last;
    if      (diff < -4096) DATA->Laps++;
    else if (diff >  4096) DATA->Laps--;

    DATA->Speed_last = DATA->Speed_now;
    DATA->Speed_now  = OneFilter1(spd_raw / 100, DATA->Speed_last, 500);
    DATA->current = cur_raw;
    DATA->Tcoil   = (float)rx_data[6];
    DATA->Tmos    = (float)rx_data[7];

    DATA->offline.last_feed_tick = HAL_GetTick();
    DATA->Angle_Infinite    = (int32_t)((DATA->Laps * 8192) + DATA->Angle_now);
}

/**
 * @brief 达妙电机模式切换命令
 * @param hcan     CAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param mode_id  模式 ID (MIT_MODE, POS_MODE, SPEED_MODE)
 * @param what     具体操作 (DMMotor_Mode_e 枚举值)
 */
void Motor_Mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id, DMMotor_Mode_e what)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = what;

    CAN_Send_Msg(hcan, id, data, 8);
}

/**
 * @brief 达妙电机 MIT 模式控制
 * @param hcan     CAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param pos      目标位置 (float)
 * @param vel      目标速度 (float)
 * @param kp       位置环增益 (float)
 * @param kd       速度环增益 (float)
 * @param torq     目标扭矩 (float)
 */
void MIT_Ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t p = float_to_uint(pos, P_MIN, P_MAX, 16);
    uint16_t v = float_to_uint(vel, V_MIN, V_MAX, 12);
    uint16_t k_p = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t k_d = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = p >> 8;
    data[1] = p & 0xFF;
    data[2] = v >> 4;
    data[3] = ((v & 0x0F) << 4) | (k_p >> 8);
    data[4] = k_p & 0xFF;
    data[5] = k_d >> 4;
    data[6] = ((k_d & 0x0F) << 4) | (t >> 8);
    data[7] = t & 0xFF;

    CAN_Send_Msg(hcan, motor_id + MIT_MODE, data, 8);
}

/**
 * @brief 达妙电机位置速度控制模式
 * @param hcan     CAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param pos      目标位置 (float)
 * @param vel      目标速度 (float)
 */
void Pos_Speed_Ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel)
{
    uint8_t data[8];
    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel, 4);
    CAN_Send_Msg(hcan, motor_id + POS_MODE, data, 8);
}

/**
 * @brief 达妙电机速度控制模式
 * @param hcan     CAN 句柄
 * @param motor_id 电机 ID (Base ID)
 * @param vel      目标速度 (float)
 */
void Speed_Ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
    uint8_t data[8];
    memcpy(&data[0], &vel, sizeof(float));
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    uint16_t send_id = motor_id + SPEED_MODE;
    CAN_Send_Msg(hcan, send_id, data, 8);
}

/**
 * @brief 达妙电机一拖四模式电流控制
 * @param hcan      CAN 句柄
 * @param master_id 主控电机 ID (Base ID)
 * @param m1_cur    电机1目标电流 (float)
 * @param m2_cur    电机2目标电流 (float)
 * @param m3_cur    电机3目标电流 (float)
 * @param m4_cur    电机4目标电流 (float)
 */
void DM_Motor_Send(hcan_t* hcan, uint16_t master_id, float m1_cur, float m2_cur, float m3_cur, float m4_cur)
{
    uint8_t data[8];
    int16_t cur_val[4];

    cur_val[0] = (int16_t)(m1_cur);
    cur_val[1] = (int16_t)(m2_cur);
    cur_val[2] = (int16_t)(m3_cur);
    cur_val[3] = (int16_t)(m4_cur);

    for(int i=0; i<4; i++) {
        data[i*2]   = (uint8_t)(cur_val[i] >> 8);
        data[i*2+1] = (uint8_t)(cur_val[i] & 0xFF);
    }
    if (CAN_GetTxFreeLevel(hcan) > 0) {
        CAN_Send_Msg(hcan, master_id, data, 8);
    }
}