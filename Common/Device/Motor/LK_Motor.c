//
// Created by CaoKangqi on 2026/3/15.
//
#include "LK_Motor.h"
#include "BSP_CAN.h"

/**
 * @brief LK 电机协议解析
 */
void LK_Motor_Resolve(void *instance, uint8_t *RxMessage)
{
    LK_MOTOR_DATA_Typedef *DATA = instance;

    DATA->offline.last_feed_tick = HAL_GetTick();
    DATA->temp = RxMessage[1];
    DATA->Current = ((uint16_t)RxMessage[3] << 8 | RxMessage[2]);
    DATA->lastRawSpeed = DATA->rawSpeed;
    DATA->rawSpeed     = ((uint16_t)RxMessage[5] << 8 | RxMessage[4]);
    DATA->lastRawEncode = DATA->rawEncode;
    DATA->rawEncode     = ((uint16_t)RxMessage[7] << 8 | RxMessage[6]);
    if(DATA->State)
    {
        int32_t diff = DATA->rawEncode - DATA->lastRawEncode;
        if(diff < -40000)
            DATA->round++;
        else if(diff > 40000)
            DATA->round--;
        DATA->lastConEncode = DATA->conEncode;
        DATA->conEncode =
            (float)DATA->round * 360.0f +
            (float)DATA->rawEncode * 360.0f / 65536.0f;
    }
    else
    {
        DATA->conEncode =
            (float)DATA->rawEncode * 360.0f / 65536.0f;
    }
}

/**
 * @brief LK iq控制
 */
void LK_Motor_Iq_Send(hcan_t* hcan, uint16_t motor_id, int16_t iq)
{
    uint8_t data[8];

    data[0] = 0xA1;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;

    data[4] = iq;
    data[5] = iq >> 8;

    data[6] = 0x00;
    data[7] = 0x00;

    CAN_Send_Msg(hcan, 0x140 + motor_id, data, 8);
}
/**
 * @brief LK 读取电机数据
 */
void LK_Motor_Data_Read(hcan_t* hcan, uint16_t motor_id)
{
    uint8_t data[8];

    data[0] = 0x9C;
    data[1] = 0x00;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    CAN_Send_Msg(hcan, 0x140 + motor_id, data, 8);
}