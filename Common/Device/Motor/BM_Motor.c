#include "BM_Motor.h"
#include "BSP_CAN.h"
// 后有大写的代码为经常调用代码

reporter BM_reporter1 = {0};
BM_MOTOR_DATA_Typedef BM_motor_data = {0};

// 驱动指令
// @param stdid 0x032 1-4
// @param stdid 0x033 5-8
// @param mode  0x00
void BM_Drive(hcan_t *hcan, uint16_t stdid ,int16_t speed, uint8_t ID)
{
    //uint16_t i = 0;
    uint8_t TxData[8] = {0};
    TxData[ID - 1] = speed >> 8;
    TxData[ID] = speed & 0x00ff;

    CAN_Send_Msg(hcan, stdid, TxData, 8);
}


void BM_Send_IQ(hcan_t *hcan, uint16_t stdid ,float IQ, uint8_t ID)
{
    int16_t speed = (int16_t)(IQ * 100.0f);
    BM_Drive(hcan, stdid, speed, ID);
}

void BM_Send_torque(hcan_t *hcan, uint16_t stdid, float torque1, float torque2, float torque3, float torque4)
{
    int16_t iq1 = (int16_t)(-torque1 * 100.0f * 1.1785113019166666666666666666667f);
    int16_t iq2 = (int16_t)(-torque2 * 100.0f * 1.1785113019166666666666666666667f);
    int16_t iq3 = (int16_t)(-torque3 * 100.0f * 1.1785113019166666666666666666667f);
    int16_t iq4 = (int16_t)(-torque4 * 100.0f * 1.1785113019166666666666666666667f);

    uint8_t TxData[8] = {0};
    TxData[0] = (uint8_t)((iq1 >> 8) & 0xFF);
    TxData[1] = (uint8_t)(iq1 & 0xFF);
    TxData[2] = (uint8_t)((iq2 >> 8) & 0xFF);
    TxData[3] = (uint8_t)(iq2 & 0xFF);
    TxData[4] = (uint8_t)((iq3 >> 8) & 0xFF);
    TxData[5] = (uint8_t)(iq3 & 0xFF);
    TxData[6] = (uint8_t)((iq4 >> 8) & 0xFF);
    TxData[7] = (uint8_t)(iq4 & 0xFF);

    CAN_Send_Msg(hcan, stdid, TxData, 8);
}


// 握手接收数据解析
// ID 0x50 + ID  （原BM_Parse_drive）
void BM_Motor_Resolve(void *instance, uint8_t *rx_data)
{
    BM_MOTOR_DATA_Typedef *DATA = instance;

    DATA->offline.last_feed_tick = HAL_GetTick();
    int16_t data[4] = {0};
    int16_t diff = 0;
    data[0] = ((int16_t)(rx_data[0] << 8) | rx_data[1]);
    data[1] = ((int16_t)(rx_data[2] << 8) | rx_data[3]);
    data[2] = ((int16_t)(rx_data[4] << 8) | rx_data[5]);
    data[3] = ((int16_t)(rx_data[6] << 8) | rx_data[7]);

    DATA->vel = (float)data[0] * 0.1f;
    DATA->IQ = (float)data[1]  * 0.01f;
    DATA->pos_raw[0] = data[2];
    DATA->voltage = (float)data[3] * 0.1f;

    diff = DATA->pos_raw[0] - DATA->pos_raw[1];
    if (diff > 16384)
        DATA->round --;
    else if (diff < -16384)
        DATA->round ++;

    DATA->pos_raw[1] = DATA->pos_raw[0];
    DATA->pos_con = DATA->round * 32768 + (float)DATA->pos_raw[0];

    DATA->vel_rad = -DATA->vel * 0.10471975511965976666666666666667f; // *pi/30
    DATA->pos_rad = -(float)DATA->pos_con * 0.00019174759848570513916015625f - DATA->pos_init_rad; // *pi/32768
    // DATA->pos_init_rad = -(float)DATA->pos_init * 0.00019174759848570513916015625f; // *pi/32768
}




// 反馈方式设置
// @brief 应该能得到4个电机的反馈
void BM_set_feedback(hcan_t *hcan, uint8_t ID, uint8_t feedback_mode, uint8_t time, uint8_t check1,
                        uint8_t check2, uint8_t check3, uint8_t check4)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x034;
    TxData[0] = ID;
    TxData[1] = feedback_mode;
    TxData[2] = time;
    TxData[3] = check1;
    TxData[4] = check2;
    TxData[5] = check3;
    TxData[6] = check4;
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

// 简化查询代码
static inline void BM_set_report_field(reporter *report, uint8_t code, uint16_t value)
{
    switch (code)
    {
    case 0x01: report->Speed = value; break;
    case 0x02: report->Current_bus = value; break;
    case 0x03: report->Current = value; break;
    case 0x04: report->Position_Rotor = value; break;
    case 0x05: report->ErrCode = value; break;
    case 0x06: report->WarnCode = value; break;
    case 0x07: report->temp_MOS = value; break;
    case 0x08: report->temp = value; break;
    case 0x09: report->Mode = value; break;
    case 0x0A: report->Voltage = value; break;
    case 0x0B: report->Round = value; break;
    case 0x0C: report->Status = value; break;
    case 0x0D: report->Position_Raw = value; break;
    case 0x0E: report->Current_MaxPhase = value; break;
    default: break;
    }
}
// 主动数据查询
// @brief 主动查询电机状态
void BM_query_check_status(hcan_t *hcan, uint8_t check1, uint8_t check2, uint8_t check3, uint8_t check4, reporter *report)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x035;
    TxData[0] = check1;
    TxData[1] = check2;
    TxData[2] = check3;
    TxData[3] = check4;
    TxData[4] = 0;
    TxData[5] = 0;
    TxData[6] = 0;
    TxData[7] = 0;
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

// 主动数据查询指令
// @brief 0x70 + ID
void BM_query_get_status(uint8_t *rx_data, uint8_t check1, uint8_t check2, uint8_t check3, uint8_t check4, reporter *report)
{
    uint16_t value[4] = {0};
    value[0] = ((uint16_t)(rx_data[0] << 8) | rx_data[1]);
    value[1] = ((uint16_t)(rx_data[2] << 8) | rx_data[3]);
    value[2] = ((uint16_t)(rx_data[4] << 8) | rx_data[5]);
    value[3] = ((uint16_t)(rx_data[6] << 8) | rx_data[7]);

    BM_set_report_field(report, check1, value[0]);
    BM_set_report_field(report, check2, value[1]);
    BM_set_report_field(report, check3, value[2]);
    BM_set_report_field(report, check4, value[3]);
}


// 参数设置指令
// @param mode 0x1C 设置模式 电流/速度/位置 2/3/4  
//             0x2A 设置ID
void BM_set_param(hcan_t *hcan, uint8_t ID, uint8_t Identifier, uint8_t mode)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x036;
    TxData[0] = ID;
    TxData[1] = Identifier;
    TxData[2] = mode;
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

// 保存flash
void BM_save_flash(hcan_t* hcan)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x039;
    TxData[0] = 0x01;
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

// 设置零点
void BM_save_zeroPoint(hcan_t *hcan)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x039;
    TxData[1] = 0x01;
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

// 模式设置
// @param mode 0x1C 设置模式 电流/速度/位置 2/3/4  
void BM_set_Mode(hcan_t *hcan, uint8_t ID, uint8_t mode)
{
    BM_set_param(hcan, ID, 0x1C, mode);
}

// ID设置
//  @param new_ID 0x2A 设置ID
void BM_set_ID(hcan_t *hcan, uint8_t ID, uint8_t new_ID)
{
    BM_set_param(hcan, ID, 0x2A, new_ID);
}

// 设状态控制指令
// @param mode 0x01 全部失能
//             0x02 全部使能
void BM_EnableDisable(hcan_t *hcan, uint8_t mode)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x038;
    for (int i = 0; i < 8; i++)
    {
        TxData[i] = mode;
    }
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

void BM_enable(hcan_t *hcan, uint8_t ID)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x038;
    TxData[ID - 1] = 0x02;
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

void BM_disable(hcan_t *hcan, uint8_t ID)
{
    uint8_t TxData[8] = {0};
    uint16_t stdid = 0x038;
    TxData[ID - 1] = 0x01;
    CAN_Send_Msg(hcan, stdid, TxData, 8);
}

// void BM_