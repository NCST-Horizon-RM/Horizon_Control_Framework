#ifndef HORIZON_BM_MOTOR_H
#define HORIZON_BM_MOTOR_H

#include "main.h"
#include "BSP_CAN.h"
#include "Offline_Detector.h"


typedef struct report
{

	int16_t Speed;		 // 0x01 中心轴速度*10
	int16_t Current_bus; // 0x02 母线电流*100
	int16_t Current;	 // 0x03 IQ*100
	uint16_t Position_Rotor;	 // 0x04 转子位置 0-32768
	uint8_t ErrCode;	 // 0x05 故障信息
	uint16_t WarnCode;	 // 0x06 警告信息
    uint16_t temp_MOS;     	// 0x07 MOS温度
	uint16_t temp;		 // 0x08 电机绕组温度
	uint8_t Mode;		 // 0x09 当前模式
	int16_t Voltage;	 // 0x0A 电机电压
	int32_t Round; 		 // 0x0B 电机转数*100
	uint8_t Status;		 // 0x0C 当前系统状态
	uint16_t Position_Raw; // 0x0D 绝对位置 0-32768
	int16_t Current_MaxPhase; // 0x0E 相电流最大值*100
}reporter;

extern reporter BM_reporter1;


typedef struct 
{
	Offline_Check_t offline;

	uint16_t ID;
	float aim;
	float vel;
	float IQ;
	uint32_t pos_raw[2];
	int32_t round;
	int32_t pos_init;
	int32_t pos_con;
	float voltage;
	float vel_rad;
	float pos_rad;
	float pos_init_rad;
}BM_MOTOR_DATA_Typedef;

extern BM_MOTOR_DATA_Typedef BM_motor_data;

void BM_Drive(hcan_t *hcan, uint16_t stdid ,int16_t speed, uint8_t ID);
void BM_EnableDisable(hcan_t *hcan, uint8_t mode);
void BM_query_check_status(hcan_t *hcan, uint8_t check1, uint8_t check2, uint8_t check3, uint8_t check4, reporter *report);
void BM_query_get_status(uint8_t *rx_data, uint8_t check1, uint8_t check2, uint8_t check3, uint8_t check4, reporter *report);
void BM_Motor_Resolve(void *instance, uint8_t *rx_data);
void BM_Send_torque(hcan_t *hcan, uint16_t stdid, float torque1, float torque2, float torque3, float torque4);
void BM_Send_IQ(hcan_t *hcan, uint16_t stdid ,float IQ, uint8_t ID);
void BM_set_ID(hcan_t *hcan, uint8_t ID, uint8_t new_ID);
void BM_save_flash(hcan_t* hcan);
void BM_save_zeroPoint(hcan_t *hcan);


#endif // !__BM_MOTOR_H__
