#ifndef HORIZON_POWER_CAP_H
#define HORIZON_POWER_CAP_H

#include <stdint.h>
#include <stdbool.h>
#include "BSP_CAN.h"
#include "Offline_Detector.h"

typedef struct {
    Offline_Check_t offline;
    uint8_t  cap_key;       // FSBB开关状态
    uint8_t  cap_state;     // 状态位 (0:正常, 1:故障)
    float    bat_voltage;   // 电池端电压
    float    nowPower;      // 当前底盘功率
    float  Cap_Capacity;  // 电容容量百分比
} CapRxData_t;

#pragma pack(1)
typedef union {
    struct {
        uint8_t power_key;       // 开关控制 (1:开, 0:关)
        uint8_t capPowerLimit;   // 功率限制 (W)
        uint8_t buffer_now;      // 裁判系统当前剩余缓冲能量
        uint8_t robot_state;     // 机器人存活状态 (1:存活, 0:死亡)
        uint8_t check_code;      // 校验位 (0xAA)
        uint8_t reserved[3];     // 凑齐 8 字节
    } Control;
    uint8_t raw_data[8];
} CapSetData_t;
#pragma pack()

typedef struct {
    CapSetData_t set;
    CapRxData_t  get;
} Cap_t;

void Power_Cap_Rx(void *instance, uint8_t *rx_buf);
void Power_Cap_Tx(hcan_t *hcan, uint16_t can_id, CapSetData_t *tx_data_ptr);

#endif //HORIZON_POWER_CAP_H