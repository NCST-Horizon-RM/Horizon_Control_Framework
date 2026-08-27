//
// Created by CaoKangqi on 2026/6/19.
//

#ifndef HORIZON_POWER_METER_H
#define HORIZON_POWER_METER_H

#include <stdint.h>

// 功率计测量数据
typedef struct {
    float shunt_volt;
    float bus_volt;
    float current;
    float power;           // 当前实际总功率
    float buffer_energy;   // 本地解算的缓冲能量
} Power_Meter_t;

void CAN_Power_Rx(Power_Meter_t *Power, uint8_t *rx_data);
void Buffer_Calc(Power_Meter_t *Power, float dt, float power_limit);

#endif //HORIZON_POWER_METER_H
