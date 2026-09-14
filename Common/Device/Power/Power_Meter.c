//
// Created by CaoKangqi on 2026/6/19.
//
#include "Power_Meter.h"
#include <stddef.h>

//功率计接收解算函数
void CAN_Power_Rx(void *instance, uint8_t *rx_buf)
{
    if (instance == NULL || rx_buf == NULL) return;

    Power_Meter_t *Meter = (Power_Meter_t *)instance;

    int16_t raw_shunt = (int16_t)((int16_t)rx_buf[0] << 8 | rx_buf[1]);
    int16_t raw_bus   = (int16_t)((int16_t)rx_buf[2] << 8 | rx_buf[3]);
    int16_t raw_curr  = (int16_t)((int16_t)rx_buf[4] << 8 | rx_buf[5]);
    //int16_t raw_pwr   = (int16_t)((int16_t)rx_buf[6] << 8 | rx_buf[7]);
    Meter->shunt_volt = (float)raw_shunt / 1000.0f;
    Meter->bus_volt   = (float)raw_bus   / 1000.0f;
    Meter->current    = (float)raw_curr  / 1000.0f;
    //Meter->power      = (float)raw_pwr   / 100.0f;
    Meter->power      = Meter->bus_volt * Meter->current;
}
//缓冲能量计算
void Buffer_Calc(Power_Meter_t *Power, float dt, float power_limit)
{
    static uint8_t is_initialized = 0;

    if (!is_initialized) {
        Power->buffer_energy = 60.0f;
        is_initialized = 1;
    }
    float max_buffer_energy = 60.0f;
    float now_power = Power->power;
    Power->buffer_energy += (power_limit - now_power) * dt;

    if (Power->buffer_energy > max_buffer_energy) {
        Power->buffer_energy = max_buffer_energy;
    }
    else if (Power->buffer_energy < 0.0f) {
        Power->buffer_energy = 0.0f;
    }
}