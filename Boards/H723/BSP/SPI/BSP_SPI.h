//
// Created by CaoKangqi on 2026/6/13.
//

#ifndef H7_FRAMEWORK_BSP_SPI_H
#define H7_FRAMEWORK_BSP_SPI_H

#include <stdint.h>

typedef enum {
    BSP_SPI_OK = 0,
    BSP_SPI_ERROR
} BSP_SPI_Status_t;

// 提供给 BMI088 的片选控制接口
void BSP_SPI_Accel_CS(uint8_t state);
void BSP_SPI_Gyro_CS(uint8_t state);

BSP_SPI_Status_t BSP_SPI_Transmit(const uint8_t *data, uint16_t size, uint32_t timeout);
BSP_SPI_Status_t BSP_SPI_Receive(uint8_t *data, uint16_t size, uint32_t timeout);

void BSP_SPI_RegisterIRQCallback(void (*callback)(void));

#endif //H7_FRAMEWORK_BSP_SPI_H
