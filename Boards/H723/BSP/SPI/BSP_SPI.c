//
// Created by CaoKangqi on 2026/6/13.
//
#include "BSP_SPI.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
#define BMI088_SPI_HANDLE &hspi2

// 静态函数指针，用于隔离外部中断
static void (*imu_irq_callback)(void) = NULL;

void BSP_SPI_Accel_CS(uint8_t state) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_SPI_Gyro_CS(uint8_t state) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, (state) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

BSP_SPI_Status_t BSP_SPI_Transmit(const uint8_t *data, uint16_t size, uint32_t timeout) {
    if (HAL_SPI_Transmit(BMI088_SPI_HANDLE, (uint8_t *)data, size, timeout) == HAL_OK) {
        return BSP_SPI_OK;
    }
    return BSP_SPI_ERROR;
}

BSP_SPI_Status_t BSP_SPI_Receive(uint8_t *data, uint16_t size, uint32_t timeout) {
    if (HAL_SPI_Receive(BMI088_SPI_HANDLE, data, size, timeout) == HAL_OK) {
        return BSP_SPI_OK;
    }
    return BSP_SPI_ERROR;
}

void BSP_SPI_RegisterIRQCallback(void (*callback)(void)) {
    imu_irq_callback = callback;
}

// 拦截 HAL 层的 EXTI 中断
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GYRO_INT_Pin) {
        if (imu_irq_callback != NULL) {
            imu_irq_callback(); // 调用应用层或任务层注册进来的函数
        }
    }
}