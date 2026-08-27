//
// Created by CaoKangqi on 2026/7/6.
//

#ifndef HORIZON_BMI088_H
#define HORIZON_BMI088_H

#include <stdint.h>
#include "BSP_SPI.h"

// --- ACCEL REGISTERS ---
#define REG_ACC_CHIP_ID         0x00
#define REG_ACC_STATUS          0x03
#define REG_ACC_XOUT_L          0x12
#define REG_TEMP_M              0x22
#define REG_TEMP_L              0x23
#define REG_ACC_CONF            0x40
#define REG_ACC_RANGE           0x41
#define REG_ACC_INT1_IO_CTRL    0x53
#define REG_ACC_INT_MAP_DATA    0x58
#define REG_ACC_PWR_CONF        0x7C
#define REG_ACC_PWR_CTRL        0x7D
#define REG_ACC_SOFTRESET       0x7E

// --- GYRO REGISTERS ---
#define REG_GYRO_CHIP_ID        0x00
#define REG_GYRO_X_L            0x02
#define REG_GYRO_INT_STAT_1     0x0A
#define REG_GYRO_RANGE          0x0F
#define REG_GYRO_BANDWIDTH      0x10
#define REG_GYRO_LPM1           0x11
#define REG_GYRO_SOFTRESET      0x14
#define REG_GYRO_CTRL           0x15
#define REG_GYRO_INT3_INT4_IO_CONF 0x16
#define REG_GYRO_INT3_INT4_IO_MAP  0x18

// Constants
#define BMI088_ACC_CHIP_ID_VAL  0x1E
#define BMI088_GYRO_CHIP_ID_VAL 0x0F
#define BMI088_SOFTRESET_VAL    0xB6

/* Accel full-scale range */
typedef enum {
    ACCEL_FS_3G  = 0x00,
    ACCEL_FS_6G  = 0x01,
    ACCEL_FS_12G = 0x02,
    ACCEL_FS_24G = 0x03,
} AccelFS_t;

/* Gyro full-scale range */
typedef enum {
    GYRO_FS_2000DPS = 0x00,
    GYRO_FS_1000DPS = 0x01,
    GYRO_FS_500DPS  = 0x02,
    GYRO_FS_250DPS  = 0x03,
    GYRO_FS_125DPS  = 0x04,
} GyroFS_t;

/* Accel Output data rate */
typedef enum {
    ACC_ODR_12_5Hz = 0x05,
    ACC_ODR_25Hz   = 0x06,
    ACC_ODR_50Hz   = 0x07,
    ACC_ODR_100Hz  = 0x08,
    ACC_ODR_200Hz  = 0x09,
    ACC_ODR_400Hz  = 0x0A,
    ACC_ODR_800Hz  = 0x0B,
    ACC_ODR_1600Hz = 0x0C,
} AccelODR_t;

/* Gyro Output data rate & Bandwidth */
typedef enum {
    GYR_ODR_2000Hz_BW_532Hz = 0x00,
    GYR_ODR_2000Hz_BW_230Hz = 0x01,
    GYR_ODR_1000Hz_BW_116Hz = 0x02,
    GYR_ODR_400Hz_BW_47Hz   = 0x03,
    GYR_ODR_200Hz_BW_23Hz   = 0x04,
    GYR_ODR_100Hz_BW_12Hz   = 0x05,
    GYR_ODR_200Hz_BW_64Hz   = 0x06,
    GYR_ODR_100Hz_BW_32Hz   = 0x07,
} GyroODR_BW_t;

extern float acc_res;
extern float gyr_res;

uint8_t BMI088_Init(void);
void BMI088_SetFormat(AccelODR_t a_odr, AccelFS_t a_fsr, GyroODR_BW_t g_odr_bw, GyroFS_t g_fsr);
uint8_t BMI088_IsDataReady(void);

void BMI088_read(float gyro[3], float accel[3], float *temperature);
void BMI088_Read_Fast(float gyro[3], float accel[3], float *temperature);
void BMI088_ResolveRaw(const uint8_t raw_data[14], float gyro[3], float accel[3], float *temperature);

#endif //HORIZON_BMI088_H
