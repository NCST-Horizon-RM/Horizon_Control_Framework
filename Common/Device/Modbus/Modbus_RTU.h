/**
 * @file Modbus_RTU.h
 * @brief Modbus RTU 协议实现
 * @date 2026-09-06
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stdbool.h>

/* Modbus 功能码 */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10

/* Modbus 异常码 */
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION       0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE     0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE   0x04

/* 协议限制 */
#define MODBUS_RTU_MAX_ADU_LENGTH    256
#define MODBUS_RTU_MIN_FRAME_LENGTH  4    // addr + fc + crc_low + crc_high
#define MODBUS_RTU_CRC_LENGTH        2

/* 响应解析结果 */
typedef enum {
    MODBUS_PARSE_OK = 0,                    // 解析成功
    MODBUS_PARSE_INVALID_LENGTH,            // 无效长度
    MODBUS_PARSE_INVALID_CRC,               // CRC 校验错误
    MODBUS_PARSE_INVALID_SLAVE_ADDR,        // 无效从机地址
    MODBUS_PARSE_INVALID_FUNCTION_CODE,     // 无效功能码
    MODBUS_PARSE_EXCEPTION_RESPONSE,        // 异常响应
    MODBUS_PARSE_DATA_MISMATCH              // 数据不匹配
} Modbus_ParseResult_e;

/**
 * @brief 计算 Modbus RTU CRC16 校验
 * @param data: 数据缓冲区
 * @param length: 数据长度
 * @return CRC16 值（低字节在前）
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length);

/**
 * @brief 构建读取线圈的 Modbus RTU 请求（功能码 0x01）
 * @param buffer: 输出缓冲区（至少 8 字节）
 * @param slave_addr: 从机地址 (1-247)
 * @param start_addr: 起始地址 (0x0000 - 0xFFFF)
 * @param quantity: 要读取的线圈数量 (1-2000)
 * @return 请求帧长度
 */
uint16_t Modbus_BuildReadCoils(uint8_t *buffer, uint8_t slave_addr,
                                uint16_t start_addr, uint16_t quantity);

// FC05：开启编码 FF00，关闭编码 0000；buffer 至少 8 字节。
uint16_t Modbus_BuildWriteSingleCoil(uint8_t *buffer, uint8_t slave_addr,
                                    uint16_t address, bool on);
// 校验 FC05 应答的站号、CRC、地址与写入值。成功仅代表协议确认，仍需 FC01 读回。
Modbus_ParseResult_e Modbus_ParseWriteSingleCoilResponse(const uint8_t *response,
    uint16_t length, uint8_t slave_addr, uint16_t address, bool on);

/**
 * @brief 构建读取离散输入的 Modbus RTU 请求（功能码 0x02）
 * @param buffer: 输出缓冲区（至少 8 字节）
 * @param slave_addr: 从机地址 (1-247)
 * @param start_addr: 起始地址 (0x0000 - 0xFFFF)
 * @param quantity: 要读取的输入数量 (1-2000)
 * @return 请求帧长度
 */
uint16_t Modbus_BuildReadDiscreteInputs(uint8_t *buffer, uint8_t slave_addr,
                                         uint16_t start_addr, uint16_t quantity);

/**
 * @brief 构建读取保持寄存器的 Modbus RTU 请求（功能码 0x03）
 * @param buffer: 输出缓冲区（至少 8 字节）
 * @param slave_addr: 从机地址 (1-247)
 * @param start_addr: 起始地址 (0x0000 - 0xFFFF)
 * @param quantity: 要读取的寄存器数量 (1-125)
 * @return 请求帧长度
 */
uint16_t Modbus_BuildReadHoldingRegisters(uint8_t *buffer, uint8_t slave_addr,
                                           uint16_t start_addr, uint16_t quantity);

/**
 * @brief 解析读取线圈/离散输入的响应
 * @param response: 接收到的响应缓冲区
 * @param response_len: 响应长度
 * @param expected_slave_addr: 期望的从机地址
 * @param expected_fc: 期望的功能码 (0x01 或 0x02)
 * @param data_out: 线圈/输入状态的输出缓冲区（位打包）
 * @param data_out_size: 输出缓冲区大小
 * @param byte_count_out: 从响应中解析的实际字节数
 * @return 解析结果
 */
Modbus_ParseResult_e Modbus_ParseReadBitsResponse(const uint8_t *response,
                                                   uint16_t response_len,
                                                   uint8_t expected_slave_addr,
                                                   uint8_t expected_fc,
                                                   uint8_t *data_out,
                                                   uint16_t data_out_size,
                                                   uint8_t *byte_count_out);

/**
 * @brief 解析读取保持/输入寄存器的响应
 * @param response: 接收到的响应缓冲区
 * @param response_len: 响应长度
 * @param expected_slave_addr: 期望的从机地址
 * @param expected_fc: 期望的功能码 (0x03 或 0x04)
 * @param data_out: 寄存器值的输出缓冲区（大端 uint16）
 * @param data_out_size: 输出缓冲区大小（字节，应为偶数）
 * @param byte_count_out: 从响应中解析的实际字节数
 * @return 解析结果
 */
Modbus_ParseResult_e Modbus_ParseReadRegistersResponse(const uint8_t *response,
                                                        uint16_t response_len,
                                                        uint8_t expected_slave_addr,
                                                        uint8_t expected_fc,
                                                        uint8_t *data_out,
                                                        uint16_t data_out_size,
                                                        uint8_t *byte_count_out);

/**
 * @brief 检查响应是否为异常响应
 * @param response: 接收到的响应缓冲区
 * @param response_len: 响应长度
 * @param slave_addr: 期望的从机地址
 * @param function_code: 原始功能码
 * @param exception_code_out: 如果是异常响应则输出异常码
 * @return 如果是异常响应返回 true
 */
bool Modbus_IsExceptionResponse(const uint8_t *response,
                                 uint16_t response_len,
                                 uint8_t slave_addr,
                                 uint8_t function_code,
                                 uint8_t *exception_code_out);

#endif // MODBUS_RTU_H
