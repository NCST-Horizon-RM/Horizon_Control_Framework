/**
 * @file Modbus_RTU.c
 * @brief Modbus RTU 协议实现
 * @date 2026-09-06
 */

#include "Modbus_RTU.h"
#include <string.h>

/**
 * @brief 计算 Modbus RTU CRC16 校验
 * CRC16-MODBUS: 多项式 0xA001（反转的 0x8005），初始值 0xFFFF
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief 构建读取线圈的 Modbus RTU 请求（功能码 0x01）
 */
uint16_t Modbus_BuildReadCoils(uint8_t *buffer, uint8_t slave_addr,
                                uint16_t start_addr, uint16_t quantity)
{
    if (buffer == NULL || slave_addr == 0 || slave_addr > 247 ||
        quantity == 0 || quantity > 2000)
    {
        return 0;
    }

    buffer[0] = slave_addr;
    buffer[1] = MODBUS_FC_READ_COILS;
    buffer[2] = (start_addr >> 8) & 0xFF;  // 起始地址高字节
    buffer[3] = start_addr & 0xFF;         // 起始地址低字节
    buffer[4] = (quantity >> 8) & 0xFF;    // 数量高字节
    buffer[5] = quantity & 0xFF;           // 数量低字节

    uint16_t crc = Modbus_CRC16(buffer, 6);
    buffer[6] = crc & 0xFF;         // CRC 低字节
    buffer[7] = (crc >> 8) & 0xFF;  // CRC 高字节

    return 8;
}

/**
 * @brief 构建读取离散输入的 Modbus RTU 请求（功能码 0x02）
 */
uint16_t Modbus_BuildReadDiscreteInputs(uint8_t *buffer, uint8_t slave_addr,
                                         uint16_t start_addr, uint16_t quantity)
{
    if (buffer == NULL || slave_addr == 0 || slave_addr > 247 ||
        quantity == 0 || quantity > 2000)
    {
        return 0;
    }

    buffer[0] = slave_addr;
    buffer[1] = MODBUS_FC_READ_DISCRETE_INPUTS;
    buffer[2] = (start_addr >> 8) & 0xFF;  // 起始地址高字节
    buffer[3] = start_addr & 0xFF;         // 起始地址低字节
    buffer[4] = (quantity >> 8) & 0xFF;    // 数量高字节
    buffer[5] = quantity & 0xFF;           // 数量低字节

    uint16_t crc = Modbus_CRC16(buffer, 6);
    buffer[6] = crc & 0xFF;         // CRC 低字节
    buffer[7] = (crc >> 8) & 0xFF;  // CRC 高字节

    return 8;
}

/**
 * @brief 构建读取保持寄存器的 Modbus RTU 请求（功能码 0x03）
 */
uint16_t Modbus_BuildReadHoldingRegisters(uint8_t *buffer, uint8_t slave_addr,
                                           uint16_t start_addr, uint16_t quantity)
{
    if (buffer == NULL || slave_addr == 0 || slave_addr > 247 ||
        quantity == 0 || quantity > 125)
    {
        return 0;
    }

    buffer[0] = slave_addr;
    buffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    buffer[2] = (start_addr >> 8) & 0xFF;  // 起始地址高字节
    buffer[3] = start_addr & 0xFF;         // 起始地址低字节
    buffer[4] = (quantity >> 8) & 0xFF;    // 数量高字节
    buffer[5] = quantity & 0xFF;           // 数量低字节

    uint16_t crc = Modbus_CRC16(buffer, 6);
    buffer[6] = crc & 0xFF;         // CRC 低字节
    buffer[7] = (crc >> 8) & 0xFF;  // CRC 高字节

    return 8;
}

/**
 * @brief 检查响应是否为异常响应
 */
bool Modbus_IsExceptionResponse(const uint8_t *response,
                                 uint16_t response_len,
                                 uint8_t slave_addr,
                                 uint8_t function_code,
                                 uint8_t *exception_code_out)
{
    if (response == NULL || response_len != 5)
    {
        return false;
    }

    // 异常响应格式：[从机地址][功能码+0x80][异常码][crc低][crc高]
    if (response[0] == slave_addr && response[1] == (function_code | 0x80))
    {
        // 验证 CRC
        uint16_t received_crc = response[3] | (response[4] << 8);
        uint16_t calculated_crc = Modbus_CRC16(response, 3);

        if (received_crc == calculated_crc)
        {
            if (exception_code_out != NULL)
            {
                *exception_code_out = response[2];
            }
            return true;
        }
    }

    return false;
}

/**
 * @brief 解析读取线圈/离散输入的响应（功能码 0x01/0x02）
 */
Modbus_ParseResult_e Modbus_ParseReadBitsResponse(const uint8_t *response,
                                                   uint16_t response_len,
                                                   uint8_t expected_slave_addr,
                                                   uint8_t expected_fc,
                                                   uint8_t *data_out,
                                                   uint16_t data_out_size,
                                                   uint8_t *byte_count_out)
{
    // 最小响应：[地址][功能码][字节数][crc低][crc高] = 5 字节
    if (response == NULL || response_len < 5 || response_len > MODBUS_RTU_MAX_ADU_LENGTH)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    if (expected_fc != MODBUS_FC_READ_COILS && expected_fc != MODBUS_FC_READ_DISCRETE_INPUTS)
    {
        return MODBUS_PARSE_INVALID_FUNCTION_CODE;
    }

    // 检查是否为异常响应
    uint8_t exception_code;
    if (Modbus_IsExceptionResponse(response, response_len, expected_slave_addr, expected_fc, &exception_code))
    {
        return MODBUS_PARSE_EXCEPTION_RESPONSE;
    }

    // 验证从机地址
    if (response[0] != expected_slave_addr)
    {
        return MODBUS_PARSE_INVALID_SLAVE_ADDR;
    }

    // 验证功能码
    if (response[1] != expected_fc)
    {
        return MODBUS_PARSE_INVALID_FUNCTION_CODE;
    }

    // 获取字节数
    uint8_t byte_count = response[2];
    if (byte_count == 0 || byte_count > 250)
    {
        return MODBUS_PARSE_DATA_MISMATCH;
    }

    // 验证响应长度：[地址][功能码][字节数][数据...][crc低][crc高]
    uint16_t expected_len = 3 + byte_count + 2;
    if (response_len != expected_len)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    // 验证 CRC
    uint16_t received_crc = response[response_len - 2] | (response[response_len - 1] << 8);
    uint16_t calculated_crc = Modbus_CRC16(response, response_len - 2);
    if (received_crc != calculated_crc)
    {
        return MODBUS_PARSE_INVALID_CRC;
    }

    // 检查输出缓冲区大小
    if (data_out != NULL && data_out_size < byte_count)
    {
        return MODBUS_PARSE_DATA_MISMATCH;
    }

    // 复制数据
    if (data_out != NULL && byte_count > 0)
    {
        memcpy(data_out, &response[3], byte_count);
    }

    if (byte_count_out != NULL)
    {
        *byte_count_out = byte_count;
    }

    return MODBUS_PARSE_OK;
}

/**
 * @brief 解析读取保持/输入寄存器的响应（功能码 0x03/0x04）
 */
Modbus_ParseResult_e Modbus_ParseReadRegistersResponse(const uint8_t *response,
                                                        uint16_t response_len,
                                                        uint8_t expected_slave_addr,
                                                        uint8_t expected_fc,
                                                        uint8_t *data_out,
                                                        uint16_t data_out_size,
                                                        uint8_t *byte_count_out)
{
    // 最小响应：[地址][功能码][字节数][crc低][crc高] = 5 字节
    if (response == NULL || response_len < 5 || response_len > MODBUS_RTU_MAX_ADU_LENGTH)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    if (expected_fc != MODBUS_FC_READ_HOLDING_REGISTERS && expected_fc != MODBUS_FC_READ_INPUT_REGISTERS)
    {
        return MODBUS_PARSE_INVALID_FUNCTION_CODE;
    }

    // 检查是否为异常响应
    uint8_t exception_code;
    if (Modbus_IsExceptionResponse(response, response_len, expected_slave_addr, expected_fc, &exception_code))
    {
        return MODBUS_PARSE_EXCEPTION_RESPONSE;
    }

    // 验证从机地址
    if (response[0] != expected_slave_addr)
    {
        return MODBUS_PARSE_INVALID_SLAVE_ADDR;
    }

    // 验证功能码
    if (response[1] != expected_fc)
    {
        return MODBUS_PARSE_INVALID_FUNCTION_CODE;
    }

    // 获取字节数（寄存器应为偶数）
    uint8_t byte_count = response[2];
    if (byte_count == 0 || byte_count > 250 || byte_count % 2 != 0)
    {
        return MODBUS_PARSE_DATA_MISMATCH;
    }

    // 验证响应长度
    uint16_t expected_len = 3 + byte_count + 2;
    if (response_len != expected_len)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    // 验证 CRC
    uint16_t received_crc = response[response_len - 2] | (response[response_len - 1] << 8);
    uint16_t calculated_crc = Modbus_CRC16(response, response_len - 2);
    if (received_crc != calculated_crc)
    {
        return MODBUS_PARSE_INVALID_CRC;
    }

    // 检查输出缓冲区大小
    if (data_out != NULL && data_out_size < byte_count)
    {
        return MODBUS_PARSE_DATA_MISMATCH;
    }

    // 复制数据（Modbus 中寄存器为大端）
    if (data_out != NULL && byte_count > 0)
    {
        memcpy(data_out, &response[3], byte_count);
    }

    if (byte_count_out != NULL)
    {
        *byte_count_out = byte_count;
    }

    return MODBUS_PARSE_OK;
}
