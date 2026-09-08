/**
 * @file Modbus_Device.c
 * @brief Modbus 设备类型和状态管理
 * @date 2026-09-06
 */

#include "Modbus_Device.h"
#include <string.h>

/**
 * @brief 初始化数字输入模块
 */
void Modbus_DigitalInput_Init(Modbus_DigitalInput_t *module,
                               uint8_t slave_addr,
                               uint16_t start_addr,
                               uint16_t input_count)
{
    if (module == NULL || slave_addr == 0 || slave_addr > 247 ||
        input_count == 0 || input_count > MODBUS_MAX_DISCRETE_CHANNELS)
    {
        return;
    }

    memset(module, 0, sizeof(Modbus_DigitalInput_t));

    module->slave_addr = slave_addr;
    module->start_addr = start_addr;
    module->input_count = input_count;
    module->state = MODBUS_STATE_IDLE;
}

/**
 * @brief 初始化继电器模块
 */
void Modbus_RelayModule_Init(Modbus_RelayModule_t *module,
                              uint8_t slave_addr,
                              uint16_t start_addr,
                              uint16_t coil_count)
{
    if (module == NULL || slave_addr == 0 || slave_addr > 247 ||
        coil_count == 0 || coil_count > MODBUS_MAX_COIL_CHANNELS)
    {
        return;
    }

    memset(module, 0, sizeof(Modbus_RelayModule_t));

    module->slave_addr = slave_addr;
    module->start_addr = start_addr;
    module->coil_count = coil_count;
    module->state = MODBUS_STATE_IDLE;
}

/**
 * @brief 构建数字输入模块的查询请求
 */
uint16_t Modbus_DigitalInput_BuildQuery(Modbus_DigitalInput_t *module)
{
    if (module == NULL)
    {
        return 0;
    }

    uint16_t len = Modbus_BuildReadDiscreteInputs(module->tx_buffer,
                                                   module->slave_addr,
                                                   module->start_addr,
                                                   module->input_count);

    if (len > 0)
    {
        module->state = MODBUS_STATE_WAITING_RESPONSE;
        module->rx_length = 0;
    }

    return len;
}

/**
 * @brief 构建继电器模块的查询请求
 */
uint16_t Modbus_RelayModule_BuildQuery(Modbus_RelayModule_t *module)
{
    if (module == NULL)
    {
        return 0;
    }

    uint16_t len = Modbus_BuildReadCoils(module->tx_buffer,
                                         module->slave_addr,
                                         module->start_addr,
                                         module->coil_count);

    if (len > 0)
    {
        module->state = MODBUS_STATE_WAITING_RESPONSE;
        module->rx_length = 0;
    }

    return len;
}

/**
 * @brief 处理数字输入模块的响应
 */
Modbus_ParseResult_e Modbus_DigitalInput_ProcessResponse(Modbus_DigitalInput_t *module,
                                                          const uint8_t *response,
                                                          uint16_t response_len,
                                                          uint32_t current_time)
{
    if (module == NULL || response == NULL)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    uint8_t data[sizeof(module->input_data)] = {0};
    uint8_t byte_count = 0;
    Modbus_ParseResult_e result = Modbus_ParseReadBitsResponse(
        response,
        response_len,
        module->slave_addr,
        MODBUS_FC_READ_DISCRETE_INPUTS,
        data,
        sizeof(data),
        &byte_count
    );

    if (result == MODBUS_PARSE_OK &&
        (module->input_count == 0 || module->input_count > MODBUS_MAX_DISCRETE_CHANNELS ||
         byte_count != (module->input_count + 7U) / 8U))
    {
        result = MODBUS_PARSE_DATA_MISMATCH;
    }

    if (result == MODBUS_PARSE_OK)
    {
        memcpy(module->input_data, data, sizeof(data));
        memmove(module->rx_buffer, response, response_len);
        module->rx_length = response_len;
        module->state = MODBUS_STATE_RESPONSE_OK;
        module->last_update_time = current_time;
        module->error_count = 0;
    }
    else
    {
        module->state = MODBUS_STATE_ERROR;
        module->error_count++;
    }

    return result;
}

/**
 * @brief 处理继电器模块的响应
 */
Modbus_ParseResult_e Modbus_RelayModule_ProcessResponse(Modbus_RelayModule_t *module,
                                                         const uint8_t *response,
                                                         uint16_t response_len,
                                                         uint32_t current_time)
{
    if (module == NULL || response == NULL)
    {
        return MODBUS_PARSE_INVALID_LENGTH;
    }

    uint8_t data[sizeof(module->coil_data)] = {0};
    uint8_t byte_count = 0;
    Modbus_ParseResult_e result = Modbus_ParseReadBitsResponse(
        response,
        response_len,
        module->slave_addr,
        MODBUS_FC_READ_COILS,
        data,
        sizeof(data),
        &byte_count
    );

    if (result == MODBUS_PARSE_OK &&
        (module->coil_count == 0 || module->coil_count > MODBUS_MAX_COIL_CHANNELS ||
         byte_count != (module->coil_count + 7U) / 8U))
    {
        result = MODBUS_PARSE_DATA_MISMATCH;
    }

    if (result == MODBUS_PARSE_OK)
    {
        memcpy(module->coil_data, data, sizeof(data));
        memmove(module->rx_buffer, response, response_len);
        module->rx_length = response_len;
        module->state = MODBUS_STATE_RESPONSE_OK;
        module->last_update_time = current_time;
        module->error_count = 0;
    }
    else
    {
        module->state = MODBUS_STATE_ERROR;
        module->error_count++;
    }

    return result;
}

/**
 * @brief 获取特定输入位的状态
 */
bool Modbus_DigitalInput_GetBit(const Modbus_DigitalInput_t *module, uint16_t index)
{
    if (module == NULL || index >= module->input_count)
    {
        return false;
    }

    uint16_t byte_index = index / 8;
    uint8_t bit_index = index % 8;

    return (module->input_data[byte_index] & (1 << bit_index)) != 0;
}

/**
 * @brief 获取特定线圈位的状态
 */
bool Modbus_RelayModule_GetBit(const Modbus_RelayModule_t *module, uint16_t index)
{
    if (module == NULL || index >= module->coil_count)
    {
        return false;
    }

    uint16_t byte_index = index / 8;
    uint8_t bit_index = index % 8;

    return (module->coil_data[byte_index] & (1 << bit_index)) != 0;
}

/**
 * @brief 处理数字输入模块的超时
 */
void Modbus_DigitalInput_HandleTimeout(Modbus_DigitalInput_t *module)
{
    if (module == NULL)
    {
        return;
    }

    module->state = MODBUS_STATE_TIMEOUT;
    module->error_count++;
}

/**
 * @brief 处理继电器模块的超时
 */
void Modbus_RelayModule_HandleTimeout(Modbus_RelayModule_t *module)
{
    if (module == NULL)
    {
        return;
    }

    module->state = MODBUS_STATE_TIMEOUT;
    module->error_count++;
}
