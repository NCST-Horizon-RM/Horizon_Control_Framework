/**
 * @file Modbus_Device.c
 * @brief Modbus 设备类型和状态管理
 * @date 2026-09-06
 */

#include "Modbus_Device.h"
#include <string.h>

static bool DigitalInput_ChannelRangeValid(uint16_t first_channel, uint16_t count)
{
    return first_channel >= 1U && first_channel <= MODBUS_MAX_DISCRETE_CHANNELS &&
           count >= 1U && count <= MODBUS_MAX_DISCRETE_CHANNELS &&
           (uint32_t)first_channel + count - 1U <= MODBUS_MAX_DISCRETE_CHANNELS;
}

bool Modbus_RelayModule_InitChannels(Modbus_RelayModule_t *module, uint8_t slave_addr,
                                     uint16_t first_channel, uint16_t count)
{
    if (module == NULL || slave_addr < 1U || slave_addr > 247U ||
        first_channel < 1U || count < 1U ||
        (uint32_t)first_channel + count - 1U > MODBUS_MAX_COIL_CHANNELS) return false;
    Modbus_RelayModule_Init(module, slave_addr, (uint16_t)(first_channel - 1U), count);
    return true;
}

bool Modbus_RelayModule_GetChannelInfo(const Modbus_RelayModule_t *module,
                                      uint16_t channel, Modbus_RelayChannelInfo_t *out)
{
    if (module == NULL || out == NULL || module->slave_addr < 1U || module->slave_addr > 247U ||
        module->coil_count < 1U || (uint32_t)module->start_addr + module->coil_count > MODBUS_MAX_COIL_CHANNELS ||
        channel <= module->start_addr || (uint32_t)channel > (uint32_t)module->start_addr + module->coil_count)
        return false;
    uint16_t index = (uint16_t)(channel - 1U - module->start_addr);
    *out = (Modbus_RelayChannelInfo_t){channel, (uint16_t)(channel - 1U),
        (uint8_t)index, (uint8_t)(index / 8U), (uint8_t)(index % 8U)};
    return true;
}

bool Modbus_RelayModule_ReadChannel(const Modbus_RelayModule_t *module, uint16_t channel, bool *value)
{
    Modbus_RelayChannelInfo_t info;
    if (value == NULL || !Modbus_RelayModule_GetChannelInfo(module, channel, &info)) return false;
    *value = Modbus_RelayModule_GetBit(module, info.response_bit);
    return true;
}

uint16_t Modbus_RelayModule_BuildWriteChannel(Modbus_RelayModule_t *module, uint16_t channel, bool on)
{
    Modbus_RelayChannelInfo_t info;
    if (!Modbus_RelayModule_GetChannelInfo(module, channel, &info)) return 0;
    return Modbus_BuildWriteSingleCoil(module->tx_buffer, module->slave_addr, info.modbus_address, on);
}

Modbus_ParseResult_e Modbus_RelayModule_ProcessWriteResponse(Modbus_RelayModule_t *module,
    const uint8_t *response, uint16_t length, uint16_t channel, bool on)
{
    Modbus_RelayChannelInfo_t info;
    if (!Modbus_RelayModule_GetChannelInfo(module, channel, &info)) return MODBUS_PARSE_DATA_MISMATCH;
    Modbus_ParseResult_e result = Modbus_ParseWriteSingleCoilResponse(response, length,
        module->slave_addr, info.modbus_address, on);
    module->state = result == MODBUS_PARSE_OK ? MODBUS_STATE_RESPONSE_OK : MODBUS_STATE_ERROR;
    if (result != MODBUS_PARSE_OK) module->error_count++;
    // FC05 应答不更新 coil_data / last_update_time；它们只来自 FC01 读回。
    return result;
}

bool Modbus_DigitalInput_InitChannels(Modbus_DigitalInput_t *module,
                                     uint8_t slave_addr,
                                     uint16_t first_channel,
                                     uint16_t input_count)
{
    if (module == NULL || slave_addr < 1U || slave_addr > 247U ||
        !DigitalInput_ChannelRangeValid(first_channel, input_count))
        return false;

    Modbus_DigitalInput_Init(module, slave_addr, (uint16_t)(first_channel - 1U), input_count);
    return true;
}

bool Modbus_DigitalInput_GetChannelInfo(const Modbus_DigitalInput_t *module,
                                       uint16_t channel,
                                       Modbus_DigitalInput_ChannelInfo_t *out)
{
    if (module == NULL || out == NULL || module->slave_addr < 1U || module->slave_addr > 247U ||
        module->start_addr >= MODBUS_MAX_DISCRETE_CHANNELS ||
        !DigitalInput_ChannelRangeValid((uint16_t)(module->start_addr + 1U), module->input_count) ||
        channel <= module->start_addr || (uint32_t)channel > (uint32_t)module->start_addr + module->input_count)
        return false;

    const uint16_t address = (uint16_t)(channel - 1U);
    const uint16_t index = (uint16_t)(address - module->start_addr);
    *out = (Modbus_DigitalInput_ChannelInfo_t){
        .channel = channel,
        .modbus_address = address,
        .response_bit = (uint8_t)index,
        .byte_index = (uint8_t)(index / 8U),
        .bit_index = (uint8_t)(index % 8U),
    };
    return true;
}

bool Modbus_DigitalInput_ReadChannel(const Modbus_DigitalInput_t *module,
                                    uint16_t channel, bool *value)
{
    Modbus_DigitalInput_ChannelInfo_t info;
    if (value == NULL || !Modbus_DigitalInput_GetChannelInfo(module, channel, &info))
        return false;

    *value = Modbus_DigitalInput_GetBit(module, info.response_bit);
    return true;
}

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

/* 单路继电器目标同步、写后读回及诊断；不依赖 BSP/HAL。 */
bool Modbus_RelayControl_Init(Modbus_RelayControl_t *c,
                              const Modbus_RelayControl_Config_t *config, uint32_t now)
{
    if (c == NULL || config == NULL || config->poll_ms == 0U || config->poll_ms >= 0x80000000U ||
        config->retry_ms == 0U || config->retry_ms >= 0x80000000U ||
        config->stale_ms == 0U || config->stale_ms >= 0x80000000U) return false;
    Modbus_RelayModule_t module;
    if (!Modbus_RelayModule_InitChannels(&module, config->slave_addr, config->channel, 1)) return false;
    memset(c, 0, sizeof(*c));
    c->config = *config;
    c->module = module;
    Modbus_RelayModule_GetChannelInfo(&c->module, config->channel, &c->status.channel_info);
    c->initialized = c->need_write = true;
    c->status.target_generation = 1;
    c->status.target_tick = c->completion_tick = now;
    c->delay_ms = config->poll_ms;
    return true;
}

void Modbus_RelayControl_SetTarget(Modbus_RelayControl_t *c, bool on, uint32_t now)
{
    if (c == NULL || !c->initialized || c->status.target_on == on) return;
    c->status.target_on = on;
    c->status.target_generation++;
    c->status.target_tick = now;
    c->status.verified = false;
    c->need_write = true;
}

uint16_t Modbus_RelayControl_PrepareRequest(Modbus_RelayControl_t *c, uint32_t now)
{
    if (c == NULL || !c->initialized || c->status.pending != MODBUS_RELAY_IDLE ||
        now - c->completion_tick < c->delay_ms) return 0;
    c->prepared_generation = c->status.target_generation;
    if (c->need_write && !c->read_next)
    {
        c->prepared = MODBUS_RELAY_WRITE;
        c->prepared_value = c->status.target_on;
        return Modbus_RelayModule_BuildWriteChannel(&c->module, c->config.channel, c->prepared_value);
    }
    c->prepared = MODBUS_RELAY_READ;
    return Modbus_BuildReadCoils(c->module.tx_buffer, c->module.slave_addr, c->module.start_addr, 1);
}

void Modbus_RelayControl_RequestSent(Modbus_RelayControl_t *c, uint32_t now)
{
    if (c == NULL || !c->initialized || c->status.pending != MODBUS_RELAY_IDLE || c->prepared == MODBUS_RELAY_IDLE)
        return;
    c->status.pending = c->prepared;
    c->prepared = MODBUS_RELAY_IDLE;
    c->pending_value = c->prepared_value;
    c->pending_generation = c->prepared_generation;
    c->module.state = MODBUS_STATE_WAITING_RESPONSE;
    if (c->status.pending == MODBUS_RELAY_WRITE)
    {
        c->status.write_count++;
        c->status.last_write_tick = now;
        c->sent_generation = c->pending_generation;
        c->status.verified = false;
    }
}

static void RelayControl_Finish(Modbus_RelayControl_t *c, uint32_t now, bool timeout)
{
    c->read_next = c->status.pending == MODBUS_RELAY_WRITE;
    c->status.pending = MODBUS_RELAY_IDLE;
    c->completion_tick = now;
    // 每次写入（即使应答丢失）之后必须先读回，不能盲目连续写入。
    c->delay_ms = timeout ? c->config.retry_ms : (c->read_next ? 3U : c->config.poll_ms);
}

bool Modbus_RelayControl_ProcessFrame(Modbus_RelayControl_t *c, const uint8_t *data,
    uint16_t length, bool overflow, uint32_t frame_tick, uint32_t now)
{
    if (c == NULL || !c->initialized || c->status.pending == MODBUS_RELAY_IDLE || data == NULL || length == 0)
        return false;
    if (!overflow && (data[0] != c->module.slave_addr ||
        (c->status.pending == MODBUS_RELAY_READ && length == 8U && memcmp(data, c->module.tx_buffer, 8) == 0)))
        return false;

    Modbus_ParseResult_e result;
    if (overflow)
    {
        result = MODBUS_PARSE_INVALID_LENGTH;
        c->module.state = MODBUS_STATE_ERROR;
        c->module.error_count++;
    }
    else if (c->status.pending == MODBUS_RELAY_WRITE)
        result = Modbus_RelayModule_ProcessWriteResponse(&c->module, data, length, c->config.channel, c->pending_value);
    else
        result = Modbus_RelayModule_ProcessResponse(&c->module, data, length, frame_tick);

    c->status.parse_result_valid = true;
    c->status.last_parse_result = result;
    if (result == MODBUS_PARSE_OK)
    {
        c->have_response = true;
        c->status.last_success_tick = frame_tick;
        if (c->status.pending == MODBUS_RELAY_WRITE)
            c->status.write_ack_count++;
        else
        {
            c->have_read = true;
            c->status.read_count++;
            c->status.last_read_tick = frame_tick;
            Modbus_RelayModule_ReadChannel(&c->module, c->config.channel, &c->status.observed_on);
            c->status.verified = c->pending_generation == c->status.target_generation &&
                c->sent_generation == c->status.target_generation && c->status.observed_on == c->status.target_on;
            c->need_write = !c->status.verified;
        }
    }
    else
    {
        c->status.parse_error_count++;
        if (result == MODBUS_PARSE_INVALID_CRC) c->status.crc_error_count++;
    }
    RelayControl_Finish(c, now, false);
    return true;
}

void Modbus_RelayControl_HandleTimeout(Modbus_RelayControl_t *c, uint32_t now)
{
    if (c == NULL || !c->initialized || c->status.pending == MODBUS_RELAY_IDLE) return;
    c->status.timeout_count++;
    Modbus_RelayModule_HandleTimeout(&c->module);
    RelayControl_Finish(c, now, true);
}

bool Modbus_RelayControl_GetStatus(const Modbus_RelayControl_t *c, uint32_t now,
                                  Modbus_RelayControl_Status_t *out)
{
    if (c == NULL || !c->initialized || out == NULL) return false;
    *out = c->status;
    out->online = c->have_response && now - c->status.last_success_tick <= c->config.stale_ms;
    out->sample_valid = c->have_read && now - c->status.last_read_tick <= c->config.stale_ms;
    out->verified = out->sample_valid && c->status.verified;
    return true;
}
