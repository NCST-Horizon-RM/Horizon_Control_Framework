/**
 * @file Modbus_Device.h
 * @brief Modbus 设备类型和状态定义
 * @date 2026-09-06
 */

#ifndef MODBUS_DEVICE_H
#define MODBUS_DEVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "Modbus_RTU.h"

/* 支持的最大通道数 */
#define MODBUS_MAX_DISCRETE_CHANNELS    48
#define MODBUS_MAX_COIL_CHANNELS        48
#define MODBUS_MAX_INPUT_REGISTERS      16
#define MODBUS_MAX_HOLDING_REGISTERS    16

/**
 * @brief Modbus 设备状态
 */
typedef enum {
    MODBUS_STATE_IDLE = 0,              // 空闲
    MODBUS_STATE_WAITING_RESPONSE,      // 等待响应
    MODBUS_STATE_RESPONSE_OK,           // 响应正常
    MODBUS_STATE_TIMEOUT,               // 超时
    MODBUS_STATE_ERROR                  // 错误
} Modbus_State_e;

/**
 * @brief 数字输入模块（离散输入）
 * 使用 Modbus 功能码 0x02 读取离散输入状态
 */
typedef struct {
    uint8_t slave_addr;                 // 从机地址 (1-247)
    uint16_t start_addr;                // 起始地址（通常为 0x0000）
    uint16_t input_count;               // 离散输入数量 (1-48)

    uint8_t input_data[6];              // 位打包的输入状态（最多 48 位 = 6 字节）
    Modbus_State_e state;               // 当前状态
    uint32_t last_update_time;          // 最后成功更新时间戳（毫秒）
    uint32_t error_count;               // 错误计数器

    uint8_t tx_buffer[8];               // 请求发送缓冲区
    uint8_t rx_buffer[32];              // 响应接收缓冲区
    uint16_t rx_length;                 // 接收数据长度
} Modbus_DigitalInput_t;

/** X1-X48 输入位号在当前查询中的映射；适用于 Xn -> 地址 n-1 的模块。 */
typedef struct {
    uint16_t channel;          // 面板输入位号，1 表示 X1
    uint16_t modbus_address;   // Modbus 零基地址
    uint8_t response_bit;      // 相对本次查询起点的位索引，供 GetBit 使用
    uint8_t byte_index;        // input_data[] 中的字节索引（不含响应帧头）
    uint8_t bit_index;         // 该字节内的位索引，0-7
} Modbus_DigitalInput_ChannelInfo_t;

// 继电器 Yn 使用同样的 n-1 地址规则；response_bit 相对于当前线圈查询起点。
typedef Modbus_DigitalInput_ChannelInfo_t Modbus_RelayChannelInfo_t;

/**
 * @brief 继电器模块（线圈）
 * 使用 Modbus 功能码 0x01 读取线圈状态
 */
typedef struct {
    uint8_t slave_addr;                 // 从机地址 (1-247)
    uint16_t start_addr;                // 起始地址（通常为 0x0000）
    uint16_t coil_count;                // 线圈数量 (1-48)

    uint8_t coil_data[6];               // 位打包的线圈状态（最多 48 位 = 6 字节）
    Modbus_State_e state;               // 当前状态
    uint32_t last_update_time;          // 最后成功更新时间戳（毫秒）
    uint32_t error_count;               // 错误计数器

    uint8_t tx_buffer[8];               // 请求发送缓冲区
    uint8_t rx_buffer[32];              // 响应接收缓冲区
    uint16_t rx_length;                 // 接收数据长度
} Modbus_RelayModule_t;

/* 可复用继电器控制器：配置、运行状态及只读快照。 */
typedef enum { MODBUS_RELAY_IDLE, MODBUS_RELAY_READ, MODBUS_RELAY_WRITE } Modbus_RelayOperation_e;
typedef struct {
    uint8_t slave_addr;
    uint16_t channel;
    uint32_t poll_ms, retry_ms, stale_ms;
} Modbus_RelayControl_Config_t;

typedef struct {
    Modbus_RelayChannelInfo_t channel_info;
    bool online;                // 最近有效读响应或写应答仍新鲜
    bool sample_valid;          // 最近 FC01 读回仍新鲜
    bool target_on;
    bool observed_on;           // 只由 FC01 更新
    bool verified;              // 当前目标已发送，并由之后的 FC01 确认
    uint32_t target_generation;
    uint32_t target_tick;
    uint32_t last_success_tick;
    uint32_t last_read_tick;
    uint32_t last_write_tick;
    uint32_t read_count;
    uint32_t write_count;
    uint32_t write_ack_count;
    uint32_t timeout_count;
    uint32_t parse_error_count;
    uint32_t crc_error_count;
    bool parse_result_valid;
    Modbus_ParseResult_e last_parse_result;
    Modbus_RelayOperation_e pending;
} Modbus_RelayControl_Status_t;

// 单任务拥有；不依赖 BSP/HAL。通过下面的接口驱动，不直接修改内部字段。
typedef struct {
    Modbus_RelayModule_t module;
    Modbus_RelayControl_Config_t config;
    Modbus_RelayControl_Status_t status;
    bool initialized, have_response, have_read, need_write, read_next;
    bool prepared_value, pending_value;
    uint32_t sent_generation, prepared_generation, pending_generation, completion_tick, delay_ms;
    Modbus_RelayOperation_e prepared;
} Modbus_RelayControl_t;

/**
 * @brief 初始化数字输入模块
 * @param module: 模块实例指针
 * @param slave_addr: 从机地址 (1-247)
 * @param start_addr: 起始地址（通常为 0x0000）
 * @param input_count: 要读取的离散输入数量 (1-48)
 */
void Modbus_DigitalInput_Init(Modbus_DigitalInput_t *module,
                               uint8_t slave_addr,
                               uint16_t start_addr,
                               uint16_t input_count);

/**
 * @brief 按面板位号初始化连续输入范围，Xn 映射到地址 n-1。
 * @param first_channel 首个输入位号（1-48）；例如 16 表示 X16。
 * @param input_count 连续读取数量，整个范围必须落在 X1-X48 内。
 * @return 配置成功返回 true；参数无效返回 false，module 保持不变。
 * 仅配置设备，不操作 UART，也不发送请求。其他地址映射仍使用原 Init 接口。
 */
bool Modbus_DigitalInput_InitChannels(Modbus_DigitalInput_t *module,
                                     uint8_t slave_addr,
                                     uint16_t first_channel,
                                     uint16_t input_count);

/**
 * @brief 获取面板位号对应的协议地址、响应位和字节位置，不依赖通信状态。
 * @return 位号不在当前查询范围、配置无效或空指针时返回 false，out 不变。
 */
bool Modbus_DigitalInput_GetChannelInfo(const Modbus_DigitalInput_t *module,
                                       uint16_t channel,
                                       Modbus_DigitalInput_ChannelInfo_t *out);

/**
 * @brief 按面板位号读取 input_data 中的缓存值，不发起通信、不判断数据时效。
 * @return true 仅表示位号有效，电平通过 value 输出；false 时 value 不变。
 * 首次有效响应前缓存为 0；出错/失联后保留旧值。调用方负责检查有效性和在线状态。
 */
bool Modbus_DigitalInput_ReadChannel(const Modbus_DigitalInput_t *module,
                                    uint16_t channel, bool *value);

/**
 * @brief 初始化继电器模块
 * @param module: 模块实例指针
 * @param slave_addr: 从机地址 (1-247)
 * @param start_addr: 起始地址（通常为 0x0000）
 * @param coil_count: 要读取的线圈数量 (1-48)
 */
void Modbus_RelayModule_Init(Modbus_RelayModule_t *module,
                              uint8_t slave_addr,
                              uint16_t start_addr,
                              uint16_t coil_count);

bool Modbus_RelayModule_InitChannels(Modbus_RelayModule_t *module, uint8_t slave_addr,
                                     uint16_t first_channel, uint16_t count);
bool Modbus_RelayModule_GetChannelInfo(const Modbus_RelayModule_t *module,
                                      uint16_t channel, Modbus_RelayChannelInfo_t *out);
// 读取 FC01 缓存，不发起通信；调用方判断时效，false 时输出不变。
bool Modbus_RelayModule_ReadChannel(const Modbus_RelayModule_t *module, uint16_t channel, bool *value);
// 在 module->tx_buffer 中准备 FC05 请求，不修改线圈读回值。
uint16_t Modbus_RelayModule_BuildWriteChannel(Modbus_RelayModule_t *module, uint16_t channel, bool on);
Modbus_ParseResult_e Modbus_RelayModule_ProcessWriteResponse(Modbus_RelayModule_t *module,
    const uint8_t *response, uint16_t length, uint16_t channel, bool on);

/**
 * @brief 构建数字输入模块的查询请求
 * @param module: 模块实例指针
 * @return 请求长度（错误时返回 0）
 */
uint16_t Modbus_DigitalInput_BuildQuery(Modbus_DigitalInput_t *module);

/**
 * @brief 构建继电器模块的查询请求
 * @param module: 模块实例指针
 * @return 请求长度（错误时返回 0）
 */
uint16_t Modbus_RelayModule_BuildQuery(Modbus_RelayModule_t *module);

/**
 * @brief 处理数字输入模块的响应
 * @param module: 模块实例指针
 * @param response: 接收到的响应缓冲区
 * @param response_len: 响应长度
 * @param current_time: 当前时间戳（毫秒）
 * @return 解析结果
 */
Modbus_ParseResult_e Modbus_DigitalInput_ProcessResponse(Modbus_DigitalInput_t *module,
                                                          const uint8_t *response,
                                                          uint16_t response_len,
                                                          uint32_t current_time);

/**
 * @brief 处理继电器模块的响应
 * @param module: 模块实例指针
 * @param response: 接收到的响应缓冲区
 * @param response_len: 响应长度
 * @param current_time: 当前时间戳（毫秒）
 * @return 解析结果
 */
Modbus_ParseResult_e Modbus_RelayModule_ProcessResponse(Modbus_RelayModule_t *module,
                                                         const uint8_t *response,
                                                         uint16_t response_len,
                                                         uint32_t current_time);

/**
 * @brief 获取特定输入位的状态
 * @param module: 模块实例指针
 * @param index: 输入索引（从 0 开始）
 * @return 输入为高电平返回 true，低电平或无效索引返回 false
 */
bool Modbus_DigitalInput_GetBit(const Modbus_DigitalInput_t *module, uint16_t index);

/**
 * @brief 获取特定线圈位的状态
 * @param module: 模块实例指针
 * @param index: 线圈索引（从 0 开始）
 * @return 线圈为开返回 true，关闭或无效索引返回 false
 */
bool Modbus_RelayModule_GetBit(const Modbus_RelayModule_t *module, uint16_t index);

/**
 * @brief 处理数字输入模块的超时
 * @param module: 模块实例指针
 */
void Modbus_DigitalInput_HandleTimeout(Modbus_DigitalInput_t *module);

/**
 * @brief 处理继电器模块的超时
 * @param module: 模块实例指针
 */
void Modbus_RelayModule_HandleTimeout(Modbus_RelayModule_t *module);

// 初始目标为关闭；首次事务写入当前目标，之后读回验证。
bool Modbus_RelayControl_Init(Modbus_RelayControl_t *control,
                              const Modbus_RelayControl_Config_t *config, uint32_t now);
void Modbus_RelayControl_SetTarget(Modbus_RelayControl_t *control, bool on, uint32_t now);
// 到期时返回 module.tx_buffer 内请求的长度。传输层 BUSY 时可在下一轮重新调用。
uint16_t Modbus_RelayControl_PrepareRequest(Modbus_RelayControl_t *control, uint32_t now);
// 仅在传输层成功接受请求后调用；发送失败不调用，不能提前把控制状态视为已发送。
void Modbus_RelayControl_RequestSent(Modbus_RelayControl_t *control, uint32_t now);
// 由传输层完成分帧/过滤超时后调用。返回 true 表示结束本次事务。
bool Modbus_RelayControl_ProcessFrame(Modbus_RelayControl_t *control, const uint8_t *data,
    uint16_t length, bool overflow, uint32_t frame_tick, uint32_t now);
void Modbus_RelayControl_HandleTimeout(Modbus_RelayControl_t *control, uint32_t now);
bool Modbus_RelayControl_GetStatus(const Modbus_RelayControl_t *control, uint32_t now,
                                  Modbus_RelayControl_Status_t *out);

#endif // MODBUS_DEVICE_H
