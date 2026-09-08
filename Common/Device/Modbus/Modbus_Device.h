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

#endif // MODBUS_DEVICE_H
