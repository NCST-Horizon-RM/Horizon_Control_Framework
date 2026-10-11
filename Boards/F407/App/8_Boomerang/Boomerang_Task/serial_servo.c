//
// Created by lemon on 2026/10/11.
//

#include "serial_servo.h"

void ServoMove(uint8_t id, uint16_t angle, uint16_t time_ms) {
    // 数据包构造
    uint8_t packet[10] = {
        FRAME_HEADER_1,
        FRAME_HEADER_2,
        0x08,                   // Length = 6参数 + 2（Cmd+Length） = 8
        CMD_SERVO_MOVE,
        0x01,                   // 舵机数量（固定为1）
        (uint8_t)(time_ms & 0xFF),      // 时间低字节
        (uint8_t)(time_ms >> 8),        // 时间高字节
        id,                     // 舵机ID
        (uint8_t)(angle & 0xFF),        // 角度低字节
        (uint8_t)(angle >> 8)           // 角度高字节
    };

    // 发送数据包
    HAL_UART_Transmit_DMA(&huart6, packet, sizeof(packet));
}

// 控制多个舵机（ID数组、角度数组、时间）
void ServoMoveMulti(uint8_t num_servos, uint8_t *ids, uint16_t *angles, uint16_t time_ms) {
    // 参数构造
    uint8_t params[1 + 2 + 3 * num_servos]; // 1+2+3N
    params[0] = num_servos;                 // 舵机数量
    params[1] = (uint8_t)(time_ms & 0xFF);  // 时间低字节
    params[2] = (uint8_t)(time_ms >> 8);    // 时间高字节

    // 填充每个舵机的ID和角度
    for (int i = 0; i < num_servos; i++) {
        params[3 + 3*i] = ids[i];           // ID
        params[4 + 3*i] = angles[i] & 0xFF; // 角度低字节
        params[5 + 3*i] = angles[i] >> 8;   // 角度高字节
    }

    // 构造完整数据包
    uint8_t packet[2 + 1 + 1 + sizeof(params)]; // 帧头(2) + Length(1) + Cmd(1) + 参数
    packet[0] = 0x55;
    packet[1] = 0x55;
    packet[2] = sizeof(params) + 2;         // Length = 参数长度 + 2
    packet[3] = 0x03;                       // CMD_SERVO_MOVE
    memcpy(&packet[4], params, sizeof(params));

    // 发送数据包
    HAL_UART_Transmit(&huart6, packet, sizeof(packet), 1000);
}