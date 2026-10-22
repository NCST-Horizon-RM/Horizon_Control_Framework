//
// Created by lemon on 2026/10/11.
//

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <usart.h>

#ifndef HORIZON_CONTROL_FRAMEWORK_SERIAL_SERVO_H
#define HORIZON_CONTROL_FRAMEWORK_SERIAL_SERVO_H

#define FRAME_HEADER_1      0x55
#define FRAME_HEADER_2      0x55
#define CMD_SERVO_MOVE      0x03
#define CMD_GET_BATTERY     0x0F
#define RX_BUF_SIZE 32

void ServoMove(uint8_t id, uint16_t angle, uint16_t time_ms);
void ServoMoveMulti(uint8_t num_servos, uint8_t *ids, uint16_t *angles, uint16_t time_ms);

#endif //HORIZON_CONTROL_FRAMEWORK_SERIAL_SERVO_H
