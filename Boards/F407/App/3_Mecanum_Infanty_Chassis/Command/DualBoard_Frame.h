#ifndef HORIZON_DUALBOARD_FRAME_H
#define HORIZON_DUALBOARD_FRAME_H

#include "BitStream.h"

/* ============================================================
 *  双板通信协议：底盘 <-> 云台 的 CAN 帧定义
 * ============================================================ */

/* ============================================================
 *  帧 0x231：云台 -> 底盘（≤ 64 bit，用 64 位快速路径）
 * ============================================================ */
#define DUALBOARD_G2C_FIELDS(X) \
    X(vx,            int16_t,  11, S) \
    X(vy,            int16_t,  11, S) \
    X(vr,            int16_t,  11, S) \
    X(key_q,         uint8_t,   1, U) \
    X(key_e,         uint8_t,   1, U) \
    X(key_v,         uint8_t,   1, U) \
    X(key_shift,     uint8_t,   1, U) \
    X(key_ctrl,      uint8_t,   1, U) \
    X(romoteOnLine,  uint8_t,   2, U) \
    X(S1,            uint8_t,   2, U) \
    X(S2,            uint8_t,   2, U) \
    X(pitch,         int8_t,    5, S) \
    X(fire_wheel,    uint8_t,   1, U) \
    X(gimbal_lixian, uint8_t,   1, U) \
    X(vision_look,   uint8_t,   1, U) \
    X(vision,        uint8_t,   1, U) \
    X(surplus_count, uint16_t,  9, U)

/* ============================================================
 *  帧 0x232：底盘 -> 云台（≤ 64 bit，用 64 位快速路径）
 * ============================================================ */
#define DUALBOARD_C2G_FIELDS(X) \
    X(heat_last,  uint16_t, 10, U) \
    X(self_color, uint8_t,   1, U) \
    X(cooling,    uint8_t,   7, U) \
    X(level,      uint8_t,   4, U) \
    X(initial_s,  uint8_t,   8, U) \
    X(robot_HP,   uint16_t,  9, U) \
    X(heat_large, uint16_t,  9, U)

/* ---- 自动生成结构体 + pack/unpack + 编译期位宽检查 ---- */
BS_FRAME_64(G2C, DUALBOARD_G2C_FIELDS);
BS_FRAME_64(C2G, DUALBOARD_C2G_FIELDS);

#endif
