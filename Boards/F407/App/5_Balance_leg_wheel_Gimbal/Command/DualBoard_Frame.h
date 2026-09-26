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
X(CH0,            int16_t,  11, S) \
X(CH1,            int16_t,  11, S) \
X(CH2,            int16_t,  11, S) \
X(CH3,            int16_t,  11, S) \
X(Dial,           int16_t,  11, S) \
X(remoteOnLine,  uint8_t,   2, U) \
X(S1,            uint8_t,   2, U) \
X(S2,            uint8_t,   2, U) \


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
