#include "Unitree_servo.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#define J288_PI 3.14159265358979323846

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* 显式恢复有符号数，避免无符号转有符号窄化的实现差异。 */
static int32_t get_i16(const uint8_t *p)
{
    uint16_t u = get_u16(p);
    return (u & 0x8000U) ? (int32_t)u - 65536 : (int32_t)u;
}

static int32_t get_i32(const uint8_t *p)
{
    uint32_t u = get_u32(p);
    return (u & 0x80000000U) ? -1 - (int32_t)(UINT32_MAX - u) : (int32_t)u;
}

static void put_u16(uint8_t *p, uint16_t u)
{
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
}

static void put_u32(uint8_t *p, uint32_t u)
{
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
    p[2] = (uint8_t)(u >> 16);
    p[3] = (uint8_t)(u >> 24);
}

static float clamp(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

static int16_t fixed16(double x)
{
    return x <= INT16_MIN ? INT16_MIN : (x >= INT16_MAX ? INT16_MAX : (int16_t)x);
}

static int32_t fixed32(double x)
{
    return x <= INT32_MIN ? INT32_MIN : (x >= INT32_MAX ? INT32_MAX : (int32_t)x);
}

uint32_t J288_CRC32(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length / 4U; ++i) {
        crc ^= get_u32(data + i * 4U);
        for (unsigned bit = 0; bit < 32U; ++bit) {
            crc = (crc & 0x80000000U) ? (crc << 1) ^ 0x04C11DB7U : crc << 1;
        }
    }
    return crc;
}

J288_Result_t J288_BuildPacket(const J288_Command_t *c, uint8_t *packet, size_t capacity)
{
    if (c == NULL || packet == NULL || c->id > J288_MAX_UNICAST_ID ||
        c->status > 1U || c->timeout_enable > 1U ||
        !isfinite(c->output_torque_nm) || !isfinite(c->output_speed_rad_s) ||
        !isfinite(c->output_position_rad) || !isfinite(c->kp) || !isfinite(c->kd)) {
        return J288_INVALID_ARGUMENT;
    }
    if (capacity < J288_TX_SIZE) return J288_LENGTH_ERROR;

    float tor = clamp(c->output_torque_nm, -36.9093f, 36.908174f);
    float spd = clamp(c->output_speed_rad_s, -278.90143f, 278.90143f);
    float pos = clamp(c->output_position_rad, -1428.018898f, 1428.018898f);
    float kp = clamp(c->kp, 0.0f, 2128.523254f);
    float kd = clamp(c->kd, 0.0f, 21.285233f);

    memset(packet, 0, J288_TX_SIZE);
    packet[0] = 0xFE;
    packet[1] = 0xEE;
    packet[2] = c->id | (uint8_t)(c->status << 4) | (uint8_t)(c->timeout_enable << 7);
    put_u16(packet + 4, (uint16_t)fixed16(tor / J288_RATIO * 256000.0f));
    put_u16(packet + 6, (uint16_t)fixed16(spd * J288_RATIO * 2.560f / J288_PI / 2.0f));
    put_u32(packet + 8, (uint32_t)fixed32(pos * J288_RATIO * 32768.0f / J288_PI / 2.0f));
    put_u16(packet + 12, (uint16_t)fixed16(kp / (J288_RATIO * J288_RATIO) * 1280000.0f));
    put_u16(packet + 14, (uint16_t)fixed16(kd / (J288_RATIO * J288_RATIO) * 128000000.0f));
    put_u32(packet + 16, J288_CRC32(packet, 16));
    return J288_OK;
}

J288_Result_t J288_ParseFeedback(const uint8_t *p, size_t length,
                               uint8_t expected_id, J288_Feedback_t *f)
{
    if (f == NULL) return J288_INVALID_ARGUMENT;
    memset(f, 0, sizeof(*f));
    if (p == NULL || expected_id > J288_MAX_UNICAST_ID) return J288_INVALID_ARGUMENT;
    if (length != J288_RX_SIZE) return J288_LENGTH_ERROR;
    f->id = p[2] & 0x0FU;
    f->status = (p[2] >> 4) & 7U;
    f->timeout = p[2] >> 7;
    f->crc_received = get_u32(p + 22);
    f->crc_calculated = J288_CRC32(p + 2, 20);
    if (p[0] != 0xFC || p[1] != 0xEE) return J288_HEADER_ERROR;
    if (f->crc_received != f->crc_calculated) return J288_CRC_ERROR;
    if (f->id != expected_id) return J288_ID_MISMATCH;

    f->temperature_c = (int8_t)(p[3] < 128U ? (int)p[3] : (int)p[3] - 256);
    f->winding_temperature_c = p[4];
    f->voltage_v = (float)p[5] / 2.0f;
    f->output_torque_nm = (float)get_i16(p + 6) / 256000.0f * J288_RATIO;
    f->output_speed_rad_s = ((float)get_i16(p + 8) / 2.56f) * 2 * J288_PI / J288_RATIO;
    f->output_position_rad = 2 * J288_PI * (float)get_i32(p + 10) / 32768.0f / J288_RATIO;
    f->merror = get_u32(p + 14);
    uint16_t extra = get_u16(p + 18);
    f->outpos = extra & 0x1FFFU;
    f->exflag = (uint8_t)(extra >> 13);
    f->exsensor2 = p[20];
    f->excom = p[21];
    f->external_position_rad = 2 * J288_PI * (float)f->outpos / 8192.0f;
    f->valid = 1U;
    return J288_OK;
}
