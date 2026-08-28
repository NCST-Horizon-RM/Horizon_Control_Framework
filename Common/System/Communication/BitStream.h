//
// Created by admin on 2026/8/28.
//

#ifndef HORIZON_BITSTREAM_H
#define HORIZON_BITSTREAM_H

#include <stdint.h>
#include <string.h>

/* ============================================================
 *  底层 1：64 位快速路径（帧 <= 8 字节，整字操作，性能最高）
 * ============================================================ */

typedef struct { uint64_t w; uint8_t bitpos; } BitWriter_t;
typedef struct { uint64_t w; uint8_t bitpos; } BitReader_t;

static inline uint64_t bytes_to_u64(const uint8_t *b)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= (uint64_t)b[i] << (8 * i);
    return v;
}

static inline void u64_to_bytes(uint64_t v, uint8_t *b)
{
    for (int i = 0; i < 8; i++) b[i] = (uint8_t)(v >> (8 * i));
}

static inline void bw_put(BitWriter_t *w, uint32_t value, uint8_t width)
{
    uint32_t mask = (width >= 32) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    w->w |= (uint64_t)(value & mask) << w->bitpos;
    w->bitpos += width;
}

static inline uint32_t br_get(BitReader_t *r, uint8_t width)
{
    uint32_t mask = (width >= 32) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    uint32_t v = (uint32_t)((r->w >> r->bitpos) & mask);
    r->bitpos += width;
    return v;
}

static inline int32_t br_get_s(BitReader_t *r, uint8_t width)
{
    uint32_t v = br_get(r, width);
    if (width && (v & (1u << (width - 1)))) v |= ~0u << width;
    return (int32_t)v;
}

/* ============================================================
 *  底层 2：通用 buf 路径（帧任意字节，逐位循环，大小端无关）
 * ============================================================ */

typedef struct { uint8_t *buf;   uint8_t bitpos; } BitWriterBuf_t;
typedef struct { const uint8_t *buf; uint8_t bitpos; } BitReaderBuf_t;

static inline void bw_put_buf(BitWriterBuf_t *w, uint32_t value, uint8_t width)
{
    uint32_t mask = (width >= 32) ? 0xFFFFFFFFu : ((1u << width) - 1u);
    value &= mask;
    for (uint8_t i = 0; i < width; i++) {
        uint8_t b = w->bitpos + i;
        uint8_t m = (uint8_t)(1u << (b & 7));
        if (value & (1u << i)) w->buf[b >> 3] |=  m;
        else                   w->buf[b >> 3] &= (uint8_t)~m;
    }
    w->bitpos += width;
}

static inline uint32_t br_get_buf(BitReaderBuf_t *r, uint8_t width)
{
    uint32_t v = 0;
    for (uint8_t i = 0; i < width; i++) {
        uint8_t b = r->bitpos + i;
        if (r->buf[b >> 3] & (1u << (b & 7))) v |= (1u << i);
    }
    r->bitpos += width;
    return v;
}

static inline int32_t br_get_s_buf(BitReaderBuf_t *r, uint8_t width)
{
    uint32_t v = br_get_buf(r, width);
    if (width && (v & (1u << (width - 1)))) v |= ~0u << width;
    return (int32_t)v;
}

/* ============================================================
 *  代码生成宏
 * ============================================================ */

#define BS_STRUCT(n, t, w, s)       t n;

#define BS_PACK_64(n, t, w, s)      bw_put(&wr, (uint32_t)(src->n), w);
#define BS_PACK_BUF(n, t, w, s)     bw_put_buf(&wr, (uint32_t)(src->n), w);

#define BS_UNPACK_S_64(n, t, w, s)  dst->n = (t)br_get_s(&rd, w);
#define BS_UNPACK_U_64(n, t, w, s)  dst->n = (t)br_get(&rd, w);
#define BS_UNPACK_64(n, t, w, s)    BS_UNPACK_##s##_64(n, t, w, s)

#define BS_UNPACK_S_BUF(n, t, w, s) dst->n = (t)br_get_s_buf(&rd, w);
#define BS_UNPACK_U_BUF(n, t, w, s) dst->n = (t)br_get_buf(&rd, w);
#define BS_UNPACK_BUF(n, t, w, s)   BS_UNPACK_##s##_BUF(n, t, w, s)

#define BS_BITS(n, t, w, s)         + (w)

/* ---- 生成器（固定 8 字节输出）---- */
#define BS_FRAME_64(name, fields) \
    typedef struct { fields(BS_STRUCT) } name##_t; \
    static inline void name##_pack(const name##_t *src, uint8_t out[8]) \
    { \
        BitWriter_t wr = {0, 0}; \
        fields(BS_PACK_64) \
        u64_to_bytes(wr.w, out); \
    } \
    static inline void name##_unpack(const uint8_t in[8], name##_t *dst) \
    { \
        BitReader_t rd = { bytes_to_u64(in), 0 }; \
        fields(BS_UNPACK_64) \
    } \
    _Static_assert((0 fields(BS_BITS)) <= 64, #name " frame exceeds 64 bits")

/* ---- 生成器（可自定义 max_bytes，1~N 字节）---- */
#define BS_FRAME_BUF(name, fields, max_bytes) \
    typedef struct { fields(BS_STRUCT) } name##_t; \
    static inline void name##_pack(const name##_t *src, uint8_t *out) \
    { \
        BitWriterBuf_t wr = {out, 0}; \
        memset(out, 0, max_bytes); \
        fields(BS_PACK_BUF) \
    } \
    static inline void name##_unpack(const uint8_t *in, name##_t *dst) \
    { \
        BitReaderBuf_t rd = {in, 0}; \
        fields(BS_UNPACK_BUF) \
    } \
    _Static_assert((0 fields(BS_BITS)) <= ((max_bytes) * 8), \
                   #name " frame exceeds " #max_bytes " bytes")

#endif
