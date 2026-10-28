#include "USB_Framer.h"
#include <string.h>

static uint16_t CacheSize(const USB_RxConfig_t *c)
{
    return c->cache_size ? c->cache_size : USB_MAX_FRAME_SIZE;
}

static uint16_t FrameLimit(const USB_RxConfig_t *c)
{
    return c->max_frame_len ? c->max_frame_len : CacheSize(c);
}

USB_Status_t USB_Framer_CheckConfig(const USB_RxConfig_t *c)
{
    if (c == NULL || (unsigned)c->mode > USB_RX_CUSTOM || CacheSize(c) == 0U ||
        CacheSize(c) > USB_MAX_FRAME_SIZE || FrameLimit(c) > CacheSize(c)) {
        return USB_ERR_CONFIG;
    }
    if (c->mode == USB_RX_RAW && CacheSize(c) < USB_RX_PACKET_SIZE) {
        return USB_ERR_CONFIG;
    }
    if (c->mode == USB_RX_FIXED &&
        (c->frame_len == 0U || c->frame_len > FrameLimit(c))) {
        return USB_ERR_CONFIG;
    }
    if (c->mode == USB_RX_DELIMITER &&
        (c->delimiter == NULL || c->delimiter_len == 0U ||
         c->delimiter_len > USB_MAX_DELIMITER_SIZE ||
         c->delimiter_len > CacheSize(c))) {
        return USB_ERR_CONFIG;
    }
    if (c->mode == USB_RX_FRAMED && FrameLimit(c) < 10U) {
        return USB_ERR_CONFIG;
    }
    if (c->mode == USB_RX_CUSTOM && c->custom_parser == NULL) {
        return USB_ERR_CONFIG;
    }
    /* A callback always receives a complete frame/chunk, never a truncated one.
     * FIXED needs one frame; RAW needs one native packet; other modes need the
     * configured maximum output length. Assembly storage remains private. */
    uint16_t required = c->mode == USB_RX_FIXED ? c->frame_len :
        (c->mode == USB_RX_RAW ? USB_RX_PACKET_SIZE : FrameLimit(c));
    if ((c->rx_buf == NULL && c->rx_buf_size != 0U) ||
        (c->rx_buf != NULL && c->rx_buf_size < required)) {
        return USB_ERR_CONFIG;
    }
    /* Unsigned wrap-safe deadlines require intervals below half the tick range. */
    if (c->partial_timeout_ms > INT32_MAX || c->offline_timeout_ms > INT32_MAX) {
        return USB_ERR_CONFIG;
    }
    return USB_OK;
}

void USB_Framer_Reset(USB_Framer_t *f, const USB_RxConfig_t *c)
{
    f->used = 0U;
    f->discard_line = false;
    if (c->parser_reset != NULL) {
        c->parser_reset(c->parser_context);
    }
}

bool USB_Framer_Expire(USB_Framer_t *f, const USB_RxConfig_t *c, uint32_t now)
{
    if ((f->used != 0U || f->discard_line) &&
        USB_TickExpired(now, f->last_rx_tick, c->partial_timeout_ms)) {
        USB_Framer_Reset(f, c);
        ++f->timeouts;
        return true;
    }
    return false;
}

static void Consume(USB_Framer_t *f, uint16_t count)
{
    f->used = (uint16_t)(f->used - count);
    memmove(f->data, f->data + count, f->used);
}

static bool EndsWithDelimiter(const USB_Framer_t *f, const USB_RxConfig_t *c)
{
    return f->used >= c->delimiter_len &&
        memcmp(f->data + f->used - c->delimiter_len,
               c->delimiter, c->delimiter_len) == 0;
}

static void ParseAvailable(USB_Framer_t *f, const USB_RxConfig_t *c,
                           USB_FrameEmit_t emit, void *context)
{
    while (f->used != 0U) {
        USB_ParseResult_t r = {USB_PARSE_WAIT, 0U, 0U, 0U};
        if (c->mode == USB_RX_FIXED) {
            if (f->used >= c->frame_len) {
                r = (USB_ParseResult_t){USB_PARSE_FRAME, c->frame_len, 0U, c->frame_len};
            }
        } else if (c->mode == USB_RX_DELIMITER) {
            if (EndsWithDelimiter(f, c)) {
                uint16_t len = c->include_delimiter ? f->used :
                    (uint16_t)(f->used - c->delimiter_len);
                r = (USB_ParseResult_t){USB_PARSE_FRAME, f->used, 0U, len};
            }
        } else {
            if (c->mode == USB_RX_FRAMED && f->used >= 6U &&
                f->data[0] == 0xA5U && f->data[1] == 0x5AU &&
                (uint32_t)f->data[4] + ((uint32_t)f->data[5] << 8U) + 10U > FrameLimit(c)) {
                ++f->bad_frames;
                Consume(f, 1U);
                continue;
            }
            USB_FrameParser_t parser = c->mode == USB_RX_FRAMED ?
                USB_DefaultFrameParser : c->custom_parser;
            r = parser(f->data, f->used, c->parser_context);
        }

        if (r.status == USB_PARSE_WAIT) {
            if (r.consumed != 0U || r.frame_offset != 0U || r.frame_len != 0U) {
                ++f->parser_errors;
                Consume(f, 1U);
                continue;
            }
            break;
        }
        if (c->mode == USB_RX_DELIMITER && r.status == USB_PARSE_FRAME &&
            r.frame_len > FrameLimit(c)) {
            ++f->overflows;
            Consume(f, r.consumed);
            continue;
        }
        if ((r.status != USB_PARSE_FRAME && r.status != USB_PARSE_DROP) ||
            r.consumed == 0U || r.consumed > f->used ||
            (r.status == USB_PARSE_FRAME &&
             ((uint32_t)r.frame_offset + r.frame_len > r.consumed ||
              r.frame_len > FrameLimit(c)))) {
            ++f->parser_errors;
            /* Invalid custom parser output must never spin or overrun memory. */
            Consume(f, 1U);
            continue;
        }
        if (r.status == USB_PARSE_DROP) {
            ++f->bad_frames;
            Consume(f, r.consumed);
            continue;
        }
        uint8_t *frame = f->data + r.frame_offset;
        if (c->check_frame != NULL && !c->check_frame(frame, r.frame_len)) {
            ++f->bad_frames;
            Consume(f, c->mode == USB_RX_DELIMITER ? r.consumed : 1U);
            continue;
        }
        emit(context, frame, r.frame_len);
        Consume(f, r.consumed);
    }
}

void USB_Framer_Feed(USB_Framer_t *f, const USB_RxConfig_t *c,
                     const uint8_t *data, uint16_t len, uint32_t tick,
                     USB_FrameEmit_t emit, void *context)
{
    (void)USB_Framer_Expire(f, c, tick);
    f->last_rx_tick = tick;
    if (c->mode == USB_RX_RAW) {
        /* Each subscriber receives its own writable copy. */
        if (len > CacheSize(c)) {
            ++f->overflows;
            return;
        }
        memcpy(f->data, data, len);
        emit(context, f->data, len);
        return;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        if (f->discard_line) {
            if (f->used == c->delimiter_len) {
                Consume(f, 1U);
            }
            f->data[f->used++] = data[i];
            if (EndsWithDelimiter(f, c)) {
                f->used = 0U;
                f->discard_line = false;
            }
            continue;
        }
        if (f->used == CacheSize(c)) {
            ++f->overflows;
            if (c->mode == USB_RX_DELIMITER) {
                /* Keep the suffix so a split multi-byte delimiter still matches. */
                uint16_t keep = (uint16_t)(c->delimiter_len - 1U);
                memmove(f->data, f->data + f->used - keep, keep);
                f->used = keep;
                f->discard_line = true;
                --i; /* Process this byte through the discard path. */
                continue;
            }
            Consume(f, 1U);
        }
        f->data[f->used++] = data[i];
        ParseAvailable(f, c, emit, context);
    }
}

uint16_t USB_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U) :
                (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

USB_ParseResult_t USB_DefaultFrameParser(const uint8_t *data, uint16_t len,
                                         void *context)
{
    (void)context;
    USB_ParseResult_t wait = {USB_PARSE_WAIT, 0U, 0U, 0U};
    USB_ParseResult_t drop = {USB_PARSE_DROP, 1U, 0U, 0U};
    if (len == 0U) return wait;
    if (data[0] != 0xA5U) return drop;
    if (len < 2U) return wait;
    if (data[1] != 0x5AU) return drop;
    if (len < 8U) return wait;
    uint32_t total = 10U + data[4] + ((uint32_t)data[5] << 8U);
    if (total > USB_MAX_FRAME_SIZE) return drop;
    if (len < total) return wait;
    uint16_t wire_crc = (uint16_t)data[total - 2U] |
        ((uint16_t)data[total - 1U] << 8U);
    if (wire_crc != USB_CRC16(data + 2U, (uint16_t)(total - 4U))) return drop;
    return (USB_ParseResult_t){USB_PARSE_FRAME, (uint16_t)total, 0U, (uint16_t)total};
}

USB_Status_t USB_BuildFrame(uint8_t version, uint8_t command, uint16_t sequence,
                            const uint8_t *payload, uint16_t len,
                            uint8_t *output, uint16_t capacity, uint16_t *written)
{
    if (output == NULL || written == NULL || (payload == NULL && len != 0U)) {
        return USB_ERR_ARG;
    }
    *written = 0U;
    uint32_t total = (uint32_t)len + 10U;
    if (total > capacity || total > USB_MAX_FRAME_SIZE) return USB_ERR_SIZE;
    /* Support a payload already located inside the output buffer. */
    if (len != 0U) memmove(output + 8U, payload, len);
    output[0] = 0xA5U;
    output[1] = 0x5AU;
    output[2] = version;
    output[3] = command;
    output[4] = (uint8_t)len;
    output[5] = (uint8_t)(len >> 8U);
    output[6] = (uint8_t)sequence;
    output[7] = (uint8_t)(sequence >> 8U);
    uint16_t crc = USB_CRC16(output + 2U, (uint16_t)(total - 4U));
    output[total - 2U] = (uint8_t)crc;
    output[total - 1U] = (uint8_t)(crc >> 8U);
    *written = (uint16_t)total;
    return USB_OK;
}
