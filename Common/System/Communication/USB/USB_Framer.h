#ifndef HORIZON_USB_FRAMER_H
#define HORIZON_USB_FRAMER_H

#include "USB_Router.h"

typedef struct {
    uint8_t data[USB_MAX_FRAME_SIZE];
    uint16_t used;
    bool discard_line;
    uint32_t last_rx_tick;
    uint32_t bad_frames, overflows, timeouts, parser_errors;
} USB_Framer_t;

typedef void (*USB_FrameEmit_t)(void *context, uint8_t *data, uint16_t len);

/* A packet/producer can arrive after a processing pass captured now_ms.
 * A slightly newer timestamp is not an elapsed interval across tick wrap. */
static inline bool USB_TickExpired(uint32_t now, uint32_t start, uint32_t timeout)
{
    uint32_t age = now - start;
    return timeout != 0U && age <= INT32_MAX && age >= timeout;
}

USB_Status_t USB_Framer_CheckConfig(const USB_RxConfig_t *config);
void USB_Framer_Reset(USB_Framer_t *framer, const USB_RxConfig_t *config);
void USB_Framer_Feed(USB_Framer_t *framer, const USB_RxConfig_t *config,
                     const uint8_t *data, uint16_t len, uint32_t rx_tick,
                     USB_FrameEmit_t emit, void *context);
bool USB_Framer_Expire(USB_Framer_t *framer, const USB_RxConfig_t *config,
                       uint32_t now_ms);

/* Built-in variable frame: A5 5A, version, command, payload_len LE16,
 * sequence LE16, payload, CRC16 LE16. CRC covers version through payload.
 * CRC: poly 0x1021, init 0xFFFF, no reflection, xorout 0. */
uint16_t USB_CRC16(const uint8_t *data, uint16_t len);
USB_ParseResult_t USB_DefaultFrameParser(const uint8_t *data, uint16_t available,
                                         void *context);
USB_Status_t USB_BuildFrame(uint8_t version, uint8_t command, uint16_t sequence,
                            const uint8_t *payload, uint16_t payload_len,
                            uint8_t *output, uint16_t capacity, uint16_t *written);

#endif
