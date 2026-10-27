#ifndef HORIZON_USB_ROUTER_H
#define HORIZON_USB_ROUTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Static memory limits. Override consistently for every target if required. */
#ifndef USB_MAX_RX_NODES
#define USB_MAX_RX_NODES 4U
#endif
#ifndef USB_MAX_FRAME_SIZE
#define USB_MAX_FRAME_SIZE 512U
#endif
#ifndef USB_RX_PACKET_SIZE
#define USB_RX_PACKET_SIZE 64U
#endif
#ifndef USB_RX_QUEUE_DEPTH
#define USB_RX_QUEUE_DEPTH 16U
#endif
#ifndef USB_TX_MAX_SIZE
#define USB_TX_MAX_SIZE 512U
#endif
#ifndef USB_TX_QUEUE_DEPTH
#define USB_TX_QUEUE_DEPTH 4U
#endif
#ifndef USB_EVENT_QUEUE_DEPTH
#define USB_EVENT_QUEUE_DEPTH 16U
#endif
#ifndef USB_SERVICE_RX_BUDGET
#define USB_SERVICE_RX_BUDGET 512U
#endif
#define USB_MAX_DELIMITER_SIZE 8U

typedef enum { USB_PORT_CDC1 = 0, USB_PORT_COUNT } USB_Port_t;
typedef enum {
    USB_OK = 0, USB_ERR_ARG, USB_ERR_SIZE, USB_ERR_FULL,
    USB_ERR_NOT_READY, USB_ERR_BUSY, USB_ERR_TIMEOUT, USB_ERR_CONFIG
} USB_Status_t;
typedef enum {
    USB_RX_RAW = 0, USB_RX_FIXED, USB_RX_DELIMITER,
    USB_RX_FRAMED, USB_RX_CUSTOM
} USB_RxMode_t;
typedef enum { USB_OVERFLOW_DROP_NEW = 0, USB_OVERFLOW_PAUSE } USB_OverflowPolicy_t;
typedef enum { USB_RX_REARM = 0, USB_RX_PAUSE } USB_RxAction_t;
typedef enum { USB_PARSE_WAIT = 0, USB_PARSE_FRAME, USB_PARSE_DROP } USB_ParseStatus_t;

typedef struct {
    USB_ParseStatus_t status;
    uint16_t consumed;
    uint16_t frame_offset;
    uint16_t frame_len;
} USB_ParseResult_t;

typedef enum {
    USB_EVENT_CONNECTED = 0, USB_EVENT_DISCONNECTED,
    USB_EVENT_SUSPENDED, USB_EVENT_RESUMED, USB_EVENT_CONTROL,
    USB_EVENT_RX_GAP, USB_EVENT_RX_OVERFLOW, USB_EVENT_RX_TIMEOUT,
    USB_EVENT_BAD_FRAME, USB_EVENT_PARSER_ERROR,
    USB_EVENT_TX_DONE, USB_EVENT_TX_TIMEOUT, USB_EVENT_TX_CANCELLED,
    USB_EVENT_ONLINE, USB_EVENT_OFFLINE, USB_EVENT_BREAK
} USB_EventCode_t;

typedef struct {
    USB_EventCode_t code;
    uint32_t ticket;       /* Zero for events without a TX ticket. */
    uint32_t detail;
    uint32_t tick_ms;
} USB_Event_t;

/* Same argument order and length type as BSP_UART_Callback_t. */
typedef void (*USB_RxCallback_t)(uint8_t *data, void *device_ptr, uint16_t size);
typedef bool (*USB_FrameCheck_t)(const uint8_t *data, uint16_t size);
typedef USB_ParseResult_t (*USB_FrameParser_t)(const uint8_t *data,
                                              uint16_t available, void *context);
typedef void (*USB_ParserReset_t)(void *context);
typedef void (*USB_EventCallback_t)(void *device_ptr, const USB_Event_t *event);

typedef struct {
    USB_RxMode_t mode;
    uint16_t frame_len;
    uint16_t max_frame_len;       /* Zero: use cache_size/default limit. */
    uint16_t cache_size;          /* Zero: use USB_MAX_FRAME_SIZE. */
    uint32_t partial_timeout_ms;  /* Zero disables the incomplete-frame timer. */
    uint32_t offline_timeout_ms;  /* Zero disables valid-frame expiry. */
    const uint8_t *delimiter;
    uint8_t delimiter_len;
    bool include_delimiter;
    USB_FrameCheck_t check_frame; /* NULL: no additional validation. */
    USB_FrameParser_t custom_parser;
    USB_ParserReset_t parser_reset;
    void *parser_context;        /* Private mutable state for this node. */
    USB_EventCallback_t event_callback;
    uint8_t *rx_buf;             /* Optional application-owned callback buffer. */
    uint16_t rx_buf_size;        /* Actual capacity; zero only when rx_buf is NULL. */
} USB_RxConfig_t;

typedef struct {
    USB_Port_t port;
    const USB_RxConfig_t *config;
    void *device_ptr;            /* The application chooses the pointed-to type. */
    USB_RxCallback_t resolve;
} Auto_USB_Reg_t;

#define USB_JOIN_IMPL(a, b) a##b
#define USB_JOIN(a, b) USB_JOIN_IMPL(a, b)
/* Nine arguments keep the internal callback buffer. Eleven arguments insert
 * rx_buf and its capacity between check_frame and device_ptr, like UART.
 * The macro owns both static registration/configuration objects.
 * Use EX for delimiter/custom parsing, events, or an explicit maximum length.
 * Declare each node on its own source line (as with UART_RX_NODE/CAN_RX_NODE). */
#define USB_RX_NODE_SELECT(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, NAME, ...) NAME
#define USB_RX_NODE(...) \
    USB_RX_NODE_SELECT(__VA_ARGS__, USB_RX_NODE_11, USB_RX_NODE_INVALID_ARGUMENT_COUNT, \
                       USB_RX_NODE_9, unused)(__VA_ARGS__)

#define USB_RX_NODE_9(port_arg, mode_arg, frame_len_arg, cache_size_arg, \
                    partial_timeout_arg, offline_timeout_arg, check_arg, \
                    device_arg, callback_arg) \
    USB_RX_NODE_EX(port_arg, device_arg, callback_arg, \
        .mode = (mode_arg), .frame_len = (frame_len_arg), \
        .cache_size = (cache_size_arg), \
        .partial_timeout_ms = (partial_timeout_arg), \
        .offline_timeout_ms = (offline_timeout_arg), .check_frame = (check_arg))

#define USB_RX_NODE_11(port_arg, mode_arg, frame_len_arg, cache_size_arg, \
                     partial_timeout_arg, offline_timeout_arg, check_arg, \
                     rx_buf_arg, rx_buf_size_arg, device_arg, callback_arg) \
    USB_RX_NODE_EX(port_arg, device_arg, callback_arg, \
        .mode = (mode_arg), .frame_len = (frame_len_arg), \
        .cache_size = (cache_size_arg), \
        .partial_timeout_ms = (partial_timeout_arg), \
        .offline_timeout_ms = (offline_timeout_arg), .check_frame = (check_arg), \
        .rx_buf = (rx_buf_arg), .rx_buf_size = (rx_buf_size_arg))

/* Advanced settings are designated fields of the auto-generated config.
 * e.g. USB_RX_NODE_EX(port, &device, Resolve, .mode = USB_RX_DELIMITER,
 *                    .delimiter = newline, .delimiter_len = 2); */
#define USB_RX_NODE_EX(port_arg, device_arg, callback_arg, ...) \
    static const USB_RxConfig_t USB_JOIN(_usb_config_, __LINE__) = { __VA_ARGS__ }; \
    static const Auto_USB_Reg_t USB_JOIN(_usb_reg_, __LINE__) \
    __attribute__((used, section("USB_Reg_Sec"), aligned(4))) = { \
        .port = (port_arg), .config = &USB_JOIN(_usb_config_, __LINE__), \
        .device_ptr = (device_arg), .resolve = (callback_arg) \
    }

typedef struct {
    uint32_t baudrate;
    uint8_t stop_bits; /* CDC encoding: 0 = 1, 1 = 1.5, 2 = 2 stop bits. */
    uint8_t parity;
    uint8_t data_bits;
} USB_LineCoding_t;

typedef struct {
    bool configured;
    bool suspended;
    bool dtr;
    bool rts;
    bool ready;
    bool rx_paused;
    uint32_t session;
    USB_LineCoding_t line_coding;
} USB_LinkState_t;

typedef struct {
    USB_OverflowPolicy_t overflow_policy;
    bool require_dtr;             /* TX gating only; default false. */
    uint32_t tx_timeout_ms;       /* Default per-message timeout; zero disables. */
    uint32_t rx_pause_timeout_ms; /* Zero permits an indefinite host NAK. */
} USB_PortConfig_t;

typedef struct {
    uint32_t rx_packets, rx_bytes, rx_dropped_packets, rx_dropped_bytes;
    uint32_t rx_queue_high_water, rx_rearm_errors, rx_pause_timeouts;
    uint32_t tx_queued, tx_completed, tx_cancelled, tx_timeouts, tx_queue_full;
    uint32_t tx_queue_high_water, reconnects, events_lost;
} USB_Stats_t;

typedef struct {
    uint32_t valid_frames, bad_frames, cache_overflows, partial_timeouts;
    uint32_t parser_errors, last_valid_tick;
    bool has_valid_frame;
    bool online;
} USB_RxState_t;

/* Idempotent, static allocation only; safe before the RTOS starts. */
USB_Status_t Auto_USB_Router_Init(void);
/* Alternative to the linker registry. Call once, before processing starts.
 * Nodes/configuration must remain alive for the lifetime of the router. */
USB_Status_t USB_Router_InitNodes(const Auto_USB_Reg_t *nodes, size_t count);
USB_Status_t USB_ConfigurePort(USB_Port_t port, const USB_PortConfig_t *config);
/* Optional task-context notifications even when no receive node is registered.
 * Receives port-wide link/control/TX events; context is application-defined. */
USB_Status_t USB_SetPortEventCallback(USB_Port_t port, USB_EventCallback_t callback,
                                       void *context);
/* Exactly one consumer (USB_Service or a bare-metal loop), never an ISR. */
void USB_Process(uint32_t now_ms, uint32_t rx_budget_bytes);

/* Main/task context only. Copy-on-enqueue, all-or-nothing.
 * USB_OK means queued, not delivered; ISR calls return USB_ERR_BUSY. */
USB_Status_t USB_SendCopy(USB_Port_t port, const uint8_t *data, uint16_t len);
USB_Status_t USB_SendCopyEx(USB_Port_t port, const uint8_t *data, uint16_t len,
                          uint32_t timeout_ms, uint32_t *ticket);
/* Compatibility path: caller owns data until the native completion callback. */
USB_Status_t USB_SendLegacy(USB_Port_t port, uint8_t *data, uint16_t len);
USB_Status_t USB_GetLinkState(USB_Port_t port, USB_LinkState_t *state);
USB_Status_t USB_GetStats(USB_Port_t port, USB_Stats_t *stats);
USB_Status_t USB_GetRxState(USB_Port_t port, const void *device_ptr,
                           USB_RxState_t *state);
/* RAW mode: call from its resolve callback only after a valid protocol frame. */
USB_Status_t USB_RxMarkValid(USB_Port_t port, const void *device_ptr);

/* Adapter hooks: receive callbacks copy the bytes before rearming USB.
 * Link-down MUST run only after native endpoints have been stopped. */
USB_RxAction_t USB_RxPushISR(USB_Port_t port, uint8_t *data, uint32_t len,
                            uint32_t tick_ms);
void USB_LinkChangedISR(USB_Port_t port, bool configured, uint32_t tick_ms);
void USB_SuspendISR(USB_Port_t port, bool suspended, uint32_t tick_ms);
void USB_TxCompleteISR(USB_Port_t port, const uint8_t *data, uint32_t tick_ms);
void USB_RearmFailedISR(USB_Port_t port, uint8_t *data);
USB_Status_t USB_SetLineCoding(USB_Port_t port, const uint8_t *data, uint16_t len);
USB_Status_t USB_GetLineCoding(USB_Port_t port, uint8_t *data, uint16_t len);
void USB_SetControlLinesISR(USB_Port_t port, bool dtr, bool rts);
void USB_ReportBreakISR(USB_Port_t port, uint16_t duration_ms);

#ifdef __cplusplus
}
#endif
#endif
