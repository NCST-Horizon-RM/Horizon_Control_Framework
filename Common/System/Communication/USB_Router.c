#include "USB_Router.h"
#include "USB_Framer.h"
#include "USB_Port.h"
#include <string.h>

_Static_assert(USB_MAX_RX_NODES > 0U && USB_RX_QUEUE_DEPTH > 0U &&
               USB_TX_QUEUE_DEPTH > 0U && USB_EVENT_QUEUE_DEPTH > 0U,
               "USB queue sizes must be positive");
_Static_assert(USB_MAX_FRAME_SIZE <= UINT16_MAX && USB_TX_MAX_SIZE <= UINT16_MAX &&
               USB_RX_PACKET_SIZE <= USB_MAX_FRAME_SIZE,
               "USB buffer sizes must fit the public length type");
_Static_assert(USB_MAX_RX_NODES <= INT16_MAX && USB_TX_QUEUE_DEPTH <= UINT16_MAX,
               "USB registry/queue indices exceed their storage type");
_Static_assert((USB_RX_QUEUE_DEPTH & (USB_RX_QUEUE_DEPTH - 1U)) == 0U &&
               (USB_TX_QUEUE_DEPTH & (USB_TX_QUEUE_DEPTH - 1U)) == 0U &&
               (USB_EVENT_QUEUE_DEPTH & (USB_EVENT_QUEUE_DEPTH - 1U)) == 0U,
               "USB ring depths must be powers of two for counter wrap");

typedef struct {
    uint8_t data[USB_RX_PACKET_SIZE];
    uint16_t len;
    uint32_t tick, sequence, session;
} RxPacket;
typedef enum { TX_FREE = 0, TX_WRITING, TX_READY, TX_ACTIVE, TX_DONE } TxState;
typedef struct {
    uint8_t data[USB_TX_MAX_SIZE];
    TxState state;
    uint16_t len;
    uint32_t ticket, session, tick, timeout;
    USB_Status_t result;
    bool report, timeout_reported;
} TxPacket;
typedef enum { OWNER_NONE = 0, OWNER_LEGACY, OWNER_QUEUE } TxOwner;
typedef struct {
    USB_LinkState_t link;
    USB_PortConfig_t config;
    USB_Stats_t stats;
    RxPacket rx[USB_RX_QUEUE_DEPTH];
    uint32_t rx_write, rx_read, rx_sequence;
    uint8_t *pending_data;
    uint16_t pending_len;
    uint32_t pending_tick, pending_sequence, pause_tick;
    bool pending_valid;
    TxPacket tx[USB_TX_QUEUE_DEPTH];
    uint32_t tx_write, tx_read, next_ticket;
    TxOwner owner;
    uint16_t active_tx;
    bool ever_configured;
    USB_EventCallback_t event_callback;
    void *event_context;
} PortState;
typedef struct {
    const Auto_USB_Reg_t *reg;
    USB_Framer_t framer;
    USB_RxState_t state;
    uint32_t session, sequence, rx_tick;
    bool seen_packet;
    bool expired;
} NodeState;
typedef struct {
    USB_Port_t port;
    int16_t node; /* -1 broadcasts to the port's registered nodes. */
    USB_Event_t event;
} QueuedEvent;

static PortState ports[USB_PORT_COUNT] = {
    [USB_PORT_CDC1] = {
        .link.line_coding = {115200U, 0U, 0U, 8U},
        .config = {USB_OVERFLOW_DROP_NEW, false, 1000U, 100U}
    }
};
static NodeState nodes[USB_MAX_RX_NODES];
static size_t node_count;
static const Auto_USB_Reg_t *registered_nodes;
static bool initialized, processing;
static NodeState *callback_node;
static QueuedEvent events[USB_EVENT_QUEUE_DEPTH];
static uint32_t event_write, event_read;

static bool ValidPort(USB_Port_t port) { return (unsigned)port < USB_PORT_COUNT; }
static bool Ready(USB_Port_t port)
{
    PortState *p = &ports[port];
    return p->link.configured && !p->link.suspended &&
        (!p->config.require_dtr || p->link.dtr) && USB_Port_CanTransmit(port);
}
static void EventLocked(USB_Port_t port, int16_t node, USB_EventCode_t code,
                         uint32_t ticket, uint32_t detail, uint32_t tick)
{
    if ((uint32_t)(event_write - event_read) == USB_EVENT_QUEUE_DEPTH) {
        ++ports[port].stats.events_lost;
        return;
    }
    events[event_write % USB_EVENT_QUEUE_DEPTH] =
        (QueuedEvent){port, node, {code, ticket, detail, tick}};
    ++event_write;
}
static void CountDropLocked(PortState *p, uint32_t len)
{
    ++p->stats.rx_dropped_packets;
    p->stats.rx_dropped_bytes += len;
}
static void EnqueueRxLocked(PortState *p, const uint8_t *data, uint16_t len,
                             uint32_t tick, uint32_t sequence)
{
    RxPacket *r = &p->rx[p->rx_write % USB_RX_QUEUE_DEPTH];
    memcpy(r->data, data, len);
    r->len = len;
    r->tick = tick;
    r->sequence = sequence;
    r->session = p->link.session;
    ++p->rx_write;
    uint32_t used = p->rx_write - p->rx_read;
    if (used > p->stats.rx_queue_high_water) p->stats.rx_queue_high_water = used;
}

USB_Status_t USB_Router_InitNodes(const Auto_USB_Reg_t *regs, size_t count)
{
    if ((regs == NULL && count != 0U) || count > USB_MAX_RX_NODES) return USB_ERR_CONFIG;
    uint32_t key = USB_Port_Lock();
    if (initialized) {
        USB_Status_t result = regs == registered_nodes && count == node_count ?
            USB_OK : USB_ERR_BUSY;
        USB_Port_Unlock(key);
        return result;
    }
    for (size_t i = 0U; i < count; ++i) {
        if (!ValidPort(regs[i].port) || regs[i].resolve == NULL ||
            USB_Framer_CheckConfig(regs[i].config) != USB_OK) {
            USB_Port_Unlock(key);
            return USB_ERR_CONFIG;
        }
        for (size_t j = 0U; j < i; ++j) {
            if (regs[j].port == regs[i].port && regs[j].device_ptr == regs[i].device_ptr) {
                USB_Port_Unlock(key);
                return USB_ERR_CONFIG;
            }
        }
    }
    /* Do not clear queues/link state: USB may already have received boot traffic. */
    for (size_t i = 0U; i < count; ++i) nodes[i].reg = &regs[i];
    registered_nodes = regs;
    node_count = count;
    initialized = true;
    USB_Port_Unlock(key);
    return USB_OK;
}

USB_Status_t Auto_USB_Router_Init(void)
{
    uint32_t key = USB_Port_Lock();
    bool ready = initialized;
    USB_Port_Unlock(key);
    if (ready) return USB_OK;
    extern const Auto_USB_Reg_t __start_USB_Reg_Sec[];
    extern const Auto_USB_Reg_t __stop_USB_Reg_Sec[];
    uintptr_t begin = (uintptr_t)__start_USB_Reg_Sec;
    uintptr_t end = (uintptr_t)__stop_USB_Reg_Sec;
    if (end < begin || (end - begin) % sizeof(Auto_USB_Reg_t) != 0U) return USB_ERR_CONFIG;
    return USB_Router_InitNodes(__start_USB_Reg_Sec,
                                (end - begin) / sizeof(Auto_USB_Reg_t));
}

USB_Status_t USB_ConfigurePort(USB_Port_t port, const USB_PortConfig_t *config)
{
    if (!ValidPort(port) || config == NULL || (unsigned)config->overflow_policy > USB_OVERFLOW_PAUSE ||
        config->tx_timeout_ms > INT32_MAX || config->rx_pause_timeout_ms > INT32_MAX) {
        return USB_ERR_ARG;
    }
    uint32_t key = USB_Port_Lock();
    ports[port].config = *config;
    USB_Port_Unlock(key);
    USB_ServiceNotify();
    return USB_OK;
}

USB_RxAction_t USB_RxPushISR(USB_Port_t port, uint8_t *data, uint32_t len, uint32_t tick)
{
    if (!ValidPort(port) || data == NULL) return USB_RX_REARM;
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    ++p->stats.rx_packets;
    p->stats.rx_bytes += len;
    if (len == 0U) {
        USB_Port_Unlock(key);
        return USB_RX_REARM;
    }
    uint32_t sequence = ++p->rx_sequence;
    if (len > USB_RX_PACKET_SIZE || !p->link.configured) {
        CountDropLocked(p, len);
        EventLocked(port, -1, USB_EVENT_RX_OVERFLOW, 0U, len, tick);
    } else if ((uint32_t)(p->rx_write - p->rx_read) < USB_RX_QUEUE_DEPTH) {
        EnqueueRxLocked(p, data, (uint16_t)len, tick, sequence);
    } else if (p->config.overflow_policy == USB_OVERFLOW_PAUSE && !p->pending_valid) {
        /* Native buffer remains owned by USB until the consumer copies it. */
        p->pending_data = data;
        p->pending_len = (uint16_t)len;
        p->pending_tick = tick;
        p->pending_sequence = sequence;
        p->pending_valid = true;
        p->pause_tick = tick;
        p->link.rx_paused = true;
        USB_Port_Unlock(key);
        USB_ServiceNotify();
        return USB_RX_PAUSE;
    } else {
        CountDropLocked(p, len);
        EventLocked(port, -1, USB_EVENT_RX_OVERFLOW, 0U, len, tick);
    }
    USB_Port_Unlock(key);
    USB_ServiceNotify();
    return USB_RX_REARM;
}

USB_Status_t USB_SetPortEventCallback(USB_Port_t port, USB_EventCallback_t callback,
                                       void *context)
{
    if (!ValidPort(port)) return USB_ERR_ARG;
    uint32_t key = USB_Port_Lock();
    ports[port].event_callback = callback;
    ports[port].event_context = context;
    USB_Port_Unlock(key);
    return USB_OK;
}

void USB_RearmFailedISR(USB_Port_t port, uint8_t *data)
{
    if (!ValidPort(port)) return;
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    ++p->stats.rx_rearm_errors;
    p->link.rx_paused = true;
    p->pending_data = data;
    p->pause_tick = USB_Port_Now();
    USB_Port_Unlock(key);
    USB_ServiceNotify();
}

void USB_LinkChangedISR(USB_Port_t port, bool configured, uint32_t tick)
{
    if (!ValidPort(port)) return;
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    /* Endpoints are already stopped on link-down. Only now may an active
     * queue buffer be reclaimed. A producer's TX_WRITING slot is never freed. */
    if (p->owner == OWNER_QUEUE) {
        TxPacket *tx = &p->tx[p->active_tx];
        tx->state = TX_DONE;
        tx->result = USB_ERR_NOT_READY;
    }
    p->owner = OWNER_NONE;
    while (p->rx_read != p->rx_write) {
        CountDropLocked(p, p->rx[p->rx_read % USB_RX_QUEUE_DEPTH].len);
        ++p->rx_read;
    }
    p->pending_valid = false;
    p->pending_data = NULL;
    p->link.rx_paused = false;
    p->link.configured = configured;
    p->link.suspended = false;
    p->link.dtr = false;
    p->link.rts = false;
    ++p->link.session;
    if (configured) {
        if (p->ever_configured) ++p->stats.reconnects;
        p->ever_configured = true;
    }
    EventLocked(port, -1, configured ? USB_EVENT_CONNECTED : USB_EVENT_DISCONNECTED,
                0U, p->link.session, tick);
    USB_Port_Unlock(key);
    USB_ServiceNotify();
}

void USB_SuspendISR(USB_Port_t port, bool suspended, uint32_t tick)
{
    if (!ValidPort(port)) return;
    uint32_t key = USB_Port_Lock();
    ports[port].link.suspended = suspended;
    EventLocked(port, -1, suspended ? USB_EVENT_SUSPENDED : USB_EVENT_RESUMED,
                0U, 0U, tick);
    USB_Port_Unlock(key);
    USB_ServiceNotify();
}

USB_Status_t USB_SetLineCoding(USB_Port_t port, const uint8_t *data, uint16_t len)
{
    if (!ValidPort(port) || data == NULL || len != 7U ||
        data[4] > 2U || data[5] > 4U ||
        (data[6] != 5U && data[6] != 6U && data[6] != 7U &&
         data[6] != 8U && data[6] != 16U)) return USB_ERR_ARG;
    USB_LineCoding_t coding = {
        (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U),
        data[4], data[5], data[6]
    };
    uint32_t key = USB_Port_Lock();
    ports[port].link.line_coding = coding;
    EventLocked(port, -1, USB_EVENT_CONTROL, 0U, coding.baudrate, USB_Port_Now());
    USB_Port_Unlock(key);
    USB_ServiceNotify();
    return USB_OK;
}

USB_Status_t USB_GetLineCoding(USB_Port_t port, uint8_t *data, uint16_t len)
{
    if (!ValidPort(port) || data == NULL || len != 7U) return USB_ERR_ARG;
    uint32_t key = USB_Port_Lock();
    USB_LineCoding_t c = ports[port].link.line_coding;
    USB_Port_Unlock(key);
    data[0] = (uint8_t)c.baudrate;
    data[1] = (uint8_t)(c.baudrate >> 8U);
    data[2] = (uint8_t)(c.baudrate >> 16U);
    data[3] = (uint8_t)(c.baudrate >> 24U);
    data[4] = c.stop_bits;
    data[5] = c.parity;
    data[6] = c.data_bits;
    return USB_OK;
}

void USB_SetControlLinesISR(USB_Port_t port, bool dtr, bool rts)
{
    if (!ValidPort(port)) return;
    uint32_t key = USB_Port_Lock();
    ports[port].link.dtr = dtr;
    ports[port].link.rts = rts;
    EventLocked(port, -1, USB_EVENT_CONTROL, 0U, (dtr ? 1U : 0U) | (rts ? 2U : 0U),
                USB_Port_Now());
    USB_Port_Unlock(key);
    USB_ServiceNotify();
}

void USB_ReportBreakISR(USB_Port_t port, uint16_t duration_ms)
{
    if (!ValidPort(port)) return;
    uint32_t key = USB_Port_Lock();
    EventLocked(port, -1, USB_EVENT_BREAK, 0U, duration_ms, USB_Port_Now());
    USB_Port_Unlock(key);
    USB_ServiceNotify();
}

USB_Status_t USB_SendCopyEx(USB_Port_t port, const uint8_t *data, uint16_t len,
                          uint32_t timeout, uint32_t *ticket)
{
    if (ticket != NULL) *ticket = 0U;
    if (USB_Port_InISR()) return USB_ERR_BUSY;
    if (!ValidPort(port) || data == NULL || len == 0U || timeout > INT32_MAX) return USB_ERR_ARG;
    if (len > USB_TX_MAX_SIZE) return USB_ERR_SIZE;
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    if (!Ready(port)) {
        USB_Port_Unlock(key);
        return USB_ERR_NOT_READY;
    }
    if ((uint32_t)(p->tx_write - p->tx_read) == USB_TX_QUEUE_DEPTH) {
        ++p->stats.tx_queue_full;
        USB_Port_Unlock(key);
        return USB_ERR_FULL;
    }
    TxPacket *tx = &p->tx[p->tx_write % USB_TX_QUEUE_DEPTH];
    tx->state = TX_WRITING;
    tx->session = p->link.session;
    tx->len = len;
    tx->tick = USB_Port_Now();
    tx->timeout = timeout;
    tx->report = false;
    tx->timeout_reported = false;
    tx->result = USB_OK;
    if (++p->next_ticket == 0U) ++p->next_ticket;
    uint32_t id = tx->ticket = p->next_ticket;
    ++p->tx_write;
    uint32_t used = p->tx_write - p->tx_read;
    if (used > p->stats.tx_queue_high_water) p->stats.tx_queue_high_water = used;
    USB_Port_Unlock(key);

    /* A producer reserves its slot first, copies with interrupts enabled,
     * then publishes it. The consumer cannot pass a TX_WRITING slot. */
    memcpy(tx->data, data, len);
    key = USB_Port_Lock();
    USB_Status_t result;
    if (tx->session != p->link.session || !Ready(port)) {
        tx->result = USB_ERR_NOT_READY;
        tx->state = TX_DONE;
        result = USB_ERR_NOT_READY;
    } else {
        tx->report = true;
        tx->state = TX_READY;
        ++p->stats.tx_queued;
        if (ticket != NULL) *ticket = id;
        result = USB_OK;
    }
    USB_Port_Unlock(key);
    USB_ServiceNotify();
    return result;
}

USB_Status_t USB_SendCopy(USB_Port_t port, const uint8_t *data, uint16_t len)
{
    if (!ValidPort(port)) return USB_ERR_ARG;
    uint32_t key = USB_Port_Lock();
    uint32_t timeout = ports[port].config.tx_timeout_ms;
    USB_Port_Unlock(key);
    return USB_SendCopyEx(port, data, len, timeout, NULL);
}

USB_Status_t USB_SendLegacy(USB_Port_t port, uint8_t *data, uint16_t len)
{
    if (!ValidPort(port) || (data == NULL && len != 0U)) return USB_ERR_ARG;
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    USB_Status_t result;
    if (!Ready(port)) result = USB_ERR_NOT_READY;
    else if (p->owner != OWNER_NONE) result = USB_ERR_BUSY;
    else {
        p->owner = OWNER_LEGACY;
        result = USB_Port_Transmit(port, data, len);
        if (result != USB_OK) p->owner = OWNER_NONE;
    }
    USB_Port_Unlock(key);
    return result;
}

void USB_TxCompleteISR(USB_Port_t port, const uint8_t *data, uint32_t tick)
{
    (void)tick;
    if (!ValidPort(port)) return;
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    if (p->owner == OWNER_QUEUE) {
        TxPacket *tx = &p->tx[p->active_tx];
        if (tx->state == TX_ACTIVE && data == tx->data && tx->session == p->link.session) {
            tx->state = TX_DONE;
            p->owner = OWNER_NONE;
        }
    } else if (p->owner == OWNER_LEGACY) {
        p->owner = OWNER_NONE;
    }
    USB_Port_Unlock(key);
    USB_ServiceNotify();
}

static bool NodeOnlineLocked(NodeState *n, uint32_t now)
{
    const PortState *p = &ports[n->reg->port];
    uint32_t timeout = n->reg->config->offline_timeout_ms;
    if (n->state.has_valid_frame &&
        USB_TickExpired(now, n->state.last_valid_tick, timeout)) n->expired = true;
    /* Once expired, a full 32-bit tick wrap must not revive stale data. */
    return n->state.has_valid_frame && !n->expired && n->session == p->link.session &&
        p->link.configured && !p->link.suspended &&
        !USB_TickExpired(now, n->state.last_valid_tick, timeout);
}

static void MarkValidLocked(NodeState *n)
{
    ++n->state.valid_frames;
    n->state.has_valid_frame = true;
    n->expired = false;
    n->state.last_valid_tick = n->rx_tick;
}

USB_Status_t USB_RxMarkValid(USB_Port_t port, const void *device)
{
    uint32_t key = USB_Port_Lock();
    NodeState *n = callback_node;
    USB_Status_t result = USB_ERR_BUSY;
    if (n != NULL && n->reg->port == port && n->reg->device_ptr == device &&
        n->session == ports[port].link.session && ports[port].link.configured) {
        MarkValidLocked(n);
        result = USB_OK;
    }
    USB_Port_Unlock(key);
    return result;
}

static void EmitFrame(void *context, uint8_t *data, uint16_t len)
{
    NodeState *n = context;
    const USB_RxConfig_t *config = n->reg->config;
    if (config->rx_buf != NULL && len > config->rx_buf_size) {
        ++n->framer.overflows;
        return;
    }
    uint32_t key = USB_Port_Lock();
    if (n->session != ports[n->reg->port].link.session ||
        !ports[n->reg->port].link.configured) {
        USB_Port_Unlock(key);
        return;
    }
    if (n->reg->config->mode != USB_RX_RAW) MarkValidLocked(n);
    callback_node = n;
    USB_Port_Unlock(key);
    if (config->rx_buf != NULL) {
        memcpy(config->rx_buf, data, len);
        data = config->rx_buf;
    }
    n->reg->resolve(data, n->reg->device_ptr, len);
    key = USB_Port_Lock();
    callback_node = NULL;
    USB_Port_Unlock(key);
}

static void PublishFramer(NodeState *n, uint32_t tick)
{
    uint32_t key = USB_Port_Lock();
    int16_t index = (int16_t)(n - nodes);
#define PUBLISH_COUNTER(field, source, event_code) do { \
    uint32_t delta = n->framer.source - n->state.field; \
    if (delta != 0U) EventLocked(n->reg->port, index, event_code, 0U, delta, tick); \
    n->state.field = n->framer.source; \
} while (0)
    PUBLISH_COUNTER(bad_frames, bad_frames, USB_EVENT_BAD_FRAME);
    PUBLISH_COUNTER(cache_overflows, overflows, USB_EVENT_RX_OVERFLOW);
    PUBLISH_COUNTER(partial_timeouts, timeouts, USB_EVENT_RX_TIMEOUT);
    PUBLISH_COUNTER(parser_errors, parser_errors, USB_EVENT_PARSER_ERROR);
#undef PUBLISH_COUNTER
    USB_Port_Unlock(key);
}

static void ResumeRx(USB_Port_t port, uint32_t now)
{
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    if (p->link.rx_paused && p->link.configured && !p->link.suspended) {
        if (p->pending_valid && USB_TickExpired(now, p->pause_tick,
                                                p->config.rx_pause_timeout_ms)) {
            CountDropLocked(p, p->pending_len);
            ++p->stats.rx_pause_timeouts;
            EventLocked(port, -1, USB_EVENT_RX_OVERFLOW, 0U, p->pending_len, now);
            p->pending_valid = false;
        }
        if (p->pending_valid && (uint32_t)(p->rx_write - p->rx_read) < USB_RX_QUEUE_DEPTH) {
            EnqueueRxLocked(p, p->pending_data, p->pending_len,
                             p->pending_tick, p->pending_sequence);
            p->pending_valid = false;
        }
        if (!p->pending_valid && p->pending_data != NULL &&
            USB_Port_Rearm(port, p->pending_data) == USB_OK) {
            p->link.rx_paused = false;
            p->pending_data = NULL;
        }
    }
    USB_Port_Unlock(key);
}

static void ProcessTx(USB_Port_t port, uint32_t now)
{
    uint32_t key = USB_Port_Lock();
    PortState *p = &ports[port];
    for (size_t count = 0U; count < USB_TX_QUEUE_DEPTH && p->tx_read != p->tx_write; ++count) {
        TxPacket *tx = &p->tx[p->tx_read % USB_TX_QUEUE_DEPTH];
        if (tx->state == TX_WRITING) break;
        if (tx->state == TX_READY &&
            (tx->session != p->link.session || !p->link.configured)) {
            tx->result = USB_ERR_NOT_READY;
            tx->state = TX_DONE;
        }
        if ((tx->state == TX_READY || tx->state == TX_ACTIVE) &&
            !tx->timeout_reported && USB_TickExpired(now, tx->tick, tx->timeout)) {
            tx->timeout_reported = true;
            tx->result = USB_ERR_TIMEOUT;
            ++p->stats.tx_timeouts;
            if (tx->report) EventLocked(port, -1, USB_EVENT_TX_TIMEOUT, tx->ticket, 0U, now);
            /* An ACTIVE buffer stays reserved until completion or endpoint teardown. */
            if (tx->state == TX_READY) tx->state = TX_DONE;
        }
        if (tx->state == TX_DONE) {
            if (tx->report && !tx->timeout_reported) {
                if (tx->result == USB_OK) ++p->stats.tx_completed;
                else ++p->stats.tx_cancelled;
                EventLocked(port, -1, tx->result == USB_OK ? USB_EVENT_TX_DONE :
                            USB_EVENT_TX_CANCELLED, tx->ticket, tx->result, now);
            }
            tx->state = TX_FREE;
            ++p->tx_read;
            continue;
        }
        if (tx->state == TX_READY && p->owner == OWNER_NONE && Ready(port)) {
            p->owner = OWNER_QUEUE;
            p->active_tx = (uint16_t)(p->tx_read % USB_TX_QUEUE_DEPTH);
            tx->state = TX_ACTIVE;
            USB_Status_t result = USB_Port_Transmit(port, tx->data, tx->len);
            if (result != USB_OK) {
                p->owner = OWNER_NONE;
                tx->state = TX_READY;
            }
        }
        break;
    }
    USB_Port_Unlock(key);
}

static void DispatchEvents(uint32_t *budget)
{
    while (*budget != 0U) {
        uint32_t key = USB_Port_Lock();
        if (event_read == event_write) {
            USB_Port_Unlock(key);
            break;
        }
        QueuedEvent e = events[event_read % USB_EVENT_QUEUE_DEPTH];
        ++event_read;
        --*budget;
        USB_EventCallback_t port_callback = ports[e.port].event_callback;
        void *port_context = ports[e.port].event_context;
        USB_Port_Unlock(key);
        if (e.node < 0 && port_callback != NULL) port_callback(port_context, &e.event);
        for (size_t i = 0U; i < node_count; ++i) {
            if (nodes[i].reg->port == e.port && (e.node < 0 || (size_t)e.node == i) &&
                nodes[i].reg->config->event_callback != NULL) {
                nodes[i].reg->config->event_callback(nodes[i].reg->device_ptr, &e.event);
            }
        }
    }
}

void USB_Process(uint32_t now, uint32_t budget)
{
    if (USB_Port_InISR()) return;
    uint32_t key = USB_Port_Lock();
    if (!initialized || processing) {
        USB_Port_Unlock(key);
        return;
    }
    processing = true;
    USB_Port_Unlock(key);

    for (size_t i = 0U; i < node_count; ++i) {
        NodeState *n = &nodes[i];
        key = USB_Port_Lock();
        uint32_t session = ports[n->reg->port].link.session;
        bool reset = n->session != session;
        if (reset) {
            n->session = session;
            n->seen_packet = false;
            n->state.has_valid_frame = false;
        }
        USB_Port_Unlock(key);
        if (reset) USB_Framer_Reset(&n->framer, n->reg->config);
    }
    uint32_t event_budget = USB_EVENT_QUEUE_DEPTH;
    DispatchEvents(&event_budget);
    uint32_t processed = 0U;
    for (unsigned bus = 0U; bus < USB_PORT_COUNT; ++bus) {
        PortState *p = &ports[bus];
        while (processed < budget) {
            /* Deliver a queued disconnect/reconnect before newer RX data.
             * Check and dequeue atomically, so an intervening USB ISR either
             * leaves an event pending or invalidates the popped packet epoch. */
            DispatchEvents(&event_budget);
            RxPacket packet;
            key = USB_Port_Lock();
            if (event_read != event_write || p->rx_read == p->rx_write) {
                USB_Port_Unlock(key);
                break;
            }
            packet = p->rx[p->rx_read % USB_RX_QUEUE_DEPTH];
            ++p->rx_read;
            bool current = packet.session == p->link.session && p->link.configured;
            USB_Port_Unlock(key);
            processed += packet.len;
            if (!current) continue;
            for (size_t i = 0U; i < node_count; ++i) {
                NodeState *n = &nodes[i];
                if (n->reg->port != (USB_Port_t)bus) continue;
                if (n->session != packet.session ||
                    (n->seen_packet && packet.sequence != n->sequence + 1U)) {
                    USB_Framer_Reset(&n->framer, n->reg->config);
                    key = USB_Port_Lock();
                    EventLocked((USB_Port_t)bus, (int16_t)i, USB_EVENT_RX_GAP,
                                0U, packet.sequence - n->sequence - 1U, packet.tick);
                    USB_Port_Unlock(key);
                }
                n->session = packet.session;
                n->seen_packet = true;
                n->sequence = packet.sequence;
                n->rx_tick = packet.tick;
                USB_Framer_Feed(&n->framer, n->reg->config, packet.data, packet.len,
                                 packet.tick, EmitFrame, n);
                PublishFramer(n, packet.tick);
            }
        }
        ResumeRx((USB_Port_t)bus, now);
        ProcessTx((USB_Port_t)bus, now);
    }
    for (size_t i = 0U; i < node_count; ++i) {
        NodeState *n = &nodes[i];
        key = USB_Port_Lock();
        PortState *p = &ports[n->reg->port];
        bool empty = p->rx_read == p->rx_write && !p->pending_valid;
        bool online = NodeOnlineLocked(n, now);
        if (n->state.online != online) {
            n->state.online = online;
            EventLocked(n->reg->port, (int16_t)i, online ? USB_EVENT_ONLINE :
                        USB_EVENT_OFFLINE, 0U, 0U, now);
        }
        USB_Port_Unlock(key);
        /* Drain queued arrival timestamps before checking wall-clock expiry. */
        if (empty && USB_Framer_Expire(&n->framer, n->reg->config, now)) PublishFramer(n, now);
    }
    DispatchEvents(&event_budget);
    key = USB_Port_Lock();
    processing = false;
    USB_Port_Unlock(key);
}

USB_Status_t USB_GetLinkState(USB_Port_t port, USB_LinkState_t *state)
{
    if (!ValidPort(port) || state == NULL) return USB_ERR_ARG;
    uint32_t key = USB_Port_Lock();
    *state = ports[port].link;
    state->ready = Ready(port);
    USB_Port_Unlock(key);
    return USB_OK;
}

USB_Status_t USB_GetStats(USB_Port_t port, USB_Stats_t *stats)
{
    if (!ValidPort(port) || stats == NULL) return USB_ERR_ARG;
    uint32_t key = USB_Port_Lock();
    *stats = ports[port].stats;
    USB_Port_Unlock(key);
    return USB_OK;
}

USB_Status_t USB_GetRxState(USB_Port_t port, const void *device, USB_RxState_t *state)
{
    if (!ValidPort(port) || state == NULL) return USB_ERR_ARG;
    uint32_t key = USB_Port_Lock();
    for (size_t i = 0U; i < node_count; ++i) {
        NodeState *n = &nodes[i];
        if (n->reg->port == port && n->reg->device_ptr == device) {
            *state = n->state;
            state->online = NodeOnlineLocked(n, USB_Port_Now());
            if (n->session != ports[port].link.session) state->has_valid_frame = false;
            USB_Port_Unlock(key);
            return USB_OK;
        }
    }
    USB_Port_Unlock(key);
    return USB_ERR_ARG;
}
