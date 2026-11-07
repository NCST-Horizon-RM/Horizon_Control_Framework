/* Protocol + adaptive scheduler. All UI packets share 30 transmission slots/s. */
#include "ui_layout.h"
#include <string.h>

static UI_Element elements[UI_ELEMENT_CAPACITY];
static uint8_t dirty[UI_ELEMENT_CAPACITY], present[UI_ELEMENT_CAPACITY], visible[UI_ELEMENT_CAPACITY];
static uint8_t changed[UI_ELEMENT_CAPACITY];
static uint32_t served[UI_ELEMENT_CAPACITY], service_order;
typedef struct { uint16_t packets, slots; } UI_BatchCost;
static UI_BatchCost batch_cost[UI_ELEMENT_CAPACITY + 1];
static const uint8_t batch_sizes[] = { 1, 2, 5, 7 };
/* Equalize average refresh counts across graphics and separately encoded texts. */
static uint64_t graphic_pass, text_pass;
static UI_SendFn transmit;
static uint16_t sender, active_frame, clear_layers, element_count;
static uint8_t sequence, clear_all, has_sent, rate_phase, next_gap;
static uint32_t last_send;
static uint8_t packet[120];
volatile uint8_t ui_redraw_request;

static void put16(uint8_t *b, uint16_t n) { b[0] = (uint8_t)n; b[1] = (uint8_t)(n >> 8); }
static void put32(uint8_t *b, uint32_t n) {
    b[0] = (uint8_t)n; b[1] = (uint8_t)(n >> 8);
    b[2] = (uint8_t)(n >> 16); b[3] = (uint8_t)(n >> 24);
}
static uint8_t crc8(const uint8_t *b, uint16_t n) {
    uint8_t c = 0xff;
    while (n--) {
        uint8_t k; c ^= *b++;
        for (k = 0; k < 8; ++k) c = (uint8_t)((c >> 1) ^ ((c & 1) ? 0x8c : 0));
    }
    return c;
}
static uint16_t crc16(const uint8_t *b, uint16_t n) {
    uint16_t c = 0xffff;
    while (n--) {
        uint8_t k; c ^= *b++;
        for (k = 0; k < 8; ++k) c = (uint16_t)((c >> 1) ^ ((c & 1) ? 0x8408 : 0));
    }
    return c;
}
static int32_t number_value(UI_Type type, double value) {
    int32_t number;
    value *= type == UI_FLOAT ? 1000.0 : 1.0;
    number = (int32_t)value;
    if (value - number >= 0.5) ++number;
    else if (value - number < -0.5) --number;
    return number;
}
static void encode(uint8_t *b, const UI_Element *e, uint8_t operation) {
    uint32_t a = 0, d = 0, c = 0, x2 = 0, y2 = 0;
    b[0] = (uint8_t)e->id; b[1] = (uint8_t)(e->id >> 8); b[2] = (uint8_t)(e->id >> 16);
    switch (e->type) {
    case UI_LINE: x2 = e->x2; y2 = e->y2; break;
    case UI_RECT: x2 = e->x + e->width; y2 = e->y + e->height; break;
    case UI_CIRCLE: c = e->radius; break;
    case UI_ELLIPSE: x2 = e->rx; y2 = e->ry; break;
    case UI_ARC: x2 = e->rx; y2 = e->ry; a = e->start_angle; d = e->end_angle; break;
    case UI_FLOAT: case UI_INTEGER: a = e->font_size; break;
    case UI_TEXT: a = e->font_size; d = (uint32_t)strlen(e->text); break;
    }
    put32(b + 3, ((uint32_t)operation & 7u) | (((uint32_t)e->type & 7u) << 3) |
          (((uint32_t)e->layer & 15u) << 6) | (((uint32_t)e->color & 15u) << 10) |
          ((a & 511u) << 14) | ((d & 511u) << 23));
    put32(b + 7, ((uint32_t)e->line_width & 1023u) | (((uint32_t)e->x & 2047u) << 10) |
          (((uint32_t)e->y & 2047u) << 21));
    if (e->type == UI_FLOAT || e->type == UI_INTEGER) {
        put32(b + 11, (uint32_t)number_value(e->type, e->value));
    } else put32(b + 11, (c & 1023u) | ((x2 & 2047u) << 10) | ((y2 & 2047u) << 21));
}
static uint8_t send_packet(uint16_t sub_id, uint16_t payload_length, uint32_t now) {
    uint16_t length = (uint16_t)(payload_length + 15u);
    packet[0] = 0xa5; put16(packet + 1, (uint16_t)(payload_length + 6u));
    packet[3] = sequence; packet[4] = crc8(packet, 4);
    put16(packet + 5, 0x0301); put16(packet + 7, sub_id);
    put16(packet + 9, sender); put16(packet + 11, (uint16_t)(sender + 0x100u));
    put16(packet + length - 2, crc16(packet, (uint16_t)(length - 2u)));
    if (!transmit(packet, length)) return 0;
    ++sequence;
    last_send = now; has_sent = 1;
    /* 34 + 33 + 33 = 100 ms. Late calls never cause catch-up bursts. */
    rate_phase = (uint8_t)(rate_phase + 10u);
    next_gap = 33;
    if (rate_phase >= 30u) { rate_phase -= 30u; ++next_gap; }
    return 1;
}

/* Minimize packet count first, then total wire bytes (padding is operation 0). */
static uint8_t choose_batch(uint16_t pending, uint16_t updates, UI_BatchCost *cost) {
    uint8_t k, best = 0;
    uint16_t best_packets = UINT16_MAX, best_slots = UINT16_MAX;
    const uint16_t minimum = updates > 7u ? 7u : updates;
    for (k = 0; k < sizeof(batch_sizes); ++k) {
        const uint8_t size = batch_sizes[k];
        const uint16_t remaining = pending > size ? (uint16_t)(pending - size) : 0;
        const uint16_t packets = (uint16_t)(1u + batch_cost[remaining].packets);
        const uint16_t slots = (uint16_t)(size + batch_cost[remaining].slots);
        if (size < minimum) continue;
        if (packets < best_packets || (packets == best_packets && slots <= best_slots)) {
            best = size; best_packets = packets; best_slots = slots;
        }
    }
    if (cost) { cost->packets = best_packets; cost->slots = best_slots; }
    return best;
}

void UI_Init(uint16_t robot_id, UI_SendFn send) {
    uint16_t i;
    memcpy(elements, UI_Initial, sizeof(elements));
    memset(dirty, 0, sizeof(dirty)); memset(present, 0, sizeof(present)); memset(visible, 0, sizeof(visible));
    memset(changed, 0, sizeof(changed)); memset(served, 0, sizeof(served));
    memset(batch_cost, 0, sizeof(batch_cost));
    for (i = 1; i <= UI_ELEMENT_CAPACITY; ++i) choose_batch(i, 0, &batch_cost[i]);
    sender = robot_id; transmit = send; active_frame = 0; element_count = UI_ELEMENT_COUNT;
    sequence = 0; clear_all = 0; clear_layers = 0; has_sent = 0; last_send = 0;
    rate_phase = 20; next_gap = 0; service_order = 0;
    graphic_pass = 0; text_pass = 0; ui_redraw_request = 0;
    UI_ShowFrame(0);
}
void UI_ShowFrame(uint16_t frame) {
    uint16_t i;
    if (frame >= UI_TOTAL_FRAMES) return;
    active_frame = frame; clear_all = 1; clear_layers = 0;
    graphic_pass = 0; text_pass = 0;
    for (i = 0; i < element_count; ++i) {
        present[i] = 0; changed[i] = 0; served[i] = 0;
        visible[i] = (uint8_t)(elements[i].frame == active_frame);
        dirty[i] = visible[i] ? 1 : 0;
    }
}
UI_Element *UI_Get(uint16_t id) { return id < element_count ? &elements[id] : NULL; }
void UI_MarkDirty(uint16_t id) {
    if (id < element_count && visible[id] && elements[id].frame == active_frame) {
        if (dirty[id] != 1) dirty[id] = present[id] ? 2 : 1;
        changed[id] = 1;
    }
}
void UI_SetNumber(uint16_t id, double value) {
    UI_Element *e = UI_Get(id);
    if (e) {
        const int32_t next = number_value(e->type, value);
        if (number_value(e->type, e->value) != next) {
            e->value = next / (e->type == UI_FLOAT ? 1000.0 : 1.0);
            UI_MarkDirty(id);
        }
    }
}
void UI_SetText(uint16_t id, const char *text) {
    UI_Element *e = UI_Get(id);
    if (e && strncmp(e->text, text, 30) != 0) { strncpy(e->text, text, 30); e->text[30] = '\0'; UI_MarkDirty(id); }
}
void UI_SetPosition(uint16_t id, uint16_t x, uint16_t y) {
    UI_Element *e = UI_Get(id);
    if (e && (e->x != x || e->y != y)) {
        if (e->type == UI_LINE) { e->x2 = (uint16_t)(e->x2 + x - e->x); e->y2 = (uint16_t)(e->y2 + y - e->y); }
        e->x = x; e->y = y; UI_MarkDirty(id);
    }
}
void UI_SetColor(uint16_t id, UI_Color color) {
    UI_Element *e = UI_Get(id); if (e && e->color != (uint16_t)color) { e->color = (uint16_t)color; UI_MarkDirty(id); }
}
void UI_SetLine(uint16_t id, uint16_t x, uint16_t y, uint16_t x2, uint16_t y2)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->x != x || e->y != y || e->x2 != x2 || e->y2 != y2)) {
        e->x = x;
        e->y = y;
        e->x2 = x2;
        e->y2 = y2;
        UI_MarkDirty(id);
    }
}

void UI_SetLineEnd(uint16_t id, uint16_t x2, uint16_t y2)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->x2 != x2 || e->y2 != y2)) {
        e->x2 = x2;
        e->y2 = y2;
        UI_MarkDirty(id);
    }
}

void UI_SetSize(uint16_t id, uint16_t width, uint16_t height)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->width != width || e->height != height)) {
        e->width = width;
        e->height = height;
        UI_MarkDirty(id);
    }
}

void UI_SetRadius(uint16_t id, uint16_t radius)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->radius != radius)) {
        e->radius = radius;
        UI_MarkDirty(id);
    }
}

void UI_SetRadii(uint16_t id, uint16_t rx, uint16_t ry)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->rx != rx || e->ry != ry)) {
        e->rx = rx;
        e->ry = ry;
        UI_MarkDirty(id);
    }
}

void UI_SetAngles(uint16_t id, uint16_t start_angle, uint16_t end_angle)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->start_angle != start_angle || e->end_angle != end_angle)) {
        e->start_angle = start_angle;
        e->end_angle = end_angle;
        UI_MarkDirty(id);
    }
}

void UI_SetLineWidth(uint16_t id, uint16_t line_width)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->line_width != line_width)) {
        e->line_width = line_width;
        UI_MarkDirty(id);
    }
}

void UI_SetFontSize(uint16_t id, uint16_t font_size)
{
    UI_Element *e = UI_Get(id);
    if (e && (e->font_size != font_size)) {
        e->font_size = font_size;
        UI_MarkDirty(id);
    }
}
void UI_SetVisible(uint16_t id, uint8_t show) {
    if (id >= element_count || elements[id].frame != active_frame) return;
    if (visible[id] == !!show) return;
    visible[id] = !!show;
    dirty[id] = show ? (present[id] ? 2 : 1) : (present[id] ? 3 : 0);
    changed[id] = (uint8_t)(dirty[id] != 0);
}
void UI_Refresh(void) {
    uint16_t i;
    for (i = 0; i < element_count; ++i) if (visible[i]) {
        dirty[i] = 1;
    }
}
void UI_RequestRedraw(void) {
    ui_redraw_request = 1;
}
void UI_Clear(void) {
    clear_all = 1; clear_layers = 0;
    memset(dirty, 0, sizeof(dirty)); memset(present, 0, sizeof(present)); memset(visible, 0, sizeof(visible));
}
void UI_ClearLayer(uint8_t layer) {
    uint16_t i; if (layer > 9) return;
    clear_layers |= (uint16_t)(1u << layer);
    for (i = 0; i < element_count; ++i) if (elements[i].layer == layer) { dirty[i] = 0; present[i] = 0; visible[i] = 0; }
}
void UI_Tick(uint32_t now_ms) {
    uint16_t ids[7], i, j, n, count, graphics = 0, texts = 0, updates = 0;
    uint16_t text_id = UINT16_MAX;
    uint64_t base;
    if (ui_redraw_request) {
        ui_redraw_request = 0;
        clear_all = 1; clear_layers = 0;
        graphic_pass = 0; text_pass = 0;
        for (i = 0; i < element_count; ++i) {
            present[i] = 0;
            dirty[i] = visible[i] ? 1 : 0;
        }
    }
    if (!transmit || (has_sent && (uint32_t)(now_ms - last_send) < next_gap)) return;
    if (clear_all || clear_layers) {
        uint8_t layer = 0;
        packet[13] = clear_all ? 2 : 1;
        if (!clear_all) while (!(clear_layers & (1u << layer))) ++layer;
        packet[14] = layer;
        if (send_packet(0x0100, 2, now_ms)) {
            if (clear_all) clear_all = 0;
            else clear_layers &= (uint16_t)~(1u << layer);
        }
        return;
    }
    for (i = 0; i < element_count; ++i) {
        if (!dirty[i]) continue;
        if (elements[i].type == UI_TEXT) {
            ++texts;
            if (text_id == UINT16_MAX || served[i] < served[text_id]) text_id = i;
        } else { ++graphics; if (changed[i]) ++updates; }
    }
    if (!graphics && !texts) { graphic_pass = 0; text_pass = 0; return; }
    if (!graphics || !texts) { graphic_pass = 0; text_pass = 0; }
    if (texts && (!graphics || text_pass < graphic_pass)) {
        i = text_id;
        memset(packet + 13, 0, 45); encode(packet + 13, &elements[i], dirty[i]);
        memcpy(packet + 28, elements[i].text, strlen(elements[i].text));
        if (!send_packet(0x0110, 45, now_ms)) return;
        present[i] = (uint8_t)(dirty[i] != 3); dirty[i] = 0; changed[i] = 0;
        served[i] = ++service_order;
        text_pass += (UINT64_C(1) << 32) / texts;
    } else {
        uint16_t sub_id;
        const uint8_t size = choose_batch(graphics, updates, NULL);
        count = graphics < size ? graphics : size;
        /* Oldest pending first: continuous updates cannot starve a redraw. */
        for (n = 0; n < count; ++n) {
            uint16_t best = UINT16_MAX;
            for (i = 0; i < element_count; ++i) {
                if (!dirty[i] || elements[i].type == UI_TEXT) continue;
                for (j = 0; j < n && ids[j] != i; ++j) { }
                if (j != n) continue;
                if (best == UINT16_MAX || served[i] < served[best]) best = i;
            }
            ids[n] = best;
        }
        sub_id = size == 7 ? 0x0104 : size == 5 ? 0x0103 : size == 2 ? 0x0102 : 0x0101;
        memset(packet + 13, 0, (size_t)size * 15u);
        for (n = 0; n < count; ++n) {
            i = ids[n]; encode(packet + 13 + n * 15, &elements[i], dirty[i]);
        }
        if (!send_packet(sub_id, (uint16_t)(size * 15u), now_ms)) return;
        for (n = 0; n < count; ++n) {
            i = ids[n]; present[i] = (uint8_t)(dirty[i] != 3);
            dirty[i] = 0; changed[i] = 0; served[i] = ++service_order;
        }
        graphic_pass += ((uint64_t)count << 32) / graphics;
    }
    base = graphic_pass < text_pass ? graphic_pass : text_pass;
    graphic_pass -= base; text_pass -= base;
}
