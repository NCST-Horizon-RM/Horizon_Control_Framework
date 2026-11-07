/* Horizon UI - RoboMaster 2027 V1.0.0. Portable C99, no HAL dependency. */
#ifndef HORIZON_UI_H
#define HORIZON_UI_H
#include <stdint.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    UI_LINE, UI_RECT, UI_CIRCLE, UI_ELLIPSE, UI_ARC, UI_FLOAT, UI_INTEGER, UI_TEXT
} UI_Type;
typedef enum {
    UI_TEAM, UI_YELLOW, UI_GREEN, UI_ORANGE, UI_PURPLE, UI_PINK, UI_CYAN, UI_BLACK, UI_WHITE
} UI_Color;
/* Wire data is encoded explicitly. This structure is NOT sent directly. */
typedef struct {
    uint32_t id;
    uint16_t frame;
    UI_Type type;
    uint16_t layer, color, line_width;
    uint16_t x, y, x2, y2, width, height;
    uint16_t radius, rx, ry, start_angle, end_angle, font_size;
    double value;
    char text[31];
} UI_Element;
/* Return 1 after accepting/copying bytes, or 0 to retry later. Never block. */
typedef uint8_t (*UI_SendFn)(const uint8_t *bytes, uint16_t length);
void UI_Init(uint16_t robot_id, UI_SendFn send);
void UI_ShowFrame(uint16_t frame);
void UI_Tick(uint32_t now_ms);
/* May be called from another task/ISR. UI_Tick clears and rebuilds the current view. */
void UI_RequestRedraw(void);
/* Set to 1 in Live Watch to request the same redraw. Consumed by UI_Tick. */
extern volatile uint8_t ui_redraw_request;
void UI_Refresh(void);
void UI_Clear(void);
void UI_ClearLayer(uint8_t layer);
UI_Element *UI_Get(uint16_t id);
void UI_MarkDirty(uint16_t id);
/* Setters compare values and mark changes. Coordinates/sizes: px; angles: degrees. */
void UI_SetNumber(uint16_t id, double value);
void UI_SetText(uint16_t id, const char *text);
void UI_SetPosition(uint16_t id, uint16_t x, uint16_t y);
void UI_SetLine(uint16_t id, uint16_t x, uint16_t y, uint16_t x2, uint16_t y2);
void UI_SetLineEnd(uint16_t id, uint16_t x2, uint16_t y2);
void UI_SetSize(uint16_t id, uint16_t width, uint16_t height);
void UI_SetRadius(uint16_t id, uint16_t radius);
void UI_SetRadii(uint16_t id, uint16_t rx, uint16_t ry);
void UI_SetAngles(uint16_t id, uint16_t start_angle, uint16_t end_angle);
void UI_SetLineWidth(uint16_t id, uint16_t line_width);
void UI_SetFontSize(uint16_t id, uint16_t font_size);
void UI_SetColor(uint16_t id, UI_Color color);
void UI_SetVisible(uint16_t id, uint8_t visible);
#ifdef __cplusplus
}
#endif
#endif
