/* 界面初始数据 */
#include "ui_layout.h"

const UI_Element UI_Initial[UI_ELEMENT_CAPACITY] = {
    /* main / 准星 · 左 */
    [UI_ID_AIM_LEFT] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 1,
        .layer = 0,
        .line_width = 2,
        .x = 912,
        .y = 540,
        .x2 = 940,
        .y2 = 540,
    },

    /* main / 准星 · 右 */
    [UI_ID_AIM_RIGHT] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 2,
        .layer = 0,
        .line_width = 2,
        .x = 980,
        .y = 540,
        .x2 = 1008,
        .y2 = 540,
    },

    /* main / 准星 · 上 */
    [UI_ID_AIM_TOP] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 3,
        .layer = 0,
        .line_width = 2,
        .x = 940,
        .y = 540,
        .x2 = 950,
        .y2 = 534,
    },

    /* main / 准星 · 下 */
    [UI_ID_AIM_BOTTOM] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 4,
        .layer = 0,
        .line_width = 2,
        .x = 970,
        .y = 534,
        .x2 = 980,
        .y2 = 540,
    },

    /* main / 瞄准中心 */
    [UI_ID_AIM_CENTER] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 5,
        .layer = 0,
        .line_width = 2,
        .x = 960,
        .y = 540,
        .radius = 4,
    },

    /* main / 模式状态 */
    [UI_ID_MODE_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 8,
        .layer = 0,
        .line_width = 1,
        .x = 836,
        .y = 862,
        .font_size = 16,
        .text = "AUTO AIM / READY",
    },

    /* main / 状态边框 */
    [UI_ID_MODE_BORDER] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 9,
        .layer = 0,
        .line_width = 1,
        .x = 836,
        .y = 840,
        .x2 = 1086,
        .y2 = 840,
    },

    /* main / hurt_direction */
    [UI_ID_HURT_DIRECTION] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_PINK,
        .id = 28,
        .layer = 0,
        .line_width = 7,
        .x = 960,
        .y = 540,
        .rx = 94,
        .ry = 94,
        .start_angle = 60,
        .end_angle = 100,
    },

    /* main / target_lock */
    [UI_ID_TARGET_LOCK] = {
        .type = UI_RECT,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 29,
        .layer = 0,
        .line_width = 2,
        .x = 936,
        .y = 518,
        .width = 48,
        .height = 44,
    },

    /* main / 电容标签 */
    [UI_ID_CAPACITOR_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 24,
        .layer = 0,
        .line_width = 1,
        .x = 650,
        .y = 792,
        .font_size = 14,
        .text = "CAP / %",
    },

    /* main / 电容外框 */
    [UI_ID_CAPACITOR_BORDER] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 25,
        .layer = 0,
        .line_width = 1,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 275,
        .end_angle = 315,
    },

    /* main / 电容电量条 */
    [UI_ID_CAPACITOR_LEVEL] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 26,
        .layer = 0,
        .line_width = 10,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 275,
        .end_angle = 315,
    },

    /* main / 电容百分比 */
    [UI_ID_CAPACITOR_PERCENT] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 27,
        .layer = 0,
        .line_width = 2,
        .x = 650,
        .y = 760,
        .font_size = 26,
        .value = 78,
    },

    /* main / 热量余量 · track */
    [UI_ID_HEAT_MARGIN_TRACK] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 38,
        .layer = 0,
        .line_width = 1,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 45,
        .end_angle = 85,
    },

    /* main / 热量余量 · level */
    [UI_ID_HEAT_MARGIN_LEVEL] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 39,
        .layer = 0,
        .line_width = 10,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 45,
        .end_angle = 85,
    },

    /* main / 热量余量 · label */
    [UI_ID_HEAT_MARGIN_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 40,
        .layer = 0,
        .line_width = 1,
        .x = 1186,
        .y = 792,
        .font_size = 14,
        .text = "HEAT LEFT",
    },

    /* main / 热量余量 · value */
    [UI_ID_HEAT_MARGIN_VALUE] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 41,
        .layer = 0,
        .line_width = 1,
        .x = 1200,
        .y = 760,
        .font_size = 26,
        .value = 40,
    },

    /* main / 弹量标签 */
    [UI_ID_AMMO_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 16,
        .layer = 0,
        .line_width = 1,
        .x = 650,
        .y = 288,
        .font_size = 14,
        .text = "AMMO",
    },

    /* main / 剩余弹量 */
    [UI_ID_AMMO_COUNT] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 17,
        .layer = 0,
        .line_width = 2,
        .x = 682,
        .y = 260,
        .font_size = 26,
        .value = 250,
    },

    /* main / AMMO · track */
    [UI_ID_AMMO_COUNT_TRACK] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 88,
        .layer = 0,
        .line_width = 1,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 225,
        .end_angle = 265,
    },

    /* main / AMMO · level */
    [UI_ID_AMMO_COUNT_LEVEL] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 89,
        .layer = 0,
        .line_width = 10,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 225,
        .end_angle = 265,
    },

    /* main / 功率标签 */
    [UI_ID_POWER_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 14,
        .layer = 0,
        .line_width = 1,
        .x = 1186,
        .y = 288,
        .font_size = 14,
        .text = "POWER / W",
    },

    /* main / 底盘功率 */
    [UI_ID_CHASSIS_POWER] = {
        .type = UI_FLOAT,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 15,
        .layer = 0,
        .line_width = 2,
        .x = 1186,
        .y = 260,
        .font_size = 26,
        .value = 65.2,
    },

    /* main / POWER / W · track */
    [UI_ID_CHASSIS_POWER_TRACK] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 92,
        .layer = 0,
        .line_width = 1,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 95,
        .end_angle = 135,
    },

    /* main / POWER / W · level */
    [UI_ID_CHASSIS_POWER_LEVEL] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 93,
        .layer = 0,
        .line_width = 10,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 95,
        .end_angle = 135,
    },

    /* main / SYSTEM READY · state */
    [UI_ID_ONLINE_STATE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 124,
        .layer = 0,
        .line_width = 4,
        .x = 848,
        .y = 147,
        .radius = 3,
    },

    /* main / SYSTEM READY · summary */
    [UI_ID_ONLINE_SUMMARY] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 125,
        .layer = 0,
        .line_width = 1,
        .x = 864,
        .y = 155,
        .font_size = 16,
        .text = "SYSTEM READY",
    },

    /* main / ring_mark_225 */
    [UI_ID_RING_MARK_225] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 126,
        .layer = 0,
        .line_width = 16,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 225,
        .end_angle = 227,
    },

    /* main / ring_mark_269 */
    [UI_ID_RING_MARK_269] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 127,
        .layer = 0,
        .line_width = 16,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 269,
        .end_angle = 271,
    },

    /* main / ring_mark_315 */
    [UI_ID_RING_MARK_315] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 128,
        .layer = 0,
        .line_width = 16,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 315,
        .end_angle = 317,
    },

    /* main / ring_mark_45 */
    [UI_ID_RING_MARK_45] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 129,
        .layer = 0,
        .line_width = 16,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 45,
        .end_angle = 47,
    },

    /* main / ring_mark_89 */
    [UI_ID_RING_MARK_89] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 130,
        .layer = 0,
        .line_width = 16,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 89,
        .end_angle = 91,
    },

    /* main / ring_mark_135 */
    [UI_ID_RING_MARK_135] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 131,
        .layer = 0,
        .line_width = 16,
        .x = 960,
        .y = 540,
        .rx = 300,
        .ry = 300,
        .start_angle = 135,
        .end_angle = 137,
    },

    /* main / bearing_tick_-6 */
    [UI_ID_BEARING_TICK_6] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 132,
        .layer = 0,
        .line_width = 1,
        .x = 780,
        .y = 770,
        .x2 = 780,
        .y2 = 781,
    },

    /* main / bearing_label_-6 */
    [UI_ID_BEARING_LABEL_6] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 133,
        .layer = 0,
        .line_width = 1,
        .x = 767,
        .y = 755,
        .font_size = 10,
        .value = -30,
    },

    /* main / bearing_tick_-5 */
    [UI_ID_BEARING_TICK_5] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 134,
        .layer = 0,
        .line_width = 1,
        .x = 810,
        .y = 770,
        .x2 = 810,
        .y2 = 776,
    },

    /* main / bearing_tick_-4 */
    [UI_ID_BEARING_TICK_4] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 135,
        .layer = 0,
        .line_width = 1,
        .x = 840,
        .y = 770,
        .x2 = 840,
        .y2 = 781,
    },

    /* main / bearing_label_-4 */
    [UI_ID_BEARING_LABEL_4] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 136,
        .layer = 0,
        .line_width = 1,
        .x = 827,
        .y = 755,
        .font_size = 10,
        .value = -20,
    },

    /* main / bearing_tick_-3 */
    [UI_ID_BEARING_TICK_3] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 137,
        .layer = 0,
        .line_width = 1,
        .x = 870,
        .y = 770,
        .x2 = 870,
        .y2 = 776,
    },

    /* main / bearing_tick_-2 */
    [UI_ID_BEARING_TICK_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 138,
        .layer = 0,
        .line_width = 1,
        .x = 900,
        .y = 770,
        .x2 = 900,
        .y2 = 781,
    },

    /* main / bearing_label_-2 */
    [UI_ID_BEARING_LABEL_2] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 139,
        .layer = 0,
        .line_width = 1,
        .x = 887,
        .y = 755,
        .font_size = 10,
        .value = -10,
    },

    /* main / bearing_tick_-1 */
    [UI_ID_BEARING_TICK_1] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 140,
        .layer = 0,
        .line_width = 1,
        .x = 930,
        .y = 770,
        .x2 = 930,
        .y2 = 776,
    },

    /* main / bearing_tick_1 */
    [UI_ID_BEARING_TICK_1_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 141,
        .layer = 0,
        .line_width = 1,
        .x = 990,
        .y = 770,
        .x2 = 990,
        .y2 = 776,
    },

    /* main / bearing_tick_2 */
    [UI_ID_BEARING_TICK_2_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 142,
        .layer = 0,
        .line_width = 1,
        .x = 1020,
        .y = 770,
        .x2 = 1020,
        .y2 = 781,
    },

    /* main / bearing_label_2 */
    [UI_ID_BEARING_LABEL_2_2] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 143,
        .layer = 0,
        .line_width = 1,
        .x = 1012,
        .y = 755,
        .font_size = 10,
        .value = 10,
    },

    /* main / bearing_tick_3 */
    [UI_ID_BEARING_TICK_3_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 144,
        .layer = 0,
        .line_width = 1,
        .x = 1050,
        .y = 770,
        .x2 = 1050,
        .y2 = 776,
    },

    /* main / bearing_tick_4 */
    [UI_ID_BEARING_TICK_4_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 145,
        .layer = 0,
        .line_width = 1,
        .x = 1080,
        .y = 770,
        .x2 = 1080,
        .y2 = 781,
    },

    /* main / bearing_label_4 */
    [UI_ID_BEARING_LABEL_4_2] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 146,
        .layer = 0,
        .line_width = 1,
        .x = 1072,
        .y = 755,
        .font_size = 10,
        .value = 20,
    },

    /* main / bearing_tick_5 */
    [UI_ID_BEARING_TICK_5_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 147,
        .layer = 0,
        .line_width = 1,
        .x = 1110,
        .y = 770,
        .x2 = 1110,
        .y2 = 776,
    },

    /* main / bearing_tick_6 */
    [UI_ID_BEARING_TICK_6_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 148,
        .layer = 0,
        .line_width = 1,
        .x = 1140,
        .y = 770,
        .x2 = 1140,
        .y2 = 781,
    },

    /* main / bearing_label_6 */
    [UI_ID_BEARING_LABEL_6_2] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 149,
        .layer = 0,
        .line_width = 1,
        .x = 1132,
        .y = 755,
        .font_size = 10,
        .value = 30,
    },

    /* main / bearing_box */
    [UI_ID_BEARING_BOX] = {
        .type = UI_RECT,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 150,
        .layer = 0,
        .line_width = 1,
        .x = 920,
        .y = 780,
        .width = 80,
        .height = 40,
    },

    /* main / bearing_title */
    [UI_ID_BEARING_TITLE] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 151,
        .layer = 0,
        .line_width = 1,
        .x = 946,
        .y = 834,
        .font_size = 10,
        .text = "HDG",
    },

    /* main / bearing_value */
    [UI_ID_BEARING_VALUE] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 152,
        .layer = 0,
        .line_width = 1,
        .x = 932,
        .y = 806,
        .font_size = 22,
        .value = 128,
    },

    /* main / attitude · axis */
    [UI_ID_ATTITUDE_AXIS] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 153,
        .layer = 0,
        .line_width = 1,
        .x = 960,
        .y = 425,
        .x2 = 960,
        .y2 = 655,
    },

    /* main / attitude · axis_tick_-4 */
    [UI_ID_ATTITUDE_AXIS_TICK_4] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 154,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 444,
        .x2 = 963,
        .y2 = 444,
    },

    /* main / attitude · axis_tick_-3 */
    [UI_ID_ATTITUDE_AXIS_TICK_3] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 155,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 468,
        .x2 = 963,
        .y2 = 468,
    },

    /* main / attitude · axis_tick_-2 */
    [UI_ID_ATTITUDE_AXIS_TICK_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 156,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 492,
        .x2 = 963,
        .y2 = 492,
    },

    /* main / attitude · axis_tick_-1 */
    [UI_ID_ATTITUDE_AXIS_TICK_1] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 157,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 516,
        .x2 = 963,
        .y2 = 516,
    },

    /* main / attitude · axis_tick_1 */
    [UI_ID_ATTITUDE_AXIS_TICK_1_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 158,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 564,
        .x2 = 963,
        .y2 = 564,
    },

    /* main / attitude · axis_tick_2 */
    [UI_ID_ATTITUDE_AXIS_TICK_2_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 159,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 588,
        .x2 = 963,
        .y2 = 588,
    },

    /* main / attitude · axis_tick_3 */
    [UI_ID_ATTITUDE_AXIS_TICK_3_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 160,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 612,
        .x2 = 963,
        .y2 = 612,
    },

    /* main / attitude · axis_tick_4 */
    [UI_ID_ATTITUDE_AXIS_TICK_4_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 161,
        .layer = 0,
        .line_width = 1,
        .x = 957,
        .y = 636,
        .x2 = 963,
        .y2 = 636,
    },

    /* main / attitude · rung_-1_-1 */
    [UI_ID_ATTITUDE_RUNG_1_1] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 164,
        .layer = 0,
        .line_width = 1,
        .x = 858,
        .y = 490,
        .x2 = 938,
        .y2 = 490,
    },

    /* main / attitude · rung_-1_1 */
    [UI_ID_ATTITUDE_RUNG_1_1_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 165,
        .layer = 0,
        .line_width = 1,
        .x = 1062,
        .y = 490,
        .x2 = 982,
        .y2 = 490,
    },

    /* main / attitude · rung_0_-1 */
    [UI_ID_ATTITUDE_RUNG_0_1] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 166,
        .layer = 0,
        .line_width = 2,
        .x = 842,
        .y = 540,
        .x2 = 938,
        .y2 = 540,
    },

    /* main / attitude · rung_0_1 */
    [UI_ID_ATTITUDE_RUNG_0_1_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 167,
        .layer = 0,
        .line_width = 2,
        .x = 1078,
        .y = 540,
        .x2 = 982,
        .y2 = 540,
    },

    /* main / attitude · rung_1_-1 */
    [UI_ID_ATTITUDE_RUNG_1_1_3] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 168,
        .layer = 0,
        .line_width = 1,
        .x = 858,
        .y = 590,
        .x2 = 938,
        .y2 = 590,
    },

    /* main / attitude · rung_1_1 */
    [UI_ID_ATTITUDE_RUNG_1_1_4] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 169,
        .layer = 0,
        .line_width = 1,
        .x = 1062,
        .y = 590,
        .x2 = 982,
        .y2 = 590,
    },

    /* main / attitude · bank_track */
    [UI_ID_ATTITUDE_BANK_TRACK] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 172,
        .layer = 0,
        .line_width = 1,
        .x = 960,
        .y = 540,
        .rx = 150,
        .ry = 150,
        .start_angle = 300,
        .end_angle = 60,
    },

    /* main / attitude · bank_tick_-60 */
    [UI_ID_ATTITUDE_BANK_TICK_60] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 173,
        .layer = 0,
        .line_width = 1,
        .x = 834,
        .y = 613,
        .x2 = 823,
        .y2 = 619,
    },

    /* main / attitude · bank_tick_-45 */
    [UI_ID_ATTITUDE_BANK_TICK_45] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 174,
        .layer = 0,
        .line_width = 1,
        .x = 857,
        .y = 643,
        .x2 = 851,
        .y2 = 649,
    },

    /* main / attitude · bank_tick_-30 */
    [UI_ID_ATTITUDE_BANK_TICK_30] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 175,
        .layer = 0,
        .line_width = 1,
        .x = 887,
        .y = 666,
        .x2 = 881,
        .y2 = 677,
    },

    /* main / attitude · bank_tick_-15 */
    [UI_ID_ATTITUDE_BANK_TICK_15] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 176,
        .layer = 0,
        .line_width = 1,
        .x = 922,
        .y = 681,
        .x2 = 920,
        .y2 = 689,
    },

    /* main / attitude · bank_tick_0 */
    [UI_ID_ATTITUDE_BANK_TICK_0] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 177,
        .layer = 0,
        .line_width = 1,
        .x = 960,
        .y = 686,
        .x2 = 960,
        .y2 = 698,
    },

    /* main / attitude · bank_tick_15 */
    [UI_ID_ATTITUDE_BANK_TICK_15_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 178,
        .layer = 0,
        .line_width = 1,
        .x = 998,
        .y = 681,
        .x2 = 1000,
        .y2 = 689,
    },

    /* main / attitude · bank_tick_30 */
    [UI_ID_ATTITUDE_BANK_TICK_30_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 179,
        .layer = 0,
        .line_width = 1,
        .x = 1033,
        .y = 666,
        .x2 = 1039,
        .y2 = 677,
    },

    /* main / attitude · bank_tick_45 */
    [UI_ID_ATTITUDE_BANK_TICK_45_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 180,
        .layer = 0,
        .line_width = 1,
        .x = 1063,
        .y = 643,
        .x2 = 1069,
        .y2 = 649,
    },

    /* main / attitude · bank_tick_60 */
    [UI_ID_ATTITUDE_BANK_TICK_60_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 181,
        .layer = 0,
        .line_width = 1,
        .x = 1086,
        .y = 613,
        .x2 = 1097,
        .y2 = 619,
    },

    /* main / attitude · bank_label */
    [UI_ID_ATTITUDE_BANK_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 182,
        .layer = 0,
        .line_width = 1,
        .x = 942,
        .y = 730,
        .font_size = 10,
        .text = "ROLL",
    },

    /* main / attitude · bank_value_-30 */
    [UI_ID_ATTITUDE_BANK_VALUE_30] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 183,
        .layer = 0,
        .line_width = 1,
        .x = 869,
        .y = 691,
        .font_size = 10,
        .value = 30,
    },

    /* main / attitude · bank_value_0 */
    [UI_ID_ATTITUDE_BANK_VALUE_0] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 184,
        .layer = 0,
        .line_width = 1,
        .x = 954,
        .y = 714,
        .font_size = 10,
        .value = 0,
    },

    /* main / attitude · bank_value_30 */
    [UI_ID_ATTITUDE_BANK_VALUE_30_2] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 185,
        .layer = 0,
        .line_width = 1,
        .x = 1039,
        .y = 691,
        .font_size = 10,
        .value = 30,
    },

    /* main / attitude · bank_pointer */
    [UI_ID_ATTITUDE_BANK_POINTER] = {
        .type = UI_ARC,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 186,
        .layer = 0,
        .line_width = 11,
        .x = 960,
        .y = 540,
        .rx = 150,
        .ry = 150,
        .start_angle = 358,
        .end_angle = 2,
    },

    /* main / attitude · pitch_label */
    [UI_ID_ATTITUDE_PITCH_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 187,
        .layer = 0,
        .line_width = 1,
        .x = 874,
        .y = 397,
        .font_size = 12,
        .text = "PITCH",
    },

    /* main / attitude · pitch_value */
    [UI_ID_ATTITUDE_PITCH_VALUE] = {
        .type = UI_FLOAT,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 188,
        .layer = 0,
        .line_width = 1,
        .x = 968,
        .y = 401,
        .font_size = 18,
        .value = 0,
    },

    /* main / 战场态势标题 */
    [UI_ID_BATTLE_TITLE] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 189,
        .layer = 0,
        .line_width = 1,
        .x = 100,
        .y = 845,
        .font_size = 15,
        .text = "BATTLE CONTROL",
    },

    /* main / battle_rule */
    [UI_ID_BATTLE_RULE] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 190,
        .layer = 0,
        .line_width = 1,
        .x = 100,
        .y = 826,
        .x2 = 520,
        .y2 = 826,
    },

    /* main / control_ally_label */
    [UI_ID_CONTROL_ALLY_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 191,
        .layer = 0,
        .line_width = 1,
        .x = 104,
        .y = 792,
        .font_size = 12,
        .text = "ALLY",
    },

    /* main / 己方累计控制度 */
    [UI_ID_CONTROL_ALLY] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 192,
        .layer = 0,
        .line_width = 2,
        .x = 104,
        .y = 753,
        .font_size = 30,
        .value = 0,
    },

    /* main / control_ally_unknown */
    [UI_ID_CONTROL_ALLY_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 193,
        .layer = 0,
        .line_width = 1,
        .x = 104,
        .y = 753,
        .font_size = 30,
        .text = "--",
    },

    /* main / control_enemy_label */
    [UI_ID_CONTROL_ENEMY_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 194,
        .layer = 0,
        .line_width = 1,
        .x = 334,
        .y = 792,
        .font_size = 12,
        .text = "OPP",
    },

    /* main / 对方累计控制度 */
    [UI_ID_CONTROL_ENEMY] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_PINK,
        .id = 195,
        .layer = 0,
        .line_width = 2,
        .x = 334,
        .y = 753,
        .font_size = 30,
        .value = 0,
    },

    /* main / control_enemy_unknown */
    [UI_ID_CONTROL_ENEMY_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 196,
        .layer = 0,
        .line_width = 1,
        .x = 334,
        .y = 753,
        .font_size = 30,
        .text = "--",
    },

    /* main / control_delta_label */
    [UI_ID_CONTROL_DELTA_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 197,
        .layer = 0,
        .line_width = 1,
        .x = 104,
        .y = 705,
        .font_size = 12,
        .text = "CP DELTA",
    },

    /* main / 累计控制度差值 */
    [UI_ID_CONTROL_DELTA] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 198,
        .layer = 0,
        .line_width = 1,
        .x = 335,
        .y = 705,
        .font_size = 22,
        .value = 0,
    },

    /* main / control_delta_unknown */
    [UI_ID_CONTROL_DELTA_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 199,
        .layer = 0,
        .line_width = 1,
        .x = 335,
        .y = 705,
        .font_size = 22,
        .text = "--",
    },

    /* main / control_delta_rule */
    [UI_ID_CONTROL_DELTA_RULE] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 200,
        .layer = 0,
        .line_width = 1,
        .x = 100,
        .y = 675,
        .x2 = 520,
        .y2 = 675,
    },

    /* main / R 基地阵地 · 标签 */
    [UI_ID_BASE_R_STATE_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 201,
        .layer = 0,
        .line_width = 1,
        .x = 100,
        .y = 632,
        .font_size = 12,
        .text = "R BASE",
    },

    /* main / R 基地阵地 · 状态 */
    [UI_ID_BASE_R_STATE_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 202,
        .layer = 0,
        .line_width = 2,
        .x = 136,
        .y = 650,
        .radius = 4,
    },

    /* main / battle_link_0 */
    [UI_ID_BATTLE_LINK_0] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 203,
        .layer = 0,
        .line_width = 1,
        .x = 144,
        .y = 650,
        .x2 = 236,
        .y2 = 650,
    },

    /* main / R 前哨阵地 · 标签 */
    [UI_ID_OUTPOST_R_STATE_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 204,
        .layer = 0,
        .line_width = 1,
        .x = 208,
        .y = 632,
        .font_size = 12,
        .text = "R OUT",
    },

    /* main / R 前哨阵地 · 状态 */
    [UI_ID_OUTPOST_R_STATE_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 205,
        .layer = 0,
        .line_width = 2,
        .x = 244,
        .y = 650,
        .radius = 4,
    },

    /* main / battle_link_1 */
    [UI_ID_BATTLE_LINK_1] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 206,
        .layer = 0,
        .line_width = 1,
        .x = 252,
        .y = 650,
        .x2 = 344,
        .y2 = 650,
    },

    /* main / B 前哨阵地 · 标签 */
    [UI_ID_OUTPOST_B_STATE_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 207,
        .layer = 0,
        .line_width = 1,
        .x = 316,
        .y = 632,
        .font_size = 12,
        .text = "B OUT",
    },

    /* main / B 前哨阵地 · 状态 */
    [UI_ID_OUTPOST_B_STATE_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 208,
        .layer = 0,
        .line_width = 2,
        .x = 352,
        .y = 650,
        .radius = 4,
    },

    /* main / battle_link_2 */
    [UI_ID_BATTLE_LINK_2] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 209,
        .layer = 0,
        .line_width = 1,
        .x = 360,
        .y = 650,
        .x2 = 452,
        .y2 = 650,
    },

    /* main / B 基地阵地 · 标签 */
    [UI_ID_BASE_B_STATE_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 210,
        .layer = 0,
        .line_width = 1,
        .x = 424,
        .y = 632,
        .font_size = 12,
        .text = "B BASE",
    },

    /* main / B 基地阵地 · 状态 */
    [UI_ID_BASE_B_STATE_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 211,
        .layer = 0,
        .line_width = 2,
        .x = 460,
        .y = 650,
        .radius = 4,
    },

    /* main / control_legend_own */
    [UI_ID_CONTROL_LEGEND_OWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_GREEN,
        .id = 212,
        .layer = 0,
        .line_width = 1,
        .x = 104,
        .y = 600,
        .font_size = 11,
        .text = "OWN",
    },

    /* main / control_legend_temp */
    [UI_ID_CONTROL_LEGEND_TEMP] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_YELLOW,
        .id = 213,
        .layer = 0,
        .line_width = 1,
        .x = 230,
        .y = 600,
        .font_size = 11,
        .text = "TEMP",
    },

    /* main / control_legend_other */
    [UI_ID_CONTROL_LEGEND_OTHER] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_PINK,
        .id = 214,
        .layer = 0,
        .line_width = 1,
        .x = 376,
        .y = 600,
        .font_size = 11,
        .text = "NOT OWN",
    },

    /* main / 小能量机关 · 标签 */
    [UI_ID_SMALL_RUNE_STATE_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 215,
        .layer = 0,
        .line_width = 1,
        .x = 128,
        .y = 567,
        .font_size = 11,
        .text = "SMALL RUNE",
    },

    /* main / 小能量机关 · 状态 */
    [UI_ID_SMALL_RUNE_STATE_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 216,
        .layer = 0,
        .line_width = 2,
        .x = 112,
        .y = 561,
        .radius = 4,
    },

    /* main / 大能量机关 · 标签 */
    [UI_ID_LARGE_RUNE_STATE_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 217,
        .layer = 0,
        .line_width = 1,
        .x = 356,
        .y = 567,
        .font_size = 11,
        .text = "LARGE RUNE",
    },

    /* main / 大能量机关 · 状态 */
    [UI_ID_LARGE_RUNE_STATE_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 218,
        .layer = 0,
        .line_width = 2,
        .x = 340,
        .y = 561,
        .radius = 4,
    },

    /* main / 机器人状态标题 */
    [UI_ID_ROBOT_TITLE] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 219,
        .layer = 0,
        .line_width = 1,
        .x = 1480,
        .y = 845,
        .font_size = 15,
        .text = "ROBOT STATUS",
    },

    /* main / robot_rule */
    [UI_ID_ROBOT_RULE] = {
        .type = UI_LINE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 220,
        .layer = 0,
        .line_width = 1,
        .x = 1480,
        .y = 826,
        .x2 = 1880,
        .y2 = 826,
    },

    /* main / buffer_label */
    [UI_ID_BUFFER_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 221,
        .layer = 0,
        .line_width = 1,
        .x = 1480,
        .y = 782,
        .font_size = 14,
        .text = "BUFFER / J",
    },

    /* main / 底盘缓冲能量 */
    [UI_ID_BUFFER_ENERGY] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 222,
        .layer = 0,
        .line_width = 1,
        .x = 1770,
        .y = 782,
        .font_size = 24,
        .value = 60,
    },

    /* main / buffer_energy_unknown */
    [UI_ID_BUFFER_ENERGY_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 223,
        .layer = 0,
        .line_width = 1,
        .x = 1770,
        .y = 782,
        .font_size = 24,
        .text = "--",
    },

    /* main / power_limit_label */
    [UI_ID_POWER_LIMIT_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 224,
        .layer = 0,
        .line_width = 1,
        .x = 1480,
        .y = 738,
        .font_size = 14,
        .text = "PWR LIMIT / W",
    },

    /* main / 实时功率上限 */
    [UI_ID_POWER_LIMIT] = {
        .type = UI_INTEGER,
        .frame = UI_FRAME_MAIN,
        .color = UI_CYAN,
        .id = 225,
        .layer = 0,
        .line_width = 1,
        .x = 1770,
        .y = 738,
        .font_size = 24,
        .value = 120,
    },

    /* main / power_limit_unknown */
    [UI_ID_POWER_LIMIT_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 226,
        .layer = 0,
        .line_width = 1,
        .x = 1770,
        .y = 738,
        .font_size = 24,
        .text = "--",
    },

    /* main / 底盘供电 · 标签 */
    [UI_ID_CHASSIS_OUTPUT_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 227,
        .layer = 0,
        .line_width = 1,
        .x = 1504,
        .y = 693,
        .font_size = 12,
        .text = "CHASSIS",
    },

    /* main / 底盘供电 · 状态 */
    [UI_ID_CHASSIS_OUTPUT_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 228,
        .layer = 0,
        .line_width = 2,
        .x = 1488,
        .y = 687,
        .radius = 4,
    },

    /* main / 发射机构供电 · 标签 */
    [UI_ID_SHOOTER_OUTPUT_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 229,
        .layer = 0,
        .line_width = 1,
        .x = 1716,
        .y = 693,
        .font_size = 12,
        .text = "SHOOTER",
    },

    /* main / 发射机构供电 · 状态 */
    [UI_ID_SHOOTER_OUTPUT_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 230,
        .layer = 0,
        .line_width = 2,
        .x = 1700,
        .y = 687,
        .radius = 4,
    },

    /* main / RFID 检测 · 标签 */
    [UI_ID_RFID_CONTACT_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 231,
        .layer = 0,
        .line_width = 1,
        .x = 1716,
        .y = 649,
        .font_size = 12,
        .text = "RFID",
    },

    /* main / RFID 检测 · 状态 */
    [UI_ID_RFID_CONTACT_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 232,
        .layer = 0,
        .line_width = 2,
        .x = 1700,
        .y = 643,
        .radius = 4,
    },

    /* main / 比赛提示 · 状态 */
    [UI_ID_COMBAT_STATE_VALUE] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 233,
        .layer = 0,
        .line_width = 1,
        .x = 1480,
        .y = 594,
        .font_size = 15,
        .text = "WAIT REFEREE",
    },

    /* main / 云台供电 · 标签 */
    [UI_ID_GIMBAL_OUTPUT_LABEL] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 234,
        .layer = 0,
        .line_width = 1,
        .x = 1504,
        .y = 649,
        .font_size = 12,
        .text = "GIMBAL",
    },

    /* main / 云台供电 · 状态 */
    [UI_ID_GIMBAL_OUTPUT_VALUE] = {
        .type = UI_CIRCLE,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 235,
        .layer = 0,
        .line_width = 2,
        .x = 1488,
        .y = 643,
        .radius = 4,
    },

    /* main / 未知状态 */
    [UI_ID_CONTROL_LEGEND_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 236,
        .layer = 0,
        .line_width = 1,
        .x = 480,
        .y = 600,
        .font_size = 11,
        .text = "UNK",
    },

    /* main / 等待数据 */
    [UI_ID_HEAT_MARGIN_VALUE_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 237,
        .layer = 0,
        .line_width = 1,
        .x = 1200,
        .y = 760,
        .font_size = 26,
        .text = "--",
    },

    /* main / 等待数据 */
    [UI_ID_CAPACITOR_PERCENT_UNKNOWN] = {
        .type = UI_TEXT,
        .frame = UI_FRAME_MAIN,
        .color = UI_WHITE,
        .id = 238,
        .layer = 0,
        .line_width = 1,
        .x = 650,
        .y = 760,
        .font_size = 26,
        .text = "--",
    },
};
