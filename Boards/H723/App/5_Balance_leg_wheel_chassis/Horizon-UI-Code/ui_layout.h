/* 步兵3UI - 界面和图形编号 */
#ifndef UI_LAYOUT_H
#define UI_LAYOUT_H
#include "horizon_ui.h"
#ifdef __cplusplus
extern "C" {
#endif
enum {
    UI_ID_AIM_LEFT = 0, /* 准星 · 左 */
    UI_ID_AIM_RIGHT = 1, /* 准星 · 右 */
    UI_ID_AIM_TOP = 2, /* 准星 · 上 */
    UI_ID_AIM_BOTTOM = 3, /* 准星 · 下 */
    UI_ID_AIM_CENTER = 4, /* 瞄准中心 */
    UI_ID_MODE_LABEL = 5, /* 模式状态 */
    UI_ID_MODE_BORDER = 6, /* 状态边框 */
    UI_ID_HURT_DIRECTION = 7, /* hurt_direction */
    UI_ID_TARGET_LOCK = 8, /* target_lock */
    UI_ID_CAPACITOR_LABEL = 9, /* 电容标签 */
    UI_ID_CAPACITOR_BORDER = 10, /* 电容外框 */
    UI_ID_CAPACITOR_LEVEL = 11, /* 电容电量条 */
    UI_ID_CAPACITOR_PERCENT = 12, /* 电容百分比 */
    UI_ID_HEAT_MARGIN_TRACK = 13, /* 热量余量 · track */
    UI_ID_HEAT_MARGIN_LEVEL = 14, /* 热量余量 · level */
    UI_ID_HEAT_MARGIN_LABEL = 15, /* 热量余量 · label */
    UI_ID_HEAT_MARGIN_VALUE = 16, /* 热量余量 · value */
    UI_ID_AMMO_LABEL = 17, /* 弹量标签 */
    UI_ID_AMMO_COUNT = 18, /* 剩余弹量 */
    UI_ID_AMMO_COUNT_TRACK = 19, /* AMMO · track */
    UI_ID_AMMO_COUNT_LEVEL = 20, /* AMMO · level */
    UI_ID_POWER_LABEL = 21, /* 功率标签 */
    UI_ID_CHASSIS_POWER = 22, /* 底盘功率 */
    UI_ID_CHASSIS_POWER_TRACK = 23, /* POWER / W · track */
    UI_ID_CHASSIS_POWER_LEVEL = 24, /* POWER / W · level */
    UI_ID_ONLINE_STATE = 25, /* SYSTEM READY · state */
    UI_ID_ONLINE_SUMMARY = 26, /* SYSTEM READY · summary */
    UI_ID_RING_MARK_225 = 27, /* ring_mark_225 */
    UI_ID_RING_MARK_269 = 28, /* ring_mark_269 */
    UI_ID_RING_MARK_315 = 29, /* ring_mark_315 */
    UI_ID_RING_MARK_45 = 30, /* ring_mark_45 */
    UI_ID_RING_MARK_89 = 31, /* ring_mark_89 */
    UI_ID_RING_MARK_135 = 32, /* ring_mark_135 */
    UI_ID_BEARING_TICK_6 = 33, /* bearing_tick_-6 */
    UI_ID_BEARING_LABEL_6 = 34, /* bearing_label_-6 */
    UI_ID_BEARING_TICK_5 = 35, /* bearing_tick_-5 */
    UI_ID_BEARING_TICK_4 = 36, /* bearing_tick_-4 */
    UI_ID_BEARING_LABEL_4 = 37, /* bearing_label_-4 */
    UI_ID_BEARING_TICK_3 = 38, /* bearing_tick_-3 */
    UI_ID_BEARING_TICK_2 = 39, /* bearing_tick_-2 */
    UI_ID_BEARING_LABEL_2 = 40, /* bearing_label_-2 */
    UI_ID_BEARING_TICK_1 = 41, /* bearing_tick_-1 */
    UI_ID_BEARING_TICK_1_2 = 42, /* bearing_tick_1 */
    UI_ID_BEARING_TICK_2_2 = 43, /* bearing_tick_2 */
    UI_ID_BEARING_LABEL_2_2 = 44, /* bearing_label_2 */
    UI_ID_BEARING_TICK_3_2 = 45, /* bearing_tick_3 */
    UI_ID_BEARING_TICK_4_2 = 46, /* bearing_tick_4 */
    UI_ID_BEARING_LABEL_4_2 = 47, /* bearing_label_4 */
    UI_ID_BEARING_TICK_5_2 = 48, /* bearing_tick_5 */
    UI_ID_BEARING_TICK_6_2 = 49, /* bearing_tick_6 */
    UI_ID_BEARING_LABEL_6_2 = 50, /* bearing_label_6 */
    UI_ID_BEARING_BOX = 51, /* bearing_box */
    UI_ID_BEARING_TITLE = 52, /* bearing_title */
    UI_ID_BEARING_VALUE = 53, /* bearing_value */
    UI_ID_ATTITUDE_AXIS = 54, /* attitude · axis */
    UI_ID_ATTITUDE_AXIS_TICK_4 = 55, /* attitude · axis_tick_-4 */
    UI_ID_ATTITUDE_AXIS_TICK_3 = 56, /* attitude · axis_tick_-3 */
    UI_ID_ATTITUDE_AXIS_TICK_2 = 57, /* attitude · axis_tick_-2 */
    UI_ID_ATTITUDE_AXIS_TICK_1 = 58, /* attitude · axis_tick_-1 */
    UI_ID_ATTITUDE_AXIS_TICK_1_2 = 59, /* attitude · axis_tick_1 */
    UI_ID_ATTITUDE_AXIS_TICK_2_2 = 60, /* attitude · axis_tick_2 */
    UI_ID_ATTITUDE_AXIS_TICK_3_2 = 61, /* attitude · axis_tick_3 */
    UI_ID_ATTITUDE_AXIS_TICK_4_2 = 62, /* attitude · axis_tick_4 */
    UI_ID_ATTITUDE_RUNG_1_1 = 63, /* attitude · rung_-1_-1 */
    UI_ID_ATTITUDE_RUNG_1_1_2 = 64, /* attitude · rung_-1_1 */
    UI_ID_ATTITUDE_RUNG_0_1 = 65, /* attitude · rung_0_-1 */
    UI_ID_ATTITUDE_RUNG_0_1_2 = 66, /* attitude · rung_0_1 */
    UI_ID_ATTITUDE_RUNG_1_1_3 = 67, /* attitude · rung_1_-1 */
    UI_ID_ATTITUDE_RUNG_1_1_4 = 68, /* attitude · rung_1_1 */
    UI_ID_ATTITUDE_BANK_TRACK = 69, /* attitude · bank_track */
    UI_ID_ATTITUDE_BANK_TICK_60 = 70, /* attitude · bank_tick_-60 */
    UI_ID_ATTITUDE_BANK_TICK_45 = 71, /* attitude · bank_tick_-45 */
    UI_ID_ATTITUDE_BANK_TICK_30 = 72, /* attitude · bank_tick_-30 */
    UI_ID_ATTITUDE_BANK_TICK_15 = 73, /* attitude · bank_tick_-15 */
    UI_ID_ATTITUDE_BANK_TICK_0 = 74, /* attitude · bank_tick_0 */
    UI_ID_ATTITUDE_BANK_TICK_15_2 = 75, /* attitude · bank_tick_15 */
    UI_ID_ATTITUDE_BANK_TICK_30_2 = 76, /* attitude · bank_tick_30 */
    UI_ID_ATTITUDE_BANK_TICK_45_2 = 77, /* attitude · bank_tick_45 */
    UI_ID_ATTITUDE_BANK_TICK_60_2 = 78, /* attitude · bank_tick_60 */
    UI_ID_ATTITUDE_BANK_LABEL = 79, /* attitude · bank_label */
    UI_ID_ATTITUDE_BANK_VALUE_30 = 80, /* attitude · bank_value_-30 */
    UI_ID_ATTITUDE_BANK_VALUE_0 = 81, /* attitude · bank_value_0 */
    UI_ID_ATTITUDE_BANK_VALUE_30_2 = 82, /* attitude · bank_value_30 */
    UI_ID_ATTITUDE_BANK_POINTER = 83, /* attitude · bank_pointer */
    UI_ID_ATTITUDE_PITCH_LABEL = 84, /* attitude · pitch_label */
    UI_ID_ATTITUDE_PITCH_VALUE = 85, /* attitude · pitch_value */
    UI_ID_BATTLE_TITLE = 86, /* 战场态势标题 */
    UI_ID_BATTLE_RULE = 87, /* battle_rule */
    UI_ID_CONTROL_ALLY_LABEL = 88, /* control_ally_label */
    UI_ID_CONTROL_ALLY = 89, /* 己方累计控制度 */
    UI_ID_CONTROL_ALLY_UNKNOWN = 90, /* control_ally_unknown */
    UI_ID_CONTROL_ENEMY_LABEL = 91, /* control_enemy_label */
    UI_ID_CONTROL_ENEMY = 92, /* 对方累计控制度 */
    UI_ID_CONTROL_ENEMY_UNKNOWN = 93, /* control_enemy_unknown */
    UI_ID_CONTROL_DELTA_LABEL = 94, /* control_delta_label */
    UI_ID_CONTROL_DELTA = 95, /* 累计控制度差值 */
    UI_ID_CONTROL_DELTA_UNKNOWN = 96, /* control_delta_unknown */
    UI_ID_CONTROL_DELTA_RULE = 97, /* control_delta_rule */
    UI_ID_BASE_R_STATE_LABEL = 98, /* R 基地阵地 · 标签 */
    UI_ID_BASE_R_STATE_VALUE = 99, /* R 基地阵地 · 状态 */
    UI_ID_BATTLE_LINK_0 = 100, /* battle_link_0 */
    UI_ID_OUTPOST_R_STATE_LABEL = 101, /* R 前哨阵地 · 标签 */
    UI_ID_OUTPOST_R_STATE_VALUE = 102, /* R 前哨阵地 · 状态 */
    UI_ID_BATTLE_LINK_1 = 103, /* battle_link_1 */
    UI_ID_OUTPOST_B_STATE_LABEL = 104, /* B 前哨阵地 · 标签 */
    UI_ID_OUTPOST_B_STATE_VALUE = 105, /* B 前哨阵地 · 状态 */
    UI_ID_BATTLE_LINK_2 = 106, /* battle_link_2 */
    UI_ID_BASE_B_STATE_LABEL = 107, /* B 基地阵地 · 标签 */
    UI_ID_BASE_B_STATE_VALUE = 108, /* B 基地阵地 · 状态 */
    UI_ID_CONTROL_LEGEND_OWN = 109, /* control_legend_own */
    UI_ID_CONTROL_LEGEND_TEMP = 110, /* control_legend_temp */
    UI_ID_CONTROL_LEGEND_OTHER = 111, /* control_legend_other */
    UI_ID_SMALL_RUNE_STATE_LABEL = 112, /* 小能量机关 · 标签 */
    UI_ID_SMALL_RUNE_STATE_VALUE = 113, /* 小能量机关 · 状态 */
    UI_ID_LARGE_RUNE_STATE_LABEL = 114, /* 大能量机关 · 标签 */
    UI_ID_LARGE_RUNE_STATE_VALUE = 115, /* 大能量机关 · 状态 */
    UI_ID_ROBOT_TITLE = 116, /* 机器人状态标题 */
    UI_ID_ROBOT_RULE = 117, /* robot_rule */
    UI_ID_BUFFER_LABEL = 118, /* buffer_label */
    UI_ID_BUFFER_ENERGY = 119, /* 底盘缓冲能量 */
    UI_ID_BUFFER_ENERGY_UNKNOWN = 120, /* buffer_energy_unknown */
    UI_ID_POWER_LIMIT_LABEL = 121, /* power_limit_label */
    UI_ID_POWER_LIMIT = 122, /* 实时功率上限 */
    UI_ID_POWER_LIMIT_UNKNOWN = 123, /* power_limit_unknown */
    UI_ID_CHASSIS_OUTPUT_LABEL = 124, /* 底盘供电 · 标签 */
    UI_ID_CHASSIS_OUTPUT_VALUE = 125, /* 底盘供电 · 状态 */
    UI_ID_SHOOTER_OUTPUT_LABEL = 126, /* 发射机构供电 · 标签 */
    UI_ID_SHOOTER_OUTPUT_VALUE = 127, /* 发射机构供电 · 状态 */
    UI_ID_RFID_CONTACT_LABEL = 128, /* RFID 检测 · 标签 */
    UI_ID_RFID_CONTACT_VALUE = 129, /* RFID 检测 · 状态 */
    UI_ID_COMBAT_STATE_VALUE = 130, /* 比赛提示 · 状态 */
    UI_ID_GIMBAL_OUTPUT_LABEL = 131, /* 云台供电 · 标签 */
    UI_ID_GIMBAL_OUTPUT_VALUE = 132, /* 云台供电 · 状态 */
    UI_ID_CONTROL_LEGEND_UNKNOWN = 133, /* 未知状态 */
    UI_ID_HEAT_MARGIN_VALUE_UNKNOWN = 134, /* 等待数据 */
    UI_ID_CAPACITOR_PERCENT_UNKNOWN = 135, /* 等待数据 */
    UI_ELEMENT_COUNT = 136,
    UI_ELEMENT_CAPACITY = 136
};
enum {
    UI_FRAME_MAIN = 0, /* main */
    UI_TOTAL_FRAMES = 1
};
extern const UI_Element UI_Initial[UI_ELEMENT_CAPACITY];
#ifdef __cplusplus
}
#endif
#endif
