/* 由设计器生成的数据映射；发送调度由现有 UI_Tick 负责。 */
#include "ui_bindings.h"
#include <string.h>

#include <math.h>


const UI_Data UI_DataDefault = {
    .mode_text = "AUTO AIM / READY",
    .mode_color = UI_GREEN,
    .hurt_start = 60,
    .hurt_end = 100,
    .hurt_active = 0,
    .target_detected = 0,
    .capacitor_percent = 78,
    .capacitor_color = UI_CYAN,
    .ammo_count = 250,
    .ammo_count_color = UI_WHITE,
    .chassis_power = 65.2,
    .chassis_power_color = UI_CYAN,
    .online_wheel_left = 1,
    .online_wheel_right = 1,
    .online_hip1 = 1,
    .online_hip2 = 1,
    .online_hip3 = 1,
    .online_hip4 = 1,
    .online_referee = 1,
    .online_supercap = 1,
    .online_shoot = 1,
    .online_yaw = 1,
    .online_pitch = 1,
    .online_rub1 = 1,
    .online_rub2 = 1,
    .online_aim = 1,
    .heading_deg = 128,
    .attitude_pitch = 4,
    .attitude_roll = -6.5,
    .attitude_color = UI_CYAN,
    .heat_margin = 40,
    .heat_margin_color = UI_GREEN,
    .heat_limit = 40,
    .chassis_power_limit = 120,
    .field_data_valid = 0,
    .robot_data_valid = 0,
    .ally_control_degree = 0,
    .enemy_control_degree = 0,
    .control_delta = 0,
    .base_r_state = -1,
    .outpost_r_state = -1,
    .outpost_b_state = -1,
    .base_b_state = -1,
    .small_rune_state = -1,
    .large_rune_state = -1,
    .buffer_energy = 60,
    .chassis_output = -1,
    .shooter_output = -1,
    .rfid_contact = -1,
    .combat_state = -1,
    .gimbal_output = -1,
};

static void ui_monitor_text(char text[31], const char *normal, const char *first, uint16_t count)
{
    uint16_t pos = 8;
    char digits[5];
    uint16_t length = 0;
    if (!count) { strncpy(text, normal, 30); text[30] = '\0'; return; }
    memcpy(text, "OFFLINE ", 8);
    do { digits[length++] = (char)('0' + count % 10); count /= 10; } while (count);
    while (length) text[pos++] = digits[--length];
    text[pos++] = ':'; text[pos++] = ' ';
    strncpy(text + pos, first, 30 - pos); text[30] = '\0';
}

static double ui_map(double value, double in_min, double in_max, double out_min, double out_max)
{
    if (in_max <= in_min) return out_min;
    double ratio = (value - in_min) / (in_max - in_min);
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;
    return out_min + ratio * (out_max - out_min);
}

static uint16_t ui_pixel(double value, uint16_t maximum)
{
    if (value < 0.0) return 0;
    if (value > maximum) return maximum;
    return (uint16_t)(value + 0.5);
}

static uint16_t ui_pose_angle(double degrees)
{
    degrees = fmod(degrees, 360.0);
    if (degrees < 0.0) degrees += 360.0;
    return (uint16_t)(degrees + 0.5) % 360;
}

static void ui_pose_line(uint16_t id, const UI_Element *base, double cx, double cy,
                         double cosine, double sine, double shift, double width, double height, uint16_t stroke)
{
    const double u1 = base->x - cx, v1 = base->y - cy + shift;
    const double u2 = base->x2 - cx, v2 = base->y2 - cy + shift;
    const double x1 = u1 * cosine - v1 * sine, y1 = u1 * sine + v1 * cosine;
    const double x2 = u2 * cosine - v2 * sine, y2 = u2 * sine + v2 * cosine;
    const double dx = x2 - x1, dy = y2 - y1;
    const double p[4] = { -dx, dx, -dy, dy };
    const double q[4] = { x1 + width, width - x1, y1 + height, height - y1 };
    double low = 0.0, high = 1.0;
    uint8_t visible = 1;
    for (uint8_t i = 0; i < 4; ++i) {
        if (p[i] == 0.0) { if (q[i] < 0.0) visible = 0; }
        else {
            const double t = q[i] / p[i];
            if (p[i] < 0.0) { if (t > low) low = t; }
            else if (t < high) high = t;
        }
        if (low > high) visible = 0;
    }
    if (visible) UI_SetLine(id, ui_pixel(cx + x1 + low * dx, 2047), ui_pixel(cy + y1 + low * dy, 2047),
                           ui_pixel(cx + x1 + high * dx, 2047), ui_pixel(cy + y1 + high * dy, 2047));
    else UI_SetLine(id, base->x, base->y, base->x2, base->y2);
    UI_SetLineWidth(id, visible ? stroke : 0);
}

void UI_Update(const UI_Data *data)
{
    /* 模式状态 */
    {
        UI_SetText(UI_ID_MODE_LABEL, data->mode_text ? data->mode_text : "");
        UI_SetColor(UI_ID_MODE_LABEL, data->mode_color);
    }

    /* 状态边框 */
    {
        UI_SetColor(UI_ID_MODE_BORDER, data->mode_color);
    }

    /* hurt_direction */
    {
        UI_SetAngles(
            UI_ID_HURT_DIRECTION,
            ui_pixel(data->hurt_start, 511),
            ui_pixel(data->hurt_end, 511)
        );
        UI_SetVisible(UI_ID_HURT_DIRECTION, data->hurt_active);
    }

    /* target_lock */
    {
        UI_SetVisible(UI_ID_TARGET_LOCK, data->target_detected);
    }

    /* 电容电量条 */
    {
        const UI_Element *base = &UI_Initial[UI_ID_CAPACITOR_LEVEL];
        const double ratio = ui_map(data->capacitor_percent, 0, 100, 0.0, 1.0);
        const uint16_t span = (base->end_angle % 360 + 360 - base->start_angle % 360) % 360;
        const uint16_t filled = ui_pixel((span ? span : 360) * ratio, 360);
        UI_SetAngles(
            UI_ID_CAPACITOR_LEVEL,
            base->start_angle,
            (base->start_angle % 360 + filled) % 360
        );
        UI_SetLineWidth(UI_ID_CAPACITOR_LEVEL, filled ? base->line_width : 0);
        UI_SetColor(UI_ID_CAPACITOR_LEVEL, data->capacitor_color);
        UI_SetVisible(UI_ID_CAPACITOR_LEVEL, data->online_supercap);
    }

    /* 电容百分比 */
    {
        UI_SetNumber(UI_ID_CAPACITOR_PERCENT, data->capacitor_percent);
        UI_SetColor(UI_ID_CAPACITOR_PERCENT, data->capacitor_color);
        UI_SetVisible(UI_ID_CAPACITOR_PERCENT, data->online_supercap);
    }

    /* 热量余量 · level */
    {
        const UI_Element *base = &UI_Initial[UI_ID_HEAT_MARGIN_LEVEL];
        const double ratio = ui_map(data->heat_margin, 0, data->heat_limit, 0.0, 1.0);
        const uint16_t span = (base->end_angle % 360 + 360 - base->start_angle % 360) % 360;
        const uint16_t filled = ui_pixel((span ? span : 360) * ratio, 360);
        UI_SetAngles(
            UI_ID_HEAT_MARGIN_LEVEL,
            (base->end_angle % 360 + 360 - filled) % 360,
            base->end_angle
        );
        UI_SetLineWidth(UI_ID_HEAT_MARGIN_LEVEL, filled ? base->line_width : 0);
        UI_SetColor(UI_ID_HEAT_MARGIN_LEVEL, data->heat_margin_color);
        UI_SetVisible(UI_ID_HEAT_MARGIN_LEVEL, data->robot_data_valid);
    }

    /* 热量余量 · value */
    {
        UI_SetNumber(UI_ID_HEAT_MARGIN_VALUE, data->heat_margin);
        UI_SetColor(UI_ID_HEAT_MARGIN_VALUE, data->heat_margin_color);
        UI_SetVisible(UI_ID_HEAT_MARGIN_VALUE, data->robot_data_valid);
    }

    /* 剩余弹量 */
    {
        UI_SetNumber(UI_ID_AMMO_COUNT, data->ammo_count);
        UI_SetColor(UI_ID_AMMO_COUNT, data->ammo_count_color);
    }

    /* AMMO · level */
    {
        const UI_Element *base = &UI_Initial[UI_ID_AMMO_COUNT_LEVEL];
        const double ratio = ui_map(data->ammo_count, 0, 500, 0.0, 1.0);
        const uint16_t span = (base->end_angle % 360 + 360 - base->start_angle % 360) % 360;
        const uint16_t filled = ui_pixel((span ? span : 360) * ratio, 360);
        UI_SetAngles(
            UI_ID_AMMO_COUNT_LEVEL,
            (base->end_angle % 360 + 360 - filled) % 360,
            base->end_angle
        );
        UI_SetLineWidth(UI_ID_AMMO_COUNT_LEVEL, filled ? base->line_width : 0);
        UI_SetColor(UI_ID_AMMO_COUNT_LEVEL, data->ammo_count_color);
    }

    /* 底盘功率 */
    {
        UI_SetNumber(UI_ID_CHASSIS_POWER, data->chassis_power);
        UI_SetColor(UI_ID_CHASSIS_POWER, data->chassis_power_color);
    }

    /* POWER / W · level */
    {
        const UI_Element *base = &UI_Initial[UI_ID_CHASSIS_POWER_LEVEL];
        const double ratio = ui_map(data->chassis_power, 0, data->chassis_power_limit, 0.0, 1.0);
        const uint16_t span = (base->end_angle % 360 + 360 - base->start_angle % 360) % 360;
        const uint16_t filled = ui_pixel((span ? span : 360) * ratio, 360);
        UI_SetAngles(
            UI_ID_CHASSIS_POWER_LEVEL,
            base->start_angle,
            (base->start_angle % 360 + filled) % 360
        );
        UI_SetLineWidth(UI_ID_CHASSIS_POWER_LEVEL, filled ? base->line_width : 0);
        UI_SetColor(UI_ID_CHASSIS_POWER_LEVEL, data->chassis_power_color);
        UI_SetVisible(UI_ID_CHASSIS_POWER_LEVEL, data->robot_data_valid);
    }

    /* bearing_value */
    {
        UI_SetNumber(UI_ID_BEARING_VALUE, data->heading_deg);
    }

    /* attitude · rung_-1_-1 */
    {
        UI_SetColor(UI_ID_ATTITUDE_RUNG_1_1, data->attitude_color);
    }

    /* attitude · rung_-1_1 */
    {
        UI_SetColor(UI_ID_ATTITUDE_RUNG_1_1_2, data->attitude_color);
    }

    /* attitude · rung_0_-1 */
    {
        UI_SetColor(UI_ID_ATTITUDE_RUNG_0_1, data->attitude_color);
    }

    /* attitude · rung_0_1 */
    {
        UI_SetColor(UI_ID_ATTITUDE_RUNG_0_1_2, data->attitude_color);
    }

    /* attitude · rung_1_-1 */
    {
        UI_SetColor(UI_ID_ATTITUDE_RUNG_1_1_3, data->attitude_color);
    }

    /* attitude · rung_1_1 */
    {
        UI_SetColor(UI_ID_ATTITUDE_RUNG_1_1_4, data->attitude_color);
    }

    /* attitude · bank_pointer */
    {
        UI_SetColor(UI_ID_ATTITUDE_BANK_POINTER, data->attitude_color);
    }

    /* attitude · pitch_value */
    {
        UI_SetNumber(UI_ID_ATTITUDE_PITCH_VALUE, data->attitude_pitch);
        UI_SetColor(UI_ID_ATTITUDE_PITCH_VALUE, data->attitude_color);
    }

    /* 己方累计控制度 */
    {
        UI_SetNumber(UI_ID_CONTROL_ALLY, data->ally_control_degree);
        UI_SetVisible(UI_ID_CONTROL_ALLY, data->field_data_valid);
    }

    /* control_ally_unknown */
    {
        UI_SetVisible(UI_ID_CONTROL_ALLY_UNKNOWN, !data->field_data_valid);
    }

    /* 对方累计控制度 */
    {
        UI_SetNumber(UI_ID_CONTROL_ENEMY, data->enemy_control_degree);
        UI_SetVisible(UI_ID_CONTROL_ENEMY, data->field_data_valid);
    }

    /* control_enemy_unknown */
    {
        UI_SetVisible(UI_ID_CONTROL_ENEMY_UNKNOWN, !data->field_data_valid);
    }

    /* 累计控制度差值 */
    {
        UI_SetNumber(UI_ID_CONTROL_DELTA, data->control_delta);
        UI_SetVisible(UI_ID_CONTROL_DELTA, data->field_data_valid);
    }

    /* control_delta_unknown */
    {
        UI_SetVisible(UI_ID_CONTROL_DELTA_UNKNOWN, !data->field_data_valid);
    }

    /* 底盘缓冲能量 */
    {
        UI_SetNumber(UI_ID_BUFFER_ENERGY, data->buffer_energy);
        UI_SetVisible(UI_ID_BUFFER_ENERGY, data->robot_data_valid);
    }

    /* buffer_energy_unknown */
    {
        UI_SetVisible(UI_ID_BUFFER_ENERGY_UNKNOWN, !data->robot_data_valid);
    }

    /* 实时功率上限 */
    {
        UI_SetNumber(UI_ID_POWER_LIMIT, data->chassis_power_limit);
        UI_SetVisible(UI_ID_POWER_LIMIT, data->robot_data_valid);
    }

    /* power_limit_unknown */
    {
        UI_SetVisible(UI_ID_POWER_LIMIT_UNKNOWN, !data->robot_data_valid);
    }

    /* 等待数据 */
    {
        UI_SetVisible(UI_ID_HEAT_MARGIN_VALUE_UNKNOWN, !data->robot_data_valid);
    }

    /* 等待数据 */
    {
        UI_SetVisible(UI_ID_CAPACITOR_PERCENT_UNKNOWN, !data->online_supercap);
    }

    if (data->field_data_valid && data->base_r_state == 0) {
        UI_SetColor(UI_ID_BASE_R_STATE_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_BASE_R_STATE_VALUE, 8);
    }
    else if (data->field_data_valid && data->base_r_state == 1) {
        UI_SetColor(UI_ID_BASE_R_STATE_VALUE, UI_YELLOW);
        UI_SetLineWidth(UI_ID_BASE_R_STATE_VALUE, 2);
    }
    else if (data->field_data_valid && data->base_r_state == 2) {
        UI_SetColor(UI_ID_BASE_R_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_BASE_R_STATE_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_BASE_R_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_BASE_R_STATE_VALUE, 2);
    }

    if (data->field_data_valid && data->outpost_r_state == 0) {
        UI_SetColor(UI_ID_OUTPOST_R_STATE_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_OUTPOST_R_STATE_VALUE, 8);
    }
    else if (data->field_data_valid && data->outpost_r_state == 1) {
        UI_SetColor(UI_ID_OUTPOST_R_STATE_VALUE, UI_YELLOW);
        UI_SetLineWidth(UI_ID_OUTPOST_R_STATE_VALUE, 2);
    }
    else if (data->field_data_valid && data->outpost_r_state == 2) {
        UI_SetColor(UI_ID_OUTPOST_R_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_OUTPOST_R_STATE_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_OUTPOST_R_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_OUTPOST_R_STATE_VALUE, 2);
    }

    if (data->field_data_valid && data->outpost_b_state == 0) {
        UI_SetColor(UI_ID_OUTPOST_B_STATE_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_OUTPOST_B_STATE_VALUE, 8);
    }
    else if (data->field_data_valid && data->outpost_b_state == 1) {
        UI_SetColor(UI_ID_OUTPOST_B_STATE_VALUE, UI_YELLOW);
        UI_SetLineWidth(UI_ID_OUTPOST_B_STATE_VALUE, 2);
    }
    else if (data->field_data_valid && data->outpost_b_state == 2) {
        UI_SetColor(UI_ID_OUTPOST_B_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_OUTPOST_B_STATE_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_OUTPOST_B_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_OUTPOST_B_STATE_VALUE, 2);
    }

    if (data->field_data_valid && data->base_b_state == 0) {
        UI_SetColor(UI_ID_BASE_B_STATE_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_BASE_B_STATE_VALUE, 8);
    }
    else if (data->field_data_valid && data->base_b_state == 1) {
        UI_SetColor(UI_ID_BASE_B_STATE_VALUE, UI_YELLOW);
        UI_SetLineWidth(UI_ID_BASE_B_STATE_VALUE, 2);
    }
    else if (data->field_data_valid && data->base_b_state == 2) {
        UI_SetColor(UI_ID_BASE_B_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_BASE_B_STATE_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_BASE_B_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_BASE_B_STATE_VALUE, 2);
    }

    if (data->field_data_valid && data->small_rune_state == 0) {
        UI_SetColor(UI_ID_SMALL_RUNE_STATE_VALUE, UI_BLACK);
        UI_SetLineWidth(UI_ID_SMALL_RUNE_STATE_VALUE, 1);
    }
    else if (data->field_data_valid && data->small_rune_state == 1) {
        UI_SetColor(UI_ID_SMALL_RUNE_STATE_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_SMALL_RUNE_STATE_VALUE, 6);
    }
    else if (data->field_data_valid && data->small_rune_state == 2) {
        UI_SetColor(UI_ID_SMALL_RUNE_STATE_VALUE, UI_YELLOW);
        UI_SetLineWidth(UI_ID_SMALL_RUNE_STATE_VALUE, 6);
    }
    else {
        UI_SetColor(UI_ID_SMALL_RUNE_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_SMALL_RUNE_STATE_VALUE, 2);
    }

    if (data->field_data_valid && data->large_rune_state == 0) {
        UI_SetColor(UI_ID_LARGE_RUNE_STATE_VALUE, UI_BLACK);
        UI_SetLineWidth(UI_ID_LARGE_RUNE_STATE_VALUE, 1);
    }
    else if (data->field_data_valid && data->large_rune_state == 1) {
        UI_SetColor(UI_ID_LARGE_RUNE_STATE_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_LARGE_RUNE_STATE_VALUE, 6);
    }
    else if (data->field_data_valid && data->large_rune_state == 2) {
        UI_SetColor(UI_ID_LARGE_RUNE_STATE_VALUE, UI_YELLOW);
        UI_SetLineWidth(UI_ID_LARGE_RUNE_STATE_VALUE, 6);
    }
    else {
        UI_SetColor(UI_ID_LARGE_RUNE_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_LARGE_RUNE_STATE_VALUE, 2);
    }

    if (data->robot_data_valid && data->chassis_output == 0) {
        UI_SetColor(UI_ID_CHASSIS_OUTPUT_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_CHASSIS_OUTPUT_VALUE, 8);
    }
    else if (data->robot_data_valid && data->chassis_output == 1) {
        UI_SetColor(UI_ID_CHASSIS_OUTPUT_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_CHASSIS_OUTPUT_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_CHASSIS_OUTPUT_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_CHASSIS_OUTPUT_VALUE, 2);
    }

    if (data->robot_data_valid && data->shooter_output == 0) {
        UI_SetColor(UI_ID_SHOOTER_OUTPUT_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_SHOOTER_OUTPUT_VALUE, 8);
    }
    else if (data->robot_data_valid && data->shooter_output == 1) {
        UI_SetColor(UI_ID_SHOOTER_OUTPUT_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_SHOOTER_OUTPUT_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_SHOOTER_OUTPUT_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_SHOOTER_OUTPUT_VALUE, 2);
    }

    if (data->robot_data_valid && data->rfid_contact == 0) {
        UI_SetColor(UI_ID_RFID_CONTACT_VALUE, UI_BLACK);
        UI_SetLineWidth(UI_ID_RFID_CONTACT_VALUE, 1);
    }
    else if (data->robot_data_valid && data->rfid_contact == 1) {
        UI_SetColor(UI_ID_RFID_CONTACT_VALUE, UI_CYAN);
        UI_SetLineWidth(UI_ID_RFID_CONTACT_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_RFID_CONTACT_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_RFID_CONTACT_VALUE, 2);
    }

    if (data->robot_data_valid && data->combat_state == 0) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "");
    }
    else if (data->robot_data_valid && data->combat_state == 1) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "CHASSIS POWER OFF");
    }
    else if (data->robot_data_valid && data->combat_state == 2) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "SHOOTER POWER OFF");
    }
    else if (data->robot_data_valid && data->combat_state == 3) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_ORANGE);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "HEAT LIMIT");
    }
    else if (data->robot_data_valid && data->combat_state == 4) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_ORANGE);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "BUFFER LOW");
    }
    else if (data->robot_data_valid && data->combat_state == 5) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_ORANGE);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "AMMO LOW");
    }
    else if (data->robot_data_valid && data->combat_state == 6) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "VULNERABLE");
    }
    else if (data->robot_data_valid && data->combat_state == 7) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_ORANGE);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "BASE GUARD LOST");
    }
    else if (data->robot_data_valid && data->combat_state == 8) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "REFEREE LOST");
    }
    else if (data->robot_data_valid && data->combat_state == 9) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "RUNE ACTIVE");
    }
    else if (data->robot_data_valid && data->combat_state == 10) {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "GIMBAL POWER OFF");
    }
    else {
        UI_SetColor(UI_ID_COMBAT_STATE_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_COMBAT_STATE_VALUE, 1);
        UI_SetText(UI_ID_COMBAT_STATE_VALUE, "WAIT REFEREE");
    }

    if (data->robot_data_valid && data->gimbal_output == 0) {
        UI_SetColor(UI_ID_GIMBAL_OUTPUT_VALUE, UI_PINK);
        UI_SetLineWidth(UI_ID_GIMBAL_OUTPUT_VALUE, 8);
    }
    else if (data->robot_data_valid && data->gimbal_output == 1) {
        UI_SetColor(UI_ID_GIMBAL_OUTPUT_VALUE, UI_GREEN);
        UI_SetLineWidth(UI_ID_GIMBAL_OUTPUT_VALUE, 8);
    }
    else {
        UI_SetColor(UI_ID_GIMBAL_OUTPUT_VALUE, UI_WHITE);
        UI_SetLineWidth(UI_ID_GIMBAL_OUTPUT_VALUE, 2);
    }

    {
        const double roll = ui_map(data->attitude_roll, -60, 60, -60, 60) * 1;
        const double pitch = ui_map(data->attitude_pitch, -30, 30, -30, 30) * 1;
        const double angle = roll * 0.017453292519943295;
        const double cosine = cos(angle), sine = sin(angle);
        ui_pose_line(UI_ID_ATTITUDE_RUNG_1_1, &UI_Initial[UI_ID_ATTITUDE_RUNG_1_1], 960, 540, cosine, sine, pitch * 5, 130, 115, UI_Initial[UI_ID_ATTITUDE_RUNG_1_1].line_width);
        ui_pose_line(UI_ID_ATTITUDE_RUNG_1_1_2, &UI_Initial[UI_ID_ATTITUDE_RUNG_1_1_2], 960, 540, cosine, sine, pitch * 5, 130, 115, UI_Initial[UI_ID_ATTITUDE_RUNG_1_1_2].line_width);
        ui_pose_line(UI_ID_ATTITUDE_RUNG_0_1, &UI_Initial[UI_ID_ATTITUDE_RUNG_0_1], 960, 540, cosine, sine, pitch * 5, 130, 115, UI_Initial[UI_ID_ATTITUDE_RUNG_0_1].line_width);
        ui_pose_line(UI_ID_ATTITUDE_RUNG_0_1_2, &UI_Initial[UI_ID_ATTITUDE_RUNG_0_1_2], 960, 540, cosine, sine, pitch * 5, 130, 115, UI_Initial[UI_ID_ATTITUDE_RUNG_0_1_2].line_width);
        ui_pose_line(UI_ID_ATTITUDE_RUNG_1_1_3, &UI_Initial[UI_ID_ATTITUDE_RUNG_1_1_3], 960, 540, cosine, sine, pitch * 5, 130, 115, UI_Initial[UI_ID_ATTITUDE_RUNG_1_1_3].line_width);
        ui_pose_line(UI_ID_ATTITUDE_RUNG_1_1_4, &UI_Initial[UI_ID_ATTITUDE_RUNG_1_1_4], 960, 540, cosine, sine, pitch * 5, 130, 115, UI_Initial[UI_ID_ATTITUDE_RUNG_1_1_4].line_width);
        UI_SetAngles(UI_ID_ATTITUDE_BANK_POINTER, ui_pose_angle(roll - 2.0), ui_pose_angle(roll + 2.0));
    }

    {
        uint16_t offline = 0;
        const char *first = "";
        if (!data->online_wheel_left) { if (!offline) first = "WHEEL L"; ++offline; }
        if (!data->online_wheel_right) { if (!offline) first = "WHEEL R"; ++offline; }
        if (!data->online_hip1) { if (!offline) first = "HIP 1"; ++offline; }
        if (!data->online_hip2) { if (!offline) first = "HIP 2"; ++offline; }
        if (!data->online_hip3) { if (!offline) first = "HIP 3"; ++offline; }
        if (!data->online_hip4) { if (!offline) first = "HIP 4"; ++offline; }
        if (!data->online_referee) { if (!offline) first = "REFEREE"; ++offline; }
        if (!data->online_supercap) { if (!offline) first = "SUPERCAP"; ++offline; }
        if (!data->online_shoot) { if (!offline) first = "FEED"; ++offline; }
        if (!data->online_yaw) { if (!offline) first = "YAW"; ++offline; }
        if (!data->online_pitch) { if (!offline) first = "PITCH"; ++offline; }
        if (!data->online_rub1) { if (!offline) first = "FRIC 1"; ++offline; }
        if (!data->online_rub2) { if (!offline) first = "FRIC 2"; ++offline; }
        if (!data->online_aim) { if (!offline) first = "AIM"; ++offline; }
        char text[31];
        ui_monitor_text(text, "SYSTEM READY", first, offline);
        UI_SetColor(UI_ID_ONLINE_STATE, offline ? UI_ORANGE : UI_GREEN);
        UI_SetColor(UI_ID_ONLINE_SUMMARY, offline ? UI_ORANGE : UI_GREEN);
        UI_SetText(UI_ID_ONLINE_SUMMARY, text);
    }
}
