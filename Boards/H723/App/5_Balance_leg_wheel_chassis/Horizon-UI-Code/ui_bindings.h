/* 自动生成的数据接口；UI_DataDefault 只读，业务使用独立的 UI_Data。 */
#ifndef UI_BINDINGS_H
#define UI_BINDINGS_H
#include "ui_layout.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *mode_text;
    UI_Color mode_color;
    double hurt_start;
    double hurt_end;
    uint8_t hurt_active;
    uint8_t target_detected;
    double capacitor_percent;
    UI_Color capacitor_color;
    double ammo_count;
    UI_Color ammo_count_color;
    double chassis_power;
    UI_Color chassis_power_color;
    uint8_t online_wheel_left;
    uint8_t online_wheel_right;
    uint8_t online_hip1;
    uint8_t online_hip2;
    uint8_t online_hip3;
    uint8_t online_hip4;
    uint8_t online_referee;
    uint8_t online_supercap;
    uint8_t online_shoot;
    uint8_t online_yaw;
    uint8_t online_pitch;
    uint8_t online_rub1;
    uint8_t online_rub2;
    uint8_t online_aim;
    double heading_deg;
    double attitude_pitch;
    double attitude_roll;
    UI_Color attitude_color;
    double heat_margin;
    UI_Color heat_margin_color;
    double heat_limit;
    double chassis_power_limit;
    uint8_t field_data_valid;
    uint8_t robot_data_valid;
    double ally_control_degree;
    double enemy_control_degree;
    double control_delta;
    double base_r_state;
    double outpost_r_state;
    double outpost_b_state;
    double base_b_state;
    double small_rune_state;
    double large_rune_state;
    double buffer_energy;
    double chassis_output;
    double shooter_output;
    double rfid_contact;
    double combat_state;
    double gimbal_output;
} UI_Data;

extern const UI_Data UI_DataDefault;
void UI_Update(const UI_Data *data);

#ifdef __cplusplus
}
#endif
#endif
