#include "System_State.h"
#include "hal.h"
#include <string.h>
#include "Referee.h"
#include "EventBus.h"

System_State_t sys_state;

static struct {
    bool ref_online;
    bool any_ref_pwr;
    bool chassis_pwr, chassis_grace;
    bool gimbal_pwr,  gimbal_grace;
    bool shoot_pwr,   shoot_grace;
} pwr_info;

static uint32_t state_timer = 0;
static System_Error_Code_u last_error = {0};
static bool g_remote_is_online = false;

static void Update_Power_Status(uint32_t now, Referee_Data_t *ref);
static bool Check_Boot_Sequence(uint32_t now);
static void Update_Error_Flags(bool remote_is_online, bool in_boot_grace_period);
static void Arbitrate_Global_Mode(uint32_t now);

static bool Is_All_Tasks_Running(void) {
    return (sys_state.task_health.Chassis == STATUS_RUN &&
            sys_state.task_health.Gimbal  == STATUS_RUN &&
            sys_state.task_health.Shoot   == STATUS_RUN);
}

void System_State_Report_Remote(bool is_online) {
    g_remote_is_online = is_online;
}

void System_State_Init(void) {
    memset(&sys_state, 0, sizeof(sys_state));
    sys_state.global_mode = GLOBAL_INIT_STAGE;

    // 发布初始状态事件
    EventBus_Publish(EVENT_MODE_CHANGED, &sys_state.global_mode);
}

void System_State_Report(Module_ID_e id, App_Status_e status) {
    if (id < MODULE_COUNT) {
        App_Status_e *health_array = (App_Status_e *)&sys_state.task_health;
        health_array[id] = status;
    }
}

void System_State_Update(const Referee_Data_t *ref) {
    uint32_t now = HAL_GetTick();
    Referee_Data_t local_ref = {0};

    if (ref) {
        local_ref = *ref;   // 快照，避免读裁判系统的途中被 UART ISR 改写
    }

    Update_Power_Status(now, &local_ref);
    bool in_boot_grace_period = !Check_Boot_Sequence(now);

    Update_Error_Flags(g_remote_is_online, in_boot_grace_period);
    Arbitrate_Global_Mode(now);
}

static void Update_Power_Status(uint32_t now, Referee_Data_t *ref) {
    pwr_info.ref_online = ref->offline.is_online;

    pwr_info.chassis_pwr = pwr_info.ref_online ? ref->robot_status.power_management_chassis_output : true;
    pwr_info.gimbal_pwr  = pwr_info.ref_online ? ref->robot_status.power_management_gimbal_output  : true;
    pwr_info.shoot_pwr   = pwr_info.ref_online ? ref->robot_status.power_management_shooter_output : true;

    static bool last_c = false, last_g = false, last_s = false;
    static uint32_t tick_c = 0, tick_g = 0, tick_s = 0;

    if (pwr_info.chassis_pwr && !last_c) tick_c = now;
    if (pwr_info.gimbal_pwr  && !last_g) tick_g = now;
    if (pwr_info.shoot_pwr   && !last_s) tick_s = now;

    last_c = pwr_info.chassis_pwr;
    last_g = pwr_info.gimbal_pwr;
    last_s = pwr_info.shoot_pwr;

    pwr_info.chassis_grace = pwr_info.chassis_pwr && ((now - tick_c) < 1800);
    pwr_info.gimbal_grace  = pwr_info.gimbal_pwr  && ((now - tick_g) < 1800);
    pwr_info.shoot_grace   = pwr_info.shoot_pwr   && ((now - tick_s) < 1800);

    pwr_info.any_ref_pwr = pwr_info.ref_online &&
                           (ref->robot_status.power_management_chassis_output ||
                            ref->robot_status.power_management_gimbal_output ||
                            ref->robot_status.power_management_shooter_output);
}

static bool Check_Boot_Sequence(uint32_t now) {
    static bool init_done = false;
    if (init_done) return true;

    // 依赖查询UI播放状态，保证开机音乐播完前不切状态
    if (now < 1500) {
        return false;
    }

    if (now < 21800 && !Is_All_Tasks_Running() && !pwr_info.any_ref_pwr) {
        return false;
    }

    init_done = true;
    return true;
}

static void Update_Error_Flags(bool remote_is_online, bool in_boot_grace_period) {
    sys_state.error.bit.remote_lost  = !remote_is_online;
    sys_state.error.bit.referee_lost = !pwr_info.ref_online;
    sys_state.error.bit.imu_fault    = sys_state.task_health.IMU == STATUS_ERROR;

    if (Is_All_Tasks_Running() || in_boot_grace_period) {
        sys_state.error.bit.chassis_offline = 0;
        sys_state.error.bit.gimbal_offline  = 0;
        sys_state.error.bit.shoot_offline   = 0;
    } else {
        bool chassis_fault = (sys_state.task_health.Chassis == STATUS_LOST || sys_state.task_health.Chassis == STATUS_ERROR);
        bool gimbal_fault  = (sys_state.task_health.Gimbal  == STATUS_LOST || sys_state.task_health.Gimbal  == STATUS_ERROR);
        bool shoot_fault   = (sys_state.task_health.Shoot   == STATUS_LOST || sys_state.task_health.Shoot   == STATUS_ERROR);

        sys_state.error.bit.chassis_offline = chassis_fault && pwr_info.chassis_pwr && !pwr_info.chassis_grace;
        sys_state.error.bit.gimbal_offline  = gimbal_fault  && pwr_info.gimbal_pwr  && !pwr_info.gimbal_grace;
        sys_state.error.bit.shoot_offline   = shoot_fault   && pwr_info.shoot_pwr   && !pwr_info.shoot_grace;
    }
}

static void Arbitrate_Global_Mode(uint32_t now) {
    if (now < 1500) {
        return;
    }

    Global_Mode_e next_mode = GLOBAL_NORMAL_MATCH;

    if (sys_state.error.bit.chassis_offline || sys_state.error.bit.gimbal_offline || sys_state.error.bit.shoot_offline) {
        next_mode = GLOBAL_MODULE_ERROR;
    } else if (sys_state.error.bit.remote_lost) {
        next_mode = GLOBAL_STANDBY;
    } else if (sys_state.error.bit.imu_fault) {
        next_mode = GLOBAL_SAFE_LOCK;
    }

    // 状态切换时发布事件
    if (next_mode != sys_state.global_mode) {
        sys_state.global_mode = next_mode;
        EventBus_Publish(EVENT_MODE_CHANGED, &sys_state.global_mode);
        state_timer = now;
    } else {
        // 遥控器恢复逻辑
        bool remote_recovered = (last_error.bit.remote_lost == 1) && (sys_state.error.bit.remote_lost == 0);
        if (remote_recovered && sys_state.global_mode != GLOBAL_NORMAL_MATCH) {
            bool is_recovered = true;
            EventBus_Publish(EVENT_REMOTE_CONNECTION, &is_recovered);
            state_timer = now;
        }
    }
    // 如果错误码有变动，发布错误更新事件
    if (sys_state.error.all != last_error.all) {
        EventBus_Publish(EVENT_ERROR_TRIGGERED, &sys_state.error);
    }
    last_error.all = sys_state.error.all;
}