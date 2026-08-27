//
// Created by CaoKangqi on 2026/6/26.
//
#include "System_Indicator.h"
#include "System_State.h"
#include "EventBus.h"
#include "Buzzer.h"
#include "LED.h"
#include <string.h>

#define MAX_FRAGMENTS 32

#define RGB_OFF       0,   0,   0
#define RGB_RED       255, 0,   0
#define RGB_GREEN     0,   255, 0
#define RGB_BLUE      0,   0,   255
#define RGB_YELLOW    255, 200, 0
#define RGB_CYAN      0,   200, 200

typedef struct {
    uint16_t freq;
    uint16_t duration;
    uint8_t r, g, b;
} Step_t;

typedef struct {
    Step_t steps[MAX_FRAGMENTS];
    uint8_t total;
} Flow_t;

// 固化乐谱定义
static const Flow_t Flow_Init = {.steps = {
    {880, 170,  RGB_RED}, {0, 50,  RGB_RED},
    {1100, 170, RGB_BLUE}, {0, 50, RGB_BLUE},
    {1320, 300, RGB_GREEN}}, .total = 5
};
static const Flow_t Flow_Hint = {.steps = {
    {2500, 120, RGB_GREEN}, {0, 20, RGB_OFF},
    {3000, 180, RGB_GREEN}}, .total = 3
};
static const Flow_t Flow_Lost = {.steps = {
        {1500, 200, RGB_BLUE}, {0, 50, RGB_OFF},
        {1500, 200, RGB_BLUE}, {0, 550, RGB_OFF}},
        .total = 4
    };
static const Flow_t Flow_Remote_Recover = {.steps = {
    {2500, 150, RGB_BLUE}, {0, 40, RGB_OFF},
    {3000, 150, RGB_GREEN}}, .total = 3
};

// 动态错误报警流
static Flow_t Flow_Dynamic_Error;

static struct {
    bool     is_running;
    uint32_t timer_ms;
    Step_t   steps[MAX_FRAGMENTS];
    uint8_t  total;
    uint8_t  idx;
} ctrl;

static const Flow_t *playing_flow = NULL;

static void Safe_Buzzer_Set(uint16_t freq) {
    if (sys_state.task_health.IMU == STATUS_PREPARING && sys_state.global_mode != GLOBAL_INIT_STAGE) {
        Buzzer_Off();
    } else {
        Buzzer_Set_Freq(freq);
    }
}

static void Action_Push(const Flow_t *flow) {
    if (!flow || flow->total == 0) return;
    if (ctrl.is_running && playing_flow == flow) return;

    playing_flow = flow;
    memcpy(ctrl.steps, flow->steps, flow->total * sizeof(Step_t));
    ctrl.total = flow->total;
    ctrl.idx = 0;
    ctrl.is_running = true;
    ctrl.timer_ms = ctrl.steps[0].duration;

    Safe_Buzzer_Set(ctrl.steps[0].freq);
    LED_SetMode_Static(ctrl.steps[0].r, ctrl.steps[0].g, ctrl.steps[0].b);
}

#define FIXED_ALARM_SLOT_MS 1000
#define BEEP_DUR            80
#define SHORT_SILENCE       80
// 动态报警生成器
static void Build_And_Play_Module_Error(System_Error_Code_u err) {
    Flow_Dynamic_Error.total = 0;
    uint8_t offline_counts[MODULE_COUNT] = {0};
    if (err.bit.chassis_offline) offline_counts[ID_CHASSIS] = ID_CHASSIS + 1; // 1 下
    if (err.bit.gimbal_offline)  offline_counts[ID_GIMBAL]  = ID_GIMBAL  + 1; // 2 下
    if (err.bit.shoot_offline)   offline_counts[ID_SHOOT]   = ID_SHOOT   + 1; // 3 下
    if (err.bit.vision_lost)     offline_counts[ID_VISION]  = ID_VISION  + 1; // 4 下
    if (err.bit.imu_fault)       offline_counts[ID_IMU]     = ID_IMU     + 1; // 5 下
    for (int i = 0; i < MODULE_COUNT; i++) {
        uint8_t beeps = offline_counts[i];
        if (beeps > 0) {
            uint32_t active_time = (beeps - 1) * (BEEP_DUR + SHORT_SILENCE) + BEEP_DUR;
            uint16_t pad_time = 0;
            if (FIXED_ALARM_SLOT_MS > active_time) {
                pad_time = FIXED_ALARM_SLOT_MS - active_time;
            } else {
                pad_time = SHORT_SILENCE; // 极限兜底保护
            }
            for (int b = 0; b < beeps; b++) {
                if (Flow_Dynamic_Error.total < MAX_FRAGMENTS - 1) {
                    Flow_Dynamic_Error.steps[Flow_Dynamic_Error.total++] = (Step_t){3000, BEEP_DUR, RGB_RED};
                    if (b == beeps - 1) {
                        Flow_Dynamic_Error.steps[Flow_Dynamic_Error.total++] = (Step_t){0, pad_time, RGB_OFF};
                    } else {
                        Flow_Dynamic_Error.steps[Flow_Dynamic_Error.total++] = (Step_t){0, SHORT_SILENCE, RGB_OFF};
                    }
                }
            }
        }
    }
    if (Flow_Dynamic_Error.total == 0) {
        Flow_Dynamic_Error.steps[0] = (Step_t){3000, 500, RGB_YELLOW};
        Flow_Dynamic_Error.steps[1] = (Step_t){0, FIXED_ALARM_SLOT_MS - 500, RGB_OFF};
        Flow_Dynamic_Error.total = 2;
    }
    Action_Push(&Flow_Dynamic_Error);
}

// --- 事件监听回调 ---

static void On_Mode_Changed(Event_ID_e event, void *param) {
    Global_Mode_e new_mode = *(Global_Mode_e *)param;

    switch(new_mode) {
        case GLOBAL_INIT_STAGE:
            Action_Push(&Flow_Init);
            break;
        case GLOBAL_MODULE_ERROR:
        case GLOBAL_SAFE_LOCK:
            Build_And_Play_Module_Error(sys_state.error);
            break;
        case GLOBAL_STANDBY:
            Action_Push(&Flow_Lost);
            break;
        case GLOBAL_NORMAL_MATCH:
            if (!ctrl.is_running) Action_Push(&Flow_Hint);
            LED_SetMode_Breathing(RGB_GREEN, 3.0f);
            break;
        default: break;
    }
}

static void On_Remote_Connection(Event_ID_e event, void *param) {
    bool is_online = *(bool *)param;
    if (is_online) {
        Action_Push(&Flow_Remote_Recover);
    }
}


bool System_Indicator_Is_Playing(void) {
    return ctrl.is_running;
}

void System_Indicator_Init(void) {
    memset(&ctrl, 0, sizeof(ctrl));

    EventBus_Subscribe(EVENT_MODE_CHANGED, On_Mode_Changed);
    EventBus_Subscribe(EVENT_REMOTE_CONNECTION, On_Remote_Connection);
}

void System_Indicator_Ticks(void) {
    if (sys_state.task_health.IMU == STATUS_PREPARING && sys_state.global_mode != GLOBAL_INIT_STAGE) {
        Buzzer_Off();
    }

    if (!ctrl.is_running && sys_state.global_mode == GLOBAL_INIT_STAGE) {
        LED_SetPixel(RGB_OFF);
    }

    if (!ctrl.is_running) return;

    if (ctrl.timer_ms > 0) {
        ctrl.timer_ms--;
        return;
    }

    if (++ctrl.idx < ctrl.total) {
        ctrl.timer_ms = ctrl.steps[ctrl.idx].duration;
        Safe_Buzzer_Set(ctrl.steps[ctrl.idx].freq);
        LED_SetMode_Static(ctrl.steps[ctrl.idx].r, ctrl.steps[ctrl.idx].g, ctrl.steps[ctrl.idx].b);
    } else {
        ctrl.is_running = false;
        playing_flow = NULL;
        Buzzer_Off();

        if (sys_state.global_mode == GLOBAL_MODULE_ERROR || sys_state.global_mode == GLOBAL_SAFE_LOCK) {
            Build_And_Play_Module_Error(sys_state.error);
        }
        else if (sys_state.global_mode == GLOBAL_STANDBY) {
            Action_Push(&Flow_Lost);
        }
        else if (sys_state.global_mode == GLOBAL_NORMAL_MATCH) {
            LED_SetMode_Breathing(RGB_GREEN, 3.0f);
        } else {
            LED_SetMode_Static(RGB_OFF);
        }
    }
}