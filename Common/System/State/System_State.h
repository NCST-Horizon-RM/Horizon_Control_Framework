#ifndef HORIZON_SYSTEM_STATE_H
#define HORIZON_SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include "Referee.h"

typedef enum {
    STATUS_INIT,
    STATUS_PREPARING,
    STATUS_RUN,
    STATUS_LOST,
    STATUS_ERROR
} App_Status_e;

typedef enum {
    ID_CHASSIS = 0,
    ID_GIMBAL,
    ID_SHOOT,
    ID_VISION,
    ID_IMU,
    MODULE_COUNT
} Module_ID_e;

typedef enum {
    GLOBAL_INIT_STAGE = 0,
    GLOBAL_MODULE_ERROR,   // 设备断联
    GLOBAL_STANDBY,        // 遥控断联
    GLOBAL_SAFE_LOCK,      // 软件/传感器故障
    GLOBAL_NORMAL_MATCH    // 正常运行
} Global_Mode_e;

typedef struct {
    App_Status_e Chassis;
    App_Status_e Gimbal;
    App_Status_e Shoot;
    App_Status_e Vision;
    App_Status_e IMU;
} App_Health_Table_t;

typedef union {
    struct {
        uint32_t imu_fault:1;
        uint32_t remote_lost:1;
        uint32_t chassis_offline:1;
        uint32_t gimbal_offline:1;
        uint32_t shoot_offline:1;
        uint32_t vision_lost:1;
        uint32_t referee_lost:1;
        uint32_t reserved:25;
    } bit;
    uint32_t all;
} System_Error_Code_u;

typedef struct {
    App_Health_Table_t  task_health;
    Global_Mode_e       global_mode;
    System_Error_Code_u error;
} System_State_t;

extern System_State_t sys_state;

void System_State_Init(void);
void System_State_Report(Module_ID_e id, App_Status_e status);
// 更新系统状态：ref 指向裁判系统数据（无裁判系统时传 NULL）
void System_State_Update(const Referee_Data_t *ref);
void System_State_Report_Remote(bool is_online);

#endif // HORIZON_SYSTEM_STATE_H