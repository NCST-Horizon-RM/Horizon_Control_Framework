//
// Created by qza on 2026/6/16.
//

#ifndef HORIZON_OFFLINE_DETECTOR_H
#define HORIZON_OFFLINE_DETECTOR_H

#include <stdint.h>
#include <stdbool.h>

// 分组枚举
typedef enum {
    GROUP_NONE = 0,
    CHASSIS,
    GIMBAL,
    SHOOT,
    GROUP_ALL
} Device_Group_e;

// 离线检测基础结构体 (必须被嵌套在各个传感器/电机的数据结构体内)
typedef struct {
    uint32_t last_feed_tick;
    bool is_online;
} Offline_Check_t;

typedef struct {
    Offline_Check_t *node;
    uint32_t timeout_ms;
    Device_Group_e group;
} Auto_Offline_Reg_t;

#ifndef MACRO_CONCAT
#define _MACRO_CONCAT_IMPL(a, b) a##b
#define MACRO_CONCAT(a, b) _MACRO_CONCAT_IMPL(a, b)
#endif

#define OFFLINE_NODE(dev_node_ptr, timeout, group_name) \
__attribute__((used, section("Offline_Reg_Sec"))) \
static const Auto_Offline_Reg_t MACRO_CONCAT(_offline_reg_, __LINE__) = { \
.node = dev_node_ptr, \
.timeout_ms = timeout, \
.group = group_name \
}
void Offline_Monitor(void);
bool Is_Group_Online(Device_Group_e group);

#endif //HORIZON_OFFLINE_DETECTOR_H
