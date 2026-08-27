//
// Created by CaoKangqi on 2026/6/26.
//

#ifndef HORIZON_EVENTBUS_H
#define HORIZON_EVENTBUS_H

#include <stdint.h>
#include <stdbool.h>

// 系统全局离散事件枚举
typedef enum {
    EVENT_MODE_CHANGED = 0,    // 模式切换事件 (附带 Global_Mode_e 参数)
    EVENT_ERROR_TRIGGERED,     // 模块错误事件 (附带 System_Error_Code_u 参数)
    EVENT_REMOTE_CONNECTION,   // 遥控器连接状态变更 (附带 bool 参数)
    EVENT_MAX_COUNT
} Event_ID_e;

// 回调函数指针类型
// event: 触发的事件ID; param: 附带的参数指针(需在回调中根据事件类型强转)
typedef void (*Event_Callback_t)(Event_ID_e event, void *param);

void EventBus_Init(void);
bool EventBus_Subscribe(Event_ID_e event, Event_Callback_t callback);
void EventBus_Publish(Event_ID_e event, void *param);

#endif //HORIZON_EVENTBUS_H
