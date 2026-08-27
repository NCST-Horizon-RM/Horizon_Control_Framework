//
// Created by CaoKangqi on 2026/6/26.
//
#include "EventBus.h"
#include <stddef.h>
#include "cmsis_os2.h"

#define MAX_SUBSCRIBERS_PER_EVENT 4

typedef struct {
    Event_Callback_t callbacks[MAX_SUBSCRIBERS_PER_EVENT];
    uint8_t count;
} Event_Node_t;

static Event_Node_t g_event_bus[EVENT_MAX_COUNT];
static osMutexId_t  g_event_mutex = NULL;

void EventBus_Init(void) {
    for (int i = 0; i < EVENT_MAX_COUNT; i++) {
        g_event_bus[i].count = 0;
        for (int j = 0; j < MAX_SUBSCRIBERS_PER_EVENT; j++) {
            g_event_bus[i].callbacks[j] = NULL;
        }
    }
    if (g_event_mutex == NULL) {
        g_event_mutex = osMutexNew(NULL);
    }
}

bool EventBus_Subscribe(Event_ID_e event, Event_Callback_t callback) {
    if (event >= EVENT_MAX_COUNT || callback == NULL) return false;

    bool ret = false;
    if (g_event_mutex) osMutexAcquire(g_event_mutex, osWaitForever);

    Event_Node_t *node = &g_event_bus[event];

    // 查重
    for (int i = 0; i < node->count; i++) {
        if (node->callbacks[i] == callback) {
            ret = true;
            goto unlock;
        }
    }
    // 插入
    if (node->count < MAX_SUBSCRIBERS_PER_EVENT) {
        node->callbacks[node->count++] = callback;
        ret = true;
    }

    unlock:
        if (g_event_mutex) osMutexRelease(g_event_mutex);
    return ret;
}

void EventBus_Publish(Event_ID_e event, void *param) {
    if (event >= EVENT_MAX_COUNT) return;

    // 持锁仅做快照，回调在锁外执行，避免回调内再 Publish/Subscribe 造成死锁
    Event_Callback_t snapshot[MAX_SUBSCRIBERS_PER_EVENT];
    uint8_t n = 0;

    if (g_event_mutex) osMutexAcquire(g_event_mutex, osWaitForever);
    Event_Node_t *node = &g_event_bus[event];
    for (int i = 0; i < node->count && n < MAX_SUBSCRIBERS_PER_EVENT; i++) {
        if (node->callbacks[i]) {
            snapshot[n++] = node->callbacks[i];
        }
    }
    if (g_event_mutex) osMutexRelease(g_event_mutex);

    for (int i = 0; i < n; i++) {
        snapshot[i](event, param);
    }
}