//
// Created by qza on 2026/6/16.
//
#include "Offline_Detector.h"
#include "hal.h"

extern const Auto_Offline_Reg_t __start_Offline_Reg_Sec __attribute__((weak));
extern const Auto_Offline_Reg_t __stop_Offline_Reg_Sec __attribute__((weak));

void Offline_Monitor(void)
{
    if (&__start_Offline_Reg_Sec == &__stop_Offline_Reg_Sec) {
        return;
    }
    uint32_t now = HAL_GetTick();
    const Auto_Offline_Reg_t *reg = &__start_Offline_Reg_Sec;

    for (; reg < &__stop_Offline_Reg_Sec; reg++)
    {
        Offline_Check_t *dev = reg->node;
        if (dev == NULL || reg->timeout_ms == 0) continue;

        if ((now - dev->last_feed_tick) > reg->timeout_ms) {
            dev->is_online = false;
        } else {
            dev->is_online = true;
        }
    }
}

bool Is_Group_Online(Device_Group_e group)
{
    if (&__start_Offline_Reg_Sec == &__stop_Offline_Reg_Sec) {
        return true;
    }
    const Auto_Offline_Reg_t *reg = &__start_Offline_Reg_Sec;
    for (; reg < &__stop_Offline_Reg_Sec; reg++)
    {
        if (group == GROUP_ALL || reg->group == group)
        {
            Offline_Check_t *dev = reg->node;
            if (dev != NULL && dev->is_online == false) {
                return false;
            }
        }
    }
    return true;
}