//
// Created by CaoKangqi on 2026/1/5.
//

#ifndef H7_FRAMEWORK_BSP_CAN_H
#define H7_FRAMEWORK_BSP_CAN_H

#include "fdcan.h"

typedef FDCAN_HandleTypeDef hcan_t;

// CAN接收统计数据结构
typedef struct
{
    uint32_t rx_count;          // 总接收消息数
    uint32_t fifo_full_count;   // FIFO满次数
    uint32_t msg_lost_count;    // 消息丢失次数
    uint32_t error_count;       // 读取错误次数
} CAN_Stats_t;

extern CAN_Stats_t can1_stats;
extern CAN_Stats_t can2_stats;
extern CAN_Stats_t can3_stats;

void CAN_App_Frame_Dispatch(hcan_t *hcan, uint32_t identifier, uint8_t *data, uint32_t len);

typedef void (*BSP_CAN_Callback_t)(void *device_ptr, uint8_t *data);

typedef struct {
    uint32_t id;
    void *device_ptr;
    BSP_CAN_Callback_t resolve;
    uint8_t is_valid;
} BSP_CAN_Hash_Node_t;

int BSP_CAN_Register_Slot(hcan_t *hcan, uint32_t id, void *device_ptr, BSP_CAN_Callback_t callback);

typedef struct {
    FDCAN_GlobalTypeDef *instance;
    uint32_t id;
    void *device_ptr;
    BSP_CAN_Callback_t resolve;
} Auto_CAN_Reg_t;

#ifndef MACRO_CONCAT
#define _MACRO_CONCAT_IMPL(a, b) a##b
#define MACRO_CONCAT(a, b) _MACRO_CONCAT_IMPL(a, b)
#endif

/* --- CAN 自动注册节点 --- */
#define CAN_RX_NODE(instance_ptr, rx_id, dev_ptr_arg, callback) \
__attribute__((used, section("CAN_Reg_Sec"))) \
static const Auto_CAN_Reg_t MACRO_CONCAT(_can_reg_, __LINE__) = { \
.instance = instance_ptr, \
.id = rx_id, \
.device_ptr = dev_ptr_arg, \
.resolve = callback \
}
void BSP_CAN_Auto_Init(void);

void CAN_Config(hcan_t *hcan, uint32_t fifo);
extern uint8_t CAN_Send_Msg(hcan_t *hcan, uint32_t id, uint8_t *data, uint32_t len);
extern uint32_t CAN_GetTxFreeLevel(hcan_t *hcan);

#endif //H7_FRAMEWORK_BSP_CAN_H