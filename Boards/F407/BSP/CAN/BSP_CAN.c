//
// Created by CaoKangqi on 2026/1/5.
//
#include "BSP_CAN.h"
#include "BSP_DWT.h"

CAN_Stats_t can1_stats;
CAN_Stats_t can2_stats;

/* 哈希表 */
#define CAN_HASH_SIZE       16
#define CAN_HASH_MASK       (CAN_HASH_SIZE - 1)
#define CAN_BUS_NUM         2

static BSP_CAN_Hash_Node_t BSP_Hash_Table[CAN_BUS_NUM][CAN_HASH_SIZE] = {0};

static inline uint8_t Get_CAN_Bus_Index(hcan_t *hcan)
{
    if (hcan->Instance == CAN1) return 0;
    if (hcan->Instance == CAN2) return 1;
    return 0;
}

static inline CAN_Stats_t* Get_CAN_Stats(hcan_t *hcan)
{
    if (hcan->Instance == CAN1) return &can1_stats;
    if (hcan->Instance == CAN2) return &can2_stats;
    return NULL;
}

/**
 * @brief CAN外设配置函数
 */
void CAN_Config(hcan_t *hcan, uint32_t fifo)
{
    if (HAL_CAN_DeInit(hcan) != HAL_OK) Error_Handler();
    if (HAL_CAN_Init(hcan) != HAL_OK) Error_Handler();

    CAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.SlaveStartFilterBank = 14;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;

    // 根据传入参数决定挂载到 FIFO0 还是 FIFO1
    sFilterConfig.FilterFIFOAssignment = (fifo == CAN_RX_FIFO0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1;
    sFilterConfig.FilterActivation     = ENABLE;

    // 分配过滤器组：CAN1 用 0，CAN2 用 14
    if (hcan->Instance == CAN1) {
        sFilterConfig.FilterBank = 0;
    } else if (hcan->Instance == CAN2) {
        sFilterConfig.FilterBank = 14;
    } else {
        Error_Handler();
    }

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) Error_Handler();

    if (HAL_CAN_Start(hcan) != HAL_OK) Error_Handler();

    uint32_t active_its = CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                          CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR;

    if (fifo == CAN_RX_FIFO0) {
        active_its |= (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN);
    } else {
        active_its |= (CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN);
    }

    if (HAL_CAN_ActivateNotification(hcan, active_its) != HAL_OK) Error_Handler();
}

/**
 * @brief 长度与DLC转换
 */
uint32_t Bytes_To_DLC(uint32_t len) { return (len > 8) ? 8 : len; }
uint8_t DLC_To_Bytes(uint32_t dlc)  { return (dlc > 8) ? 8 : (uint8_t)dlc; }

/**
 * @brief CAN发送通用函数
 */
uint8_t CAN_Send_Msg(hcan_t *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
    CAN_TxHeaderTypeDef TxHeader = {
        .DLC = Bytes_To_DLC(len),
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .StdId = id,
        .ExtId = 0,
        .TransmitGlobalTime = DISABLE
    };
    uint32_t txMailbox;
    return (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &txMailbox) == HAL_OK) ? 0 : 1;
}

uint32_t CAN_GetTxFreeLevel(hcan_t *hcan)
{
    return HAL_CAN_GetTxMailboxesFreeLevel(hcan);
}

/**
 * @brief 动态注册槽位与分发
 */
int BSP_CAN_Register_Slot(hcan_t *hcan, uint32_t id, void *device_ptr, BSP_CAN_Callback_t callback)
{
    uint8_t bus_idx = Get_CAN_Bus_Index(hcan);
    if (bus_idx >= CAN_BUS_NUM) return -1;

    uint32_t hash_idx = id & CAN_HASH_MASK;
    uint32_t start_idx = hash_idx;
    while (BSP_Hash_Table[bus_idx][hash_idx].is_valid == 1 &&
           BSP_Hash_Table[bus_idx][hash_idx].id != id)
    {
        hash_idx = (hash_idx + 1) & CAN_HASH_MASK;
        if (hash_idx == start_idx) return -2; // 表满
    }
    // 写入（覆盖或新增）
    BSP_Hash_Table[bus_idx][hash_idx].id = id;
    BSP_Hash_Table[bus_idx][hash_idx].device_ptr = device_ptr;
    BSP_Hash_Table[bus_idx][hash_idx].resolve = callback;
    BSP_Hash_Table[bus_idx][hash_idx].is_valid = 1;
    return 0;
}

void CAN_App_Frame_Dispatch(hcan_t *hcan, uint32_t identifier, uint8_t *data, uint32_t len)
{
    (void)len;
    uint8_t bus_idx = Get_CAN_Bus_Index(hcan);
    if (bus_idx >= CAN_BUS_NUM) return;

    uint32_t hash_idx = identifier & CAN_HASH_MASK;
    uint32_t start_idx = hash_idx;
    while (BSP_Hash_Table[bus_idx][hash_idx].is_valid == 1)
    {
        if (BSP_Hash_Table[bus_idx][hash_idx].id == identifier)
        {
            if (BSP_Hash_Table[bus_idx][hash_idx].resolve != NULL)
                BSP_Hash_Table[bus_idx][hash_idx].resolve(BSP_Hash_Table[bus_idx][hash_idx].device_ptr, data);
            return;
        }
        hash_idx = (hash_idx + 1) & CAN_HASH_MASK;
        if (hash_idx == start_idx) return;
    }
}

extern const Auto_CAN_Reg_t __start_CAN_Reg_Sec;
extern const Auto_CAN_Reg_t __stop_CAN_Reg_Sec;

void BSP_CAN_Auto_Init(void)
{
    const Auto_CAN_Reg_t *node = &__start_CAN_Reg_Sec;
    for (; node < &__stop_CAN_Reg_Sec; node++) {
        hcan_t temp_hcan = { .Instance = node->instance };
        BSP_CAN_Register_Slot(&temp_hcan, node->id, node->device_ptr, node->resolve);
    }
}

/* ================= 接收与中断处理 ================= */

static inline void CAN_Rx_FIFO_Process(hcan_t *hcan, uint32_t fifo, CAN_Stats_t *stats)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_header, rx_data) == HAL_OK) {
            if (stats) stats->rx_count++;
            if ((rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA)) {
                CAN_App_Frame_Dispatch(hcan, rx_header.StdId, rx_data, DLC_To_Bytes(rx_header.DLC));
            }
        } else {
            if (stats) stats->error_count++;
            break;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t *hcan) {
    CAN_Rx_FIFO_Process(hcan, CAN_RX_FIFO0, Get_CAN_Stats(hcan));
}

void HAL_CAN_RxFifo1MsgPendingCallback(hcan_t *hcan) {
    CAN_Rx_FIFO_Process(hcan, CAN_RX_FIFO1, Get_CAN_Stats(hcan));
}

/* bxCAN 独立的中断回调，用于统计状态 */
void HAL_CAN_RxFifo0FullCallback(hcan_t *hcan) {
    CAN_Stats_t *stats = Get_CAN_Stats(hcan);
    if (stats) stats->fifo_full_count++;
}
void HAL_CAN_RxFifo1FullCallback(hcan_t *hcan) {
    CAN_Stats_t *stats = Get_CAN_Stats(hcan);
    if (stats) stats->fifo_full_count++;
}

void HAL_CAN_RxFifo0OverrunCallback(hcan_t *hcan) {
    CAN_Stats_t *stats = Get_CAN_Stats(hcan);
    if (stats) stats->msg_lost_count++;
}

void HAL_CAN_RxFifo1OverrunCallback(hcan_t *hcan) {
    CAN_Stats_t *stats = Get_CAN_Stats(hcan);
    if (stats) stats->msg_lost_count++;
}

/**
 * @brief CAN错误回调函数
 */
void HAL_CAN_ErrorCallback(hcan_t *hcan)
{
    uint32_t error_code = HAL_CAN_GetError(hcan);
    CAN_Stats_t *stats = Get_CAN_Stats(hcan);

    if (error_code & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_RX_FOV1)) {
        if (stats) stats->msg_lost_count++;
    }

    // Bus-Off 恢复逻辑
    if (error_code & HAL_CAN_ERROR_BOF)
    {
        static uint32_t last_recovery_time[CAN_BUS_NUM] = {0};
        uint32_t now = HAL_GetTick();
        uint8_t idx = Get_CAN_Bus_Index(hcan);

        if ((now - last_recovery_time[idx]) > 100) {
            last_recovery_time[idx] = now;
            HAL_CAN_Stop(hcan);
            if (HAL_CAN_Start(hcan) == HAL_OK) {
                // 重新激活所有的中断
                uint32_t active_its = CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
                                      CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR |
                                      CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN |
                                      CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN;
                HAL_CAN_ActivateNotification(hcan, active_its);
            }
        }
    }
}
