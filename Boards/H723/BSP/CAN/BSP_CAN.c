#include "BSP_CAN.h"
#include "BSP_DWT.h"

/**
 * @brief FDCAN外设配置函数
 * @param hcan CAN句柄
 * @param fifo   选择接收FIFO（FDCAN_RX_FIFO0或FDCAN_RX_FIFO1）
 * @note 该函数完成以下配置：
 *       0. 重置外设：在配置前先进行去初始化和重新初始化
 *       1. 配置过滤器：默认接收所有标准帧到指定的 FIFO
 *       2. 全局过滤：拒绝远程帧，拒绝不匹配的扩展帧
 *       3. 开启中断：根据传入的 FIFO 开启对应的中断源（含溢出、丢失等）
 *       4. 启动外设
 */
void CAN_Config(hcan_t *hcan, uint32_t fifo)
{
    // 重置FDCAN外设：去初始化后重新初始化，确保外设处于干净状态
    if (HAL_FDCAN_DeInit(hcan) != HAL_OK) Error_Handler();
    // 配置FDCAN模式：上层统一走经典 CAN（RoboMaster 全链路经典帧，FD 暂不使用）
    hcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    if (HAL_FDCAN_Init(hcan) != HAL_OK) Error_Handler();

    FDCAN_FilterTypeDef sFilterConfig = {0};
    // 过滤器配置：仅使用标准帧过滤器，接收所有ID
    sFilterConfig.IdType = FDCAN_STANDARD_ID; // 仅配置标准帧过滤器
    sFilterConfig.FilterIndex = 0; // 过滤器索引，0表示第一个过滤器
    sFilterConfig.FilterType = FDCAN_FILTER_MASK; // 掩码模式：FilterID1与FilterID2配合使用，0表示接收所有ID
    sFilterConfig.FilterConfig = (fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1; // 根据参数选择接收FIFO
    sFilterConfig.FilterID1 = 0x000; // 标识符掩码为0表示接收所有ID
    sFilterConfig.FilterID2 = 0x000; // 0表示掩码模式下的ID掩码，配合FilterID1使用

    if (HAL_FDCAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) Error_Handler();

    // 全局过滤配置：拒绝远程帧，拒绝不匹配的扩展帧，允许所有标准帧
    if (HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) Error_Handler();

    // 根据选择的 FIFO 开启对应的中断源
    uint32_t it_source = FDCAN_IT_BUS_OFF |
                        FDCAN_IT_ARB_PROTOCOL_ERROR |
                        FDCAN_IT_DATA_PROTOCOL_ERROR;
    if (fifo == FDCAN_RX_FIFO0)
    {
        it_source |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                    FDCAN_IT_RX_FIFO0_FULL |
                    FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
    }
    else
    {
        it_source |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                    FDCAN_IT_RX_FIFO1_FULL |
                    FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
    }
    if (HAL_FDCAN_ActivateNotification(hcan, it_source, 0) != HAL_OK) Error_Handler();

    if (HAL_FDCAN_Start(hcan) != HAL_OK) Error_Handler();
}

/**
 * @brief 将字节长度转换为 FDCAN DLC 值
 * @param len 数据长度（字节）
 * @return 对应的 DLC 值
 */
uint32_t Bytes_To_DLC(uint32_t len) {
    switch (len) {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:  return FDCAN_DLC_BYTES_8;
            // FDCAN模式下的长字节
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_8; // 默认给8，防止越界
    }
}

/**
 * @brief 将 FDCAN 硬件的 DLC 宏转换为实际的整数长度
 */
uint8_t DLC_To_Bytes(uint32_t dlc) {
    // 针对不同芯片系列，宏定义的具体数值不同，但逻辑是一致的
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        // 如果是 CAN FD 模式，还需要处理以下部分
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 8; // 默认容错处理
    }
}

/**
 * @brief FDCAN发送通用函数
 * @param hcan FDCAN句柄
 * @param id     消息ID（标准帧）
 * @param data   数据指针
 * @param len    数据长度（字节）
 * @return 0表示成功，1表示失败
 */
uint8_t CAN_Send_Msg(hcan_t *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader = {
        .Identifier = id, // 标准帧ID
        .IdType = FDCAN_STANDARD_ID, // 标准帧
        .TxFrameType = FDCAN_DATA_FRAME, // 数据帧
        .DataLength = Bytes_To_DLC(len), // 根据长度转换为DLC
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE, // 错误状态指示器：主动状态
        .BitRateSwitch = (hcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) ? FDCAN_BRS_ON : FDCAN_BRS_OFF, // 如果是FD模式，开启BRS
        .FDFormat = (hcan->Init.FrameFormat == FDCAN_FRAME_FD_BRS) ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN, // 自动切换 FD 模式
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS, // 不使用事件FIFO
        .MessageMarker = 0 // 消息标记，用户自定义用途
    };
    return (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) == HAL_OK) ? 0 : 1;
}

uint32_t CAN_GetTxFreeLevel(hcan_t *hcan)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(hcan);
}

/**
 * @brief FDCAN 错误回调函数
 * @param hcan 发生错误的 FDCAN 句柄
 */
void HAL_FDCAN_ErrorCallback(hcan_t *hcan)
{
    FDCAN_ProtocolStatusTypeDef protocolStatus;
    // 获取协议状态
    HAL_FDCAN_GetProtocolStatus(hcan, &protocolStatus);
    // 检查是否进入 Bus-Off 状态
    if (protocolStatus.BusOff == 1)
    {
        static uint32_t last_recovery_time[3] = {0};
        uint32_t now = HAL_GetTick();

        uint8_t idx = 0;
        uint32_t target_fifo = FDCAN_RX_FIFO0;
        // 识别当前 FDCAN 实例
        if (hcan->Instance == FDCAN1)
        {
            idx = 0;
            target_fifo = FDCAN_RX_FIFO0;
        }
        else if (hcan->Instance == FDCAN2)
        {
            idx = 1;
            target_fifo = FDCAN_RX_FIFO1;
        }
        else if (hcan->Instance == FDCAN3)
        {
            idx = 2;
            target_fifo = FDCAN_RX_FIFO0;
        }
        else
        {
            return; // 未知实例直接返回，防止误操作
        }
        // 100ms
        if ((now - last_recovery_time[idx]) > 100)
        {
            last_recovery_time[idx] = now;
            HAL_FDCAN_Stop(hcan);
            if (HAL_FDCAN_Start(hcan) == HAL_OK)
            {
                uint32_t it_source = FDCAN_IT_BUS_OFF |
                                     FDCAN_IT_ARB_PROTOCOL_ERROR |
                                     FDCAN_IT_DATA_PROTOCOL_ERROR;
                if (target_fifo == FDCAN_RX_FIFO0)
                {
                    it_source |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_RX_FIFO0_FULL |
                                 FDCAN_IT_RX_FIFO0_MESSAGE_LOST;
                }
                else // FDCAN_RX_FIFO1
                {
                    it_source |= FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                                 FDCAN_IT_RX_FIFO1_FULL |
                                 FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
                }
                HAL_FDCAN_ActivateNotification(hcan, it_source, 0);
            }
        }
    }
}

extern const Auto_CAN_Reg_t __start_CAN_Reg_Sec;
extern const Auto_CAN_Reg_t __stop_CAN_Reg_Sec;

void BSP_CAN_Auto_Init(void)
{
    const Auto_CAN_Reg_t *node = &__start_CAN_Reg_Sec;
    for (; node < &__stop_CAN_Reg_Sec; node++)
    {
        hcan_t temp_hcan = { .Instance = node->instance };
        BSP_CAN_Register_Slot(&temp_hcan, node->id, node->device_ptr, node->resolve);
    }
}

#define CAN_HASH_SIZE       16
#define CAN_HASH_MASK       (CAN_HASH_SIZE - 1)
#define CAN_BUS_NUM         3

// 哈希表
static BSP_CAN_Hash_Node_t BSP_Hash_Table[CAN_BUS_NUM][CAN_HASH_SIZE] = {0};

static inline int8_t Get_CAN_Bus_Index(hcan_t *hcan)
{
    if (hcan->Instance == FDCAN1) return 0;
    if (hcan->Instance == FDCAN2) return 1;
    if (hcan->Instance == FDCAN3) return 2;
    return -1;   // 未知实例
}

/**
 * @brief 动态注册槽位函数：供应用层初始化时调用，用于填充哈希表
 */
int BSP_CAN_Register_Slot(hcan_t *hcan, uint32_t id, void *device_ptr, BSP_CAN_Callback_t callback)
{
    uint8_t bus_idx = Get_CAN_Bus_Index(hcan);
    if (bus_idx >= CAN_BUS_NUM) return -1;

    uint32_t hash_idx = id & CAN_HASH_MASK;
    uint32_t start_idx = hash_idx;
    // 线性探测：跳过已占用且 ID 不同的槽位
    while (BSP_Hash_Table[bus_idx][hash_idx].is_valid == 1 &&
           BSP_Hash_Table[bus_idx][hash_idx].id != id)
    {
        hash_idx = (hash_idx + 1) & CAN_HASH_MASK;
        if (hash_idx == start_idx) return -2;   // 表满
    }
    // 此时 hash_idx 指向空槽或相同 ID 的槽，直接写入（覆盖旧值）
    BSP_Hash_Table[bus_idx][hash_idx].id = id;
    BSP_Hash_Table[bus_idx][hash_idx].device_ptr = device_ptr;
    BSP_Hash_Table[bus_idx][hash_idx].resolve = callback;
    BSP_Hash_Table[bus_idx][hash_idx].is_valid = 1;
    return 0;
}

/**
 * @brief O(1) 级中断分发
 */
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
            if (BSP_Hash_Table[bus_idx][hash_idx].resolve != NULL) {
                BSP_Hash_Table[bus_idx][hash_idx].resolve(BSP_Hash_Table[bus_idx][hash_idx].device_ptr, data);
            }
            return;
        }
        hash_idx = (hash_idx + 1) & CAN_HASH_MASK;
        if (hash_idx == start_idx) return;   // 已遍历一圈，未找到
    }
}

CAN_Stats_t can1_stats;
CAN_Stats_t can2_stats;
CAN_Stats_t can3_stats;


static inline void FDCAN_Rx_FIFO_Process(hcan_t *hcan, uint32_t fifo, uint32_t its, CAN_Stats_t *stats)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    uint32_t fill_level;
    // 统计 FIFO 状态
    if (stats) {
        uint32_t full_it = (fifo == FDCAN_RX_FIFO0) ? FDCAN_IT_RX_FIFO0_FULL : FDCAN_IT_RX_FIFO1_FULL;
        uint32_t lost_it = (fifo == FDCAN_RX_FIFO0) ? FDCAN_IT_RX_FIFO0_MESSAGE_LOST : FDCAN_IT_RX_FIFO1_MESSAGE_LOST;
        if (its & full_it)  stats->fifo_full_count++;
        if (its & lost_it)  stats->msg_lost_count++;
    }
    // 循环读取 FIFO 确保不丢帧
    while ((fill_level = HAL_FDCAN_GetRxFifoFillLevel(hcan, fifo)) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hcan, fifo, &rx_header, rx_data) != HAL_OK) {
            if (stats) stats->error_count++;
            break;
        }
        if (stats) stats->rx_count++;
        CAN_App_Frame_Dispatch(hcan, rx_header.Identifier, rx_data, DLC_To_Bytes(rx_header.DataLength));
        if (fill_level > 64) break;
    }
}

void HAL_FDCAN_RxFifo0Callback(hcan_t *hcan, uint32_t RxFifo0ITs)
{
    CAN_Stats_t *stats = (hcan->Instance == FDCAN1) ? &can1_stats : ((hcan->Instance == FDCAN3) ? &can3_stats : NULL);
    FDCAN_Rx_FIFO_Process(hcan, FDCAN_RX_FIFO0, RxFifo0ITs, stats);
}

void HAL_FDCAN_RxFifo1Callback(hcan_t *hcan, uint32_t RxFifo1ITs)
{
    CAN_Stats_t *stats = (hcan->Instance == FDCAN2) ? &can2_stats : NULL;
    FDCAN_Rx_FIFO_Process(hcan, FDCAN_RX_FIFO1, RxFifo1ITs, stats);
}