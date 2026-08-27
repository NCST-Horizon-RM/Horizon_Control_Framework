#include "Referee.h"
#include <stdbool.h>

static void Referee_System_Info_Update(uint16_t cmd_id, uint8_t *data_ptr, Referee_Data_t *user_data);

void Referee_System_Frame_Update(uint8_t *Buff, void *device_ptr, uint16_t Size)
{
    Referee_Data_t*Referee = (Referee_Data_t*)device_ptr;
    uint16_t i = 0;
    uint16_t data_length = 0;
    uint16_t cmd_id = 0;
    uint8_t *data_ptr;

    // 静态变量：用于保存上一次数据中的断包
    static uint8_t remain_buf[REFEREE_MAX_PACKET_SIZE];
    static uint16_t remain_len = 0; // 记录断包的实际长度

    if (Buff == NULL || Size == 0) return;

    // 处理上一次遗留下来的断包
    if (remain_len > 0)
    {
        // 计算新缓冲区需要借多少字节过来才能凑成一包，此时遗留包的 remain_buf 里已经包含了原本的帧头，我们可以直接读出它需要的 data_length
        if (remain_len >= 3) // 至少要拿到长度字段
        {
            data_length = (uint16_t)(remain_buf[2] << 8 | remain_buf[1])
                        + FrameHeader_Length + CMDID_Length + CRC16_Length;

            uint16_t need_len = data_length - remain_len;
            // 如果新缓冲区的数据足够填满这个断包
            if (Size >= need_len && data_length <= REFEREE_MAX_PACKET_SIZE)
            {
                // 把新缓冲区的头部拼接到遗留缓存的尾部
                memcpy(&remain_buf[remain_len], Buff, need_len);
                // 校验这个拼接完成的临时完整包
                if (Verify_CRC8_Check_Sum(remain_buf, FrameHeader_Length) == true &&
                    Verify_CRC16_Check_Sum(remain_buf, data_length) == true)
                {
                    cmd_id = (uint16_t)(remain_buf[FrameHeader_Length + 1] << 8 | remain_buf[FrameHeader_Length]);
                    data_ptr = &remain_buf[FrameHeader_Length + CMDID_Length];
                    Referee_System_Info_Update(cmd_id, data_ptr, Referee);
                }
                // 拼接包处理完毕，新缓冲区的指针需要向后跳过被借走的 need_len 字节
                i = need_len;
            }
            else
            {
                // 如果新来的数据太短，连剩下的断包都填不满
                if (remain_len + Size < REFEREE_MAX_PACKET_SIZE) {
                    memcpy(&remain_buf[remain_len], Buff, Size);
                    remain_len += Size;
                } else {
                    remain_len = 0; // 异常超长，直接丢弃
                }
                return; // 新数据用完了，直接退出
            }
        }
        remain_len = 0; // 清空暂存标记，准备开始常规遍历
    }
    while (i < Size)
    {
        if (Buff[i] == 0xA5)
        {
            // 检查剩余长度是否够一个帧头，不够的话说明帧头本身被截断了
            if (i + FrameHeader_Length > Size)
            {
                remain_len = Size - i;
                memcpy(remain_buf, &Buff[i], remain_len);
                break;
            }
            // CRC8 校验帧头
            if (Verify_CRC8_Check_Sum(&Buff[i], FrameHeader_Length) == true)
            {
                data_length = (uint16_t)(Buff[i+2] << 8 | Buff[i+1])
                            + FrameHeader_Length + CMDID_Length + CRC16_Length;
                // 检查整帧是否超出了当前缓冲区的边界
                if (i + data_length > Size)
                {
                    // 把这一段断包暂存到 remain_buf 中，留到下一次回调拼接
                    remain_len = Size - i;
                    if (remain_len <= REFEREE_MAX_PACKET_SIZE) {
                        memcpy(remain_buf, &Buff[i], remain_len);
                    } else {
                        remain_len = 0;
                    }
                    break; // 已经到缓冲区末尾了，直接退出循环
                }
                // 如果没越界，正常进行整帧 CRC16 校验
                if (Verify_CRC16_Check_Sum(&Buff[i], data_length) == true)
                {
                    cmd_id = (uint16_t)(Buff[i + FrameHeader_Length + 1] << 8 | Buff[i + FrameHeader_Length]);
                    data_ptr = &Buff[i + FrameHeader_Length + CMDID_Length];

                    Referee_System_Info_Update(cmd_id, data_ptr, Referee);

                    i += data_length; // 成功跳过整帧
                    continue;
                }
            }
        }
        i++;
    }
}

static void Referee_System_Info_Update(uint16_t cmd_id, uint8_t *data_ptr, Referee_Data_t *user_data)
{
    user_data->offline.last_feed_tick = HAL_GetTick();
    switch (cmd_id)
    {
        case game_state:
            memcpy(&user_data->game_status, data_ptr, sizeof(game_status_t));
            break;

        case Match_results:
            memcpy(&user_data->game_result, data_ptr, sizeof(game_result_t));
            break;

        case Robot_HP:
            memcpy(&user_data->game_robot_HP, data_ptr, sizeof(game_robot_HP_t));
            break;

        case Venue_Events:
            memcpy(&user_data->event_data, data_ptr, sizeof(event_data_t));
            break;

        case Referee_warning:
            memcpy(&user_data->referee_warning, data_ptr, sizeof(referee_warning_t));
            break;

        case Dart_fire:
            memcpy(&user_data->dart_info, data_ptr, sizeof(dart_info_t));
            break;

        case Robot_performan:
            memcpy(&user_data->robot_status, data_ptr, sizeof(robot_status_t));
            break;

        case time_power:
            memcpy(&user_data->power_heat_data, data_ptr, sizeof(power_heat_data_t));
            break;

        case Robot_location:
            memcpy(&user_data->robot_pos, data_ptr, sizeof(robot_pos_t));
            break;

        case Robot_buff:
            memcpy(&user_data->buff, data_ptr, sizeof(buff_t));
            break;

        case Damage_status:
            memcpy(&user_data->hurt_data, data_ptr, sizeof(hurt_data_t));
            break;

        case time_shooting:
            memcpy(&user_data->shoot_data, data_ptr, sizeof(shoot_data_t));
            break;

        case Allowable_ammo:
            memcpy(&user_data->projectile_allowance, data_ptr, sizeof(projectile_allowance_t));
            break;

        case RFID_status:
            memcpy(&user_data->rfid_status, data_ptr, sizeof(rfid_status_t));
            break;

        case Dart_directives:
            memcpy(&user_data->dart_client_cmd, data_ptr, sizeof(dart_client_cmd_t));
            break;

        case Ground_location:
            memcpy(&user_data->ground_robot_position, data_ptr, sizeof(ground_robot_position_t));
            break;

        case Radar_Marking:
            memcpy(&user_data->radar_mark_data, data_ptr, sizeof(radar_mark_data_t));
            break;

        case Route_Informat:
            memcpy(&user_data->sentry_info, data_ptr, sizeof(sentry_info_t));
            break;

        case Radar_Informat:
            memcpy(&user_data->radar_info, data_ptr, sizeof(radar_info_t));
            break;

        case Minimap:
            memcpy(&user_data->map_command, data_ptr, sizeof(map_command_t));
            break;

        default:
            break;
    }
}


static uint8_t referee_tx_seq = 0;

/**
  * @brief  裁判系统通用发送函数
  * @param  cmd_id: 命令码
  * @param  p_data: 负载数据指针
  * @param  len: 负载数据长度
  */
void Referee_Send_Data(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    static uint8_t tx_buf[REFEREE_RXFRAME_LENGTH]; // 发送缓冲区
    uint16_t frame_length = FrameHeader_Length + CMDID_Length + len + CRC16_Length;

    tx_buf[0] = 0xA5;                                  // SOF
    tx_buf[1] = (uint8_t)(len & 0x00FF);               // Data Length LSB
    tx_buf[2] = (uint8_t)((len >> 8) & 0x00FF);        // Data Length MSB
    tx_buf[3] = referee_tx_seq++;                      // Seq
    Append_CRC8_Check_Sum(tx_buf, FrameHeader_Length); // 计算并添加 CRC8

    tx_buf[5] = (uint8_t)(cmd_id & 0x00FF);
    tx_buf[6] = (uint8_t)((cmd_id >> 8) & 0x00FF);

    memcpy(&tx_buf[7], p_data, len);

    Append_CRC16_Check_Sum(tx_buf, frame_length);

    //HAL_UART_Transmit(&huart1, tx_buf, frame_length, HAL_MAX_DELAY);
}