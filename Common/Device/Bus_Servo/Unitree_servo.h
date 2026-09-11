#ifndef UNITREE_SERVO_H
#define UNITREE_SERVO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define J288_TX_SIZE 20U
#define J288_RX_SIZE 26U
#define J288_MAX_UNICAST_ID 14U
#define J288_RATIO (70070.0f / 243.0f)

typedef enum {
    J288_OK = 0,
    J288_INVALID_ARGUMENT,
    J288_LENGTH_ERROR,
    J288_HEADER_ERROR,
    J288_CRC_ERROR,
    J288_ID_MISMATCH
} J288_Result_t;

typedef struct {
    uint8_t id;                  /* 仅允许 0～14，拒绝广播。 */
    uint8_t status;              /* 仅允许 0 或 1，拒绝校准。 */
    uint8_t timeout_enable;
    float output_torque_nm;
    float output_speed_rad_s;
    float output_position_rad;
    float kp;
    float kd;
} J288_Command_t;

typedef struct {
    uint8_t valid;               /* 每次解析先清零，失败时保持无效。 */
    uint8_t id;
    uint8_t status;
    uint8_t timeout;
    int8_t temperature_c;
    uint8_t winding_temperature_c;
    float voltage_v;
    float output_torque_nm;
    float output_speed_rad_s;
    float output_position_rad;
    uint32_t merror;
    uint16_t outpos;
    uint8_t exflag;
    uint8_t exsensor2;
    uint8_t excom;
    float external_position_rad;
    uint32_t crc_received;
    uint32_t crc_calculated;
} J288_Feedback_t;

/* 官方 CRC：初值 FFFFFFFF，多项式 04C11DB7，无最终异或。
 * 按小端读取完整 32 位字，再从最高位处理；与官方
 * crc32_lookup_byte_by_byte() 一样忽略不足 4 字节的尾部。
 * 协议调用固定传入 16 或 20 字节；非零长度要求 data 非空。 */
uint32_t J288_CRC32(const uint8_t *data, size_t length);
/* 有限物理量按官方范围限幅，采用 STM32 例程的截断转换。
 * 成功只写入 20 字节，失败不修改输出；不将结构体直接作为线上数据。 */
J288_Result_t J288_BuildPacket(const J288_Command_t *command,
                             uint8_t *packet, size_t capacity);
/* 必须为 26 字节，CRC 覆盖字节 2～21，期望 ID 必须为单播。
 * 失败时仅可能填充原始 ID、模式、超时位及 CRC 诊断值。 */
J288_Result_t J288_ParseFeedback(const uint8_t *packet, size_t length,
                               uint8_t expected_id, J288_Feedback_t *feedback);

#ifdef __cplusplus
}
#endif
#endif
