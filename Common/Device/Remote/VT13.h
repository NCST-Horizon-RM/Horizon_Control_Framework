/**
 * @file VT13.h
 * @brief RoboMaster 裁判系统相机图传模块 VT13 协议解析头文件
 * @version 1.0
 * @date 2026-03-21
 * @author CaoKangqi
 * * @copyright Copyright (c) 2026
 */

#ifndef HORIZON_VT13_H
#define HORIZON_VT13_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Offline_Detector.h"

/**
 * @brief 按键状态枚举
 */
typedef enum {
    VT13_KEY_UP    = 0,
    VT13_KEY_CLICK = 1,
    VT13_KEY_PRESS = 2
} VT13_KeyState_Env;

/**
 * @brief 应用层使用的解析结果结构体
 */
typedef struct {
    Offline_Check_t offline;
    int8_t Ctrl_Mode;               /**< 控制模式（键鼠/遥控） */
    bool CRC_flag;                  /**< CRC校验通过标志位 */

    /**
     * @brief 遥控器数据解算值
     */
    struct {
       int16_t Channel[4];          /**< 通道 0~3 归一化数据 (-1024 ~ 1024) */
       uint8_t mode_sw;             /**< 挡位开关 */
       uint8_t pause;               /**< 暂停键 */
       uint8_t fn_1;                /**< 自定义左键 */
       uint8_t fn_2;                /**< 自定义右键 */
       int16_t wheel;               /**< 拨轮归一化数据 (-1024 ~ 1024) */
       uint8_t trigger;             /**< 扳机键 */
    } Remote;

    /**
     * @brief 鼠标处理后的数据
     */
    struct {
       float X_Flt;                 /**< 滤波及死区后的 X 轴移动速度 */
       float Y_Flt;                 /**< 滤波及死区后的 Y 轴移动速度 */
       float Z_Flt;                 /**< 滤波及死区后的 Z 轴滚动速度 */
       uint8_t R_State;             /**< 右键状态 (点击/长按/释放) */
       uint8_t L_State;             /**< 左键状态 (点击/长按/释放) */
       uint8_t M_State;             /**< 中键状态 (点击/长按/释放) */
       uint8_t R_PressTime;        /**< 右键按下计时 */
       uint8_t L_PressTime;        /**< 左键按下计时 */
       uint8_t M_PressTime;        /**< 中键按下计时 */
    } Mouse;

    /**
     * @brief 键盘处理后的数据
     */
    struct {
       uint8_t W;                   /**< W 键复合状态 */
       uint8_t S;                   /**< S 键复合状态 */
       uint8_t A;                   /**< A 键复合状态 */
       uint8_t D;                   /**< D 键复合状态 */
       uint8_t Shift;               /**< Shift 键当前物理状态 */
       uint8_t Ctrl;                /**< Ctrl 键当前物理状态 */
       uint8_t Q;                   /**< Q 键当前物理状态 */
       uint8_t E;                   /**< E 键当前物理状态 */
       uint8_t R;                   /**< R 键当前物理状态 */
       uint8_t F;                   /**< F 键当前物理状态 */
       uint8_t G;                   /**< G 键当前物理状态 */
       uint8_t Z;                   /**< Z 键当前物理状态 */
       uint8_t X;                   /**< X 键当前物理状态 */
       uint8_t C;                   /**< C 键当前物理状态 */
       uint8_t V;                   /**< V 键当前物理状态 */
       uint8_t B;                   /**< B 键当前物理状态 */

       uint8_t W_PressTime;         /**< W 按压时间 */
       uint8_t S_PressTime;         /**< S 按压时间 */
       uint8_t A_PressTime;         /**< A 按压时间 */
       uint8_t D_PressTime;         /**< D 按压时间 */

       uint8_t Shift_PreeNumber;    /**< Shift 按键翻转计数器 */
       uint8_t Ctrl_PreeNumber;     /**< Ctrl 按键翻转计数器 */
       uint8_t Q_PreeNumber;        /**< Q 按键翻转计数器 */
       uint8_t E_PreeNumber;        /**< E 按键翻转计数器 */
       uint8_t R_PreeNumber;        /**< R 按键翻转计数器 */
       uint8_t F_PreeNumber;        /**< F 按键翻转计数器 */
       uint8_t G_PreeNumber;        /**< G 按键翻转计数器 */
       uint8_t Z_PreeNumber;        /**< Z 按键翻转计数器 */
       uint8_t X_PreeNumber;        /**< X 按键翻转计数器 */
       uint8_t C_PreeNumber;        /**< C 按键翻转计数器 */
       uint8_t V_PreeNumber;        /**< V 按键翻转计数器 */
       uint8_t B_PreeNumber;        /**< B 按键翻转计数器 */
    } KeyBoard;

} VT13_Typedef;

#pragma pack(push, 1)

/**
 * @brief VT13 原始数据帧 21 字节位域结构体定义
 */
typedef struct {
    // Byte 0 - 1
    uint8_t header1;            /**< 帧头1：固定值 0xA9 */
    uint8_t header2;            /**< 帧头2：固定值 0x53 */

    // Byte 2 - 9 (遥控器)
    uint16_t channel0    : 11;  /**< 通道 0：接收端右摇杆水平位置 */
    uint16_t channel1    : 11;  /**< 通道 1：接收端右摇杆竖直位置 */
    uint16_t channel2    : 11;  /**< 通道 2：接收端左摇杆竖直位置 */
    uint16_t channel3    : 11;  /**< 通道 3：接收端左摇杆水平位置 */
    uint16_t mode_sw     : 2;   /**< 挡位切换开关位置 (C:0, N:1, S:2) */
    uint16_t pause       : 1;   /**< 暂停按键是否按下 (0:未按下, 1:按下) */
    uint16_t fn_1        : 1;   /**< 自定义按键（左）是否按下 (0:未按下, 1:按下) */
    uint16_t fn_2        : 1;   /**< 自定义按键（右）是否按下 (0:未按下, 1:按下) */
    uint16_t wheel       : 11;  /**< 接收端拨轮位置 */
    uint16_t trigger     : 1;   /**< 接收端扳机键是否按下 (0:未按下, 1:按下) */
    uint16_t reserved1   : 3;   /**< 遥控器域保留填充位 */

    // Byte 10 - 15 (鼠标坐标轴)
    int16_t mouse_x;            /**< 鼠标 X 轴左右移动速度 */
    int16_t mouse_y;            /**< 鼠标 Y 轴前后移动速度 */
    int16_t mouse_z;            /**< 鼠标 Z 轴滚轮滚动速度 */

    // Byte 16 (鼠标按键)
    uint8_t mouse_l      : 2;   /**< 鼠标左键按下状态 */
    uint8_t mouse_r      : 2;   /**< 鼠标右键按下状态 */
    uint8_t mouse_m      : 2;   /**< 鼠标中键按下状态 */
    uint8_t reserved2    : 2;   /**< 鼠标按键保留填充位 */

    // Byte 17 - 18 (键盘按键)
    uint16_t key_w       : 1;   /**< W 键按下状态 */
    uint16_t key_s       : 1;   /**< S 键按下状态 */
    uint16_t key_a       : 1;   /**< A 键按下状态 */
    uint16_t key_d       : 1;   /**< D 键按下状态 */
    uint16_t key_shift   : 1;   /**< Shift 键按下状态 */
    uint16_t key_ctrl    : 1;   /**< Ctrl 键按下状态 */
    uint16_t key_q       : 1;   /**< Q 键按下状态 */
    uint16_t key_e       : 1;   /**< E 键按下状态 */
    uint16_t key_r       : 1;   /**< R 键按下状态 */
    uint16_t key_f       : 1;   /**< F 键按下状态 */
    uint16_t key_g       : 1;   /**< G 键按下状态 */
    uint16_t key_z       : 1;   /**< Z 键按下状态 */
    uint16_t key_x       : 1;   /**< X 键按下状态 */
    uint16_t key_c       : 1;   /**< C 键按下状态 */
    uint16_t key_v       : 1;   /**< V 键按下状态 */
    uint16_t key_b       : 1;   /**< B 键按下状态 */

    // Byte 19 - 20 (CRC)
    uint16_t crc16;             /**< CRC-16/CCITT-FALSE 校验和 */
} VT13_FrameTypeDef;

#pragma pack(pop)

/**
 * @brief VT13 联合体，方便接收缓冲区指针转换
 */
typedef union {
    uint8_t GetData[21];
    VT13_FrameTypeDef Frame;
} VT13_UNION_Typdef;

/**
 * @brief 解析入口函数
 * @param Data 21字节原始数据输入缓冲区指针
 * @param VT13 解析结果目标结构体指针
 */
void VT13_Resolved(uint8_t* Data, void *device_ptr, uint16_t size);

/**
 * @brief CRC16 查找表
 */
static const uint16_t crc16_tab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

#endif //HORIZON_VT13_H