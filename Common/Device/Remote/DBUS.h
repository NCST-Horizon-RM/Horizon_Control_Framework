/**
 * @file DBUS.h
 * @brief  DBUS 协议解析头文件
 * @version 1.0
 * @date 2026-01-19
 * @author CaoKangqi
 */

#ifndef HORIZON_DBUS_H
#define HORIZON_DBUS_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "Offline_Detector.h"

/**
 * @brief 按键复合状态枚举
 */
typedef enum {
    DBUS_KEY_UP    = 0,  /**< 按键未按下 */
    DBUS_KEY_CLICK = 1,  /**< 按键点按（单击） */
    DBUS_KEY_PRESS = 2   /**< 按键长按 */
} KeyState_Env;

/**
 * @brief 应用层使用的解析结果结构体
 */
typedef struct {
    Offline_Check_t offline;
    int8_t Ctrl_Mode;

    /**
     * @brief 遥控器数据解算
     */
    struct {
        int16_t CH0;                /**< 右摇杆水平通道 (-660 ~ 660) */
        int16_t CH1;                /**< 右摇杆竖直通道 (-660 ~ 660) */
        int16_t CH2;                /**< 左摇杆竖直通道 (-660 ~ 660) */
        int16_t CH3;                /**< 左摇杆水平通道 (-660 ~ 660) */
        int16_t Dial;               /**< 左侧拨轮通道数据 (-660 ~ 660) */
        uint8_t S1;                 /**< 右侧开关状态 */
        uint8_t S2;                 /**< 左侧开关状态 */
    } Remote;

    /**
     * @brief 鼠标数据解算与滤波
     */
    struct {
        float X_Flt;                /**< 滤波处理后的鼠标 X 轴数据 */
        float Y_Flt;                /**< 滤波处理后的鼠标 Y 轴数据 */
        float Z_Flt;                /**< 滤波处理后的鼠标 Z 轴数据 */
        uint8_t L_State;            /**< 左键复合状态 (点击/长按/释放) */
        uint8_t R_State;            /**< 右键复合状态 (点击/长按/释放) */
        uint8_t L_PressTime;        /**< 左键持续按下时间计时 */
        uint8_t R_PressTime;        /**< 右键持续按下时间计时 */
    } Mouse;

    /**
     * @brief 键盘数据解算与锁定翻转
     */
    struct {
        uint8_t W;                  /**< W 键复合状态 */
        uint8_t S;                  /**< S 键复合状态 */
        uint8_t A;                  /**< A 键复合状态 */
        uint8_t D;                  /**< D 键复合状态 */
        uint8_t Shift;              /**< Shift 键当前物理状态 */
        uint8_t Ctrl;               /**< Ctrl 键当前物理状态 */
        uint8_t Q;                  /**< Q 键当前物理状态 */
        uint8_t E;                  /**< E 键当前物理状态 */
        uint8_t R;                  /**< R 键当前物理状态 */
        uint8_t F;                  /**< F 键当前物理状态 */
        uint8_t G;                  /**< G 键当前物理状态 */
        uint8_t Z;                  /**< Z 键当前物理状态 */
        uint8_t X;                  /**< X 键当前物理状态 */
        uint8_t C;                  /**< C 键当前物理状态 */
        uint8_t V;                  /**< V 键当前物理状态 */
        uint8_t B;                  /**< B 键当前物理状态 */

        uint8_t W_PressTime;         /**< W 按压时间计数 */
        uint8_t S_PressTime;         /**< S 按压时间计数 */
        uint8_t A_PressTime;         /**< A 按压时间计数 */
        uint8_t D_PressTime;         /**< D 按压时间计数 */

        uint8_t Shift_PreeNumber;    /**< Shift 按键锁定翻转状态 (0或1) */
        uint8_t Ctrl_PreeNumber;     /**< Ctrl 按键锁定翻转状态 (0或1) */
        uint8_t Q_PreeNumber;        /**< Q 按键锁定翻转状态 (0或1) */
        uint8_t E_PreeNumber;        /**< E 按键锁定翻转状态 (0或1) */
        uint8_t R_PreeNumber;        /**< R 按键锁定翻转状态 (0或1) */
        uint8_t F_PreeNumber;        /**< F 按键锁定翻转状态 (0或1) */
        uint8_t G_PreeNumber;        /**< G 按键锁定翻转状态 (0或1) */
        uint8_t Z_PreeNumber;        /**< Z 按键锁定翻转状态 (0或1) */
        uint8_t X_PreeNumber;        /**< X 按键锁定翻转状态 (0或1) */
        uint8_t C_PreeNumber;        /**< C 按键锁定翻转状态 (0或1) */
        uint8_t V_PreeNumber;        /**< V 按键锁定翻转状态 (0或1) */
        uint8_t B_PreeNumber;        /**< B 按键锁定翻转状态 (0或1) */
    } KeyBoard;
} DBUS_Typedef;

#pragma pack(push, 1)

/**
 * @brief DBUS 原始数据帧 18 字节位域结构体定义
 * @note 严格对应 DR16 接收机输出的 18 字节传统 DBUS 数据流
 */
typedef struct {
    // 遥控器通道与开关数据 (Byte 0 - 5)
    uint64_t channel0        : 11;  /**< 通道 0：右摇杆左右 (中点1024) */
    uint64_t channel1        : 11;  /**< 通道 1：右摇杆上下 (中点1024) */
    uint64_t channel2        : 11;  /**< 通道 2：左摇杆上下 (中点1024) */
    uint64_t channel3        : 11;  /**< 通道 3：左摇杆左右 (中点1024) */
    uint64_t switch_right    : 2;   /**< 右侧开关 S1 (1:上, 3:中, 2:下) */
    uint64_t switch_left     : 2;   /**< 左侧开关 S2 (1:上, 3:中, 2:下) */

    // 鼠标轴向移动与按键数据 (Byte 6 - 13)
    int16_t mouse_x;                /**< 鼠标 X 轴移动速度 */
    int16_t mouse_y;                /**< 鼠标 Y 轴移动速度 */
    int16_t mouse_z;                /**< 鼠标 Z 轴滚轮滚动速度 */
    uint8_t mouse_press_l;          /**< 鼠标左键物理按下状态 (0:释放, 1:按下) */
    uint8_t mouse_press_r;          /**< 鼠标右键物理按下状态 (0:释放, 1:按下) */

    // 键盘按键数据按位映射 (Byte 14 - 15)
    uint16_t key_w           : 1;   /**< W 键按下状态 */
    uint16_t key_s           : 1;   /**< S 键按下状态 */
    uint16_t key_a           : 1;   /**< A 键按下状态 */
    uint16_t key_d           : 1;   /**< D 键按下状态 */
    uint16_t key_shift       : 1;   /**< Shift 键按下状态 */
    uint16_t key_ctrl        : 1;   /**< Ctrl 键按下状态 */
    uint16_t key_q           : 1;   /**< Q 键按下状态 */
    uint16_t key_e           : 1;   /**< E 键按下状态 */
    uint16_t key_r           : 1;   /**< R 键按下状态 */
    uint16_t key_f           : 1;   /**< F 键按下状态 */
    uint16_t key_g           : 1;   /**< G 键按下状态 */
    uint16_t key_z           : 1;   /**< Z 键按下状态 */
    uint16_t key_x           : 1;   /**< X 键按下状态 */
    uint16_t key_c           : 1;   /**< C 键按下状态 */
    uint16_t key_v           : 1;   /**< V 键按下状态 */
    uint16_t key_b           : 1;   /**< B 键按下状态 */

    // 遥控器左侧拨轮/辅助通道 (Byte 16 - 17)
    uint16_t dial            : 11;  /**< 左侧拨轮通道数据 (中点1024) */
    uint16_t reserved        : 5;   /**< 保留填充位 */
} DBUS_FrameTypeDef;

#pragma pack(pop)

/**
 * @brief DBUS 联合体，兼容数组流读写与直接位域解算
 */
typedef union {
    uint8_t GetData[18];
    DBUS_FrameTypeDef Frame;
} DBUS_UNION_TypeDef;

/**
 * @brief DBUS 接收数据解算
 * @param Data 18 字节原始输入缓存数据指针
 * @param DBUS 解算结果存放的应用层目标结构体指针
 */
void DBUS_Resolved(uint8_t* Data, void *device_ptr, uint16_t size);

#endif //HORIZON_DBUS_H