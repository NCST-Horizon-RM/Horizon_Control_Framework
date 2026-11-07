/* 首次接入时复制为 UI_Task.c；业务和串口代码由用户维护。 */
#include "UI_Task.h"

#include <string.h>

#include "Robot_Config.h"
#include "ui_bindings.h"
#include "usart.h"
#include "IMU_Task.h"

static UI_Data ui_data;
static uint8_t tx_buf[128]__attribute__((section(".RAM_D2")));
static uint8_t referee_send(const uint8_t *bytes, uint16_t length)
{
    memcpy(tx_buf, bytes, length);
    if (HAL_UART_Transmit_DMA(&huart1,tx_buf,length) == HAL_OK) {
        return 1;
    }
    return 0;
}

void App_UI_Init(void)
{
    ui_data = UI_DataDefault;
    UI_Init(103, referee_send); /* 使用裁判系统实际机器人 ID */
    UI_Update(&ui_data);
}

void App_UI_Task(uint32_t now_ms)
{
    float capacitor_level = 50.0f + 50.0f * sinf((float)now_ms * 0.0005f);
    ui_data.capacitor_percent = capacitor_level;

    if (capacitor_level >= 75.0f)
        ui_data.capacitor_color = UI_GREEN;
    else if (capacitor_level >= 50.0f)
        ui_data.capacitor_color = UI_YELLOW;
    else if (capacitor_level >= 30.0f)
        ui_data.capacitor_color = UI_ORANGE;
    else
        ui_data.capacitor_color = UI_PINK;

    ui_data.chassis_power  = Meter.power;
    ui_data.attitude_pitch = IMU_Data.pitch;
    ui_data.attitude_roll  = IMU_Data.roll;
    ui_data.heading_deg    = IMU_Data.yaw;

    ui_data.robot_data_valid = 1;
    ui_data.field_data_valid = 1;

    UI_Update(&ui_data);
    UI_Tick(now_ms);
}

void App_UI_RequestRedraw(void)
{
    UI_RequestRedraw();
}

/* 也可在 UI 任务中直接修改属性：
    UI_SetNumber(UI_ID_ROBOT_HP, number_value);
    UI_SetText(UI_ID_MODE_LABEL, status_text);
    UI_SetLineEnd(UI_ID_AIM_LEFT, end_x, end_y);
    UI_SetAngles(UI_ID_AIM_RANGE, start_angle, end_angle);
    UI_SetRadius(UI_ID_AIM_CENTER, radius);
    UI_SetColor(UI_ID_AIM_LEFT, UI_GREEN);
   属性接口自动比较并标记变化；UI_DataDefault 和 UI_Initial 保持只读。
   上电调用一次 App_UI_Init；周期调用 App_UI_Task；按需调用一次重绘接口。
*/
