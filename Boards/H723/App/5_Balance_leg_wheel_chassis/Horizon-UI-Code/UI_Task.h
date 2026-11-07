/* 首次接入时复制为 UI_Task.h；之后由用户维护。 */
#ifndef UI_TASK_H
#define UI_TASK_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

void App_UI_Init(void);
void App_UI_Task(uint32_t now_ms);
void App_UI_RequestRedraw(void);

#ifdef __cplusplus
}
#endif
#endif
