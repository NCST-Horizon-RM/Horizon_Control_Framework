#include "USB_Service.h"
#include "USB_Port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#ifndef USB_SERVICE_STACK_WORDS
#define USB_SERVICE_STACK_WORDS 256U
#endif

static StaticTask_t service_tcb;
static uint32_t service_stack[USB_SERVICE_STACK_WORDS] __attribute__((aligned(8)));
static osThreadId_t service_thread;

static void ServiceTask(void *argument)
{
    (void)argument;
    for (;;) {
        USB_Process(USB_Port_Now(), USB_SERVICE_RX_BUDGET);
        /* One tick also services deadlines and missed/coalesced notifications. */
        (void)ulTaskNotifyTake(pdTRUE, 1U);
    }
}

USB_Status_t USB_ServiceStart(void)
{
    if (USB_Port_InISR() || osKernelGetState() == osKernelInactive) return USB_ERR_NOT_READY;
    if (service_thread != NULL) return USB_OK;
    USB_Status_t status = Auto_USB_Router_Init();
    if (status != USB_OK) return status;
    const osThreadAttr_t attr = {
        .name = "USB_CDC",
        .cb_mem = &service_tcb,
        .cb_size = sizeof(service_tcb),
        .stack_mem = service_stack,
        .stack_size = sizeof(service_stack),
        .priority = osPriorityNormal,
    };
    service_thread = osThreadNew(ServiceTask, NULL, &attr);
    return service_thread != NULL ? USB_OK : USB_ERR_FULL;
}

void USB_ServiceNotify(void)
{
    /* USB enumeration may run before RTOS initialization/scheduler startup. */
    TaskHandle_t thread = (TaskHandle_t)service_thread;
    if (thread == NULL || xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return;
    if (USB_Port_InISR()) {
        BaseType_t wake = pdFALSE;
        vTaskNotifyGiveFromISR(thread, &wake);
        portYIELD_FROM_ISR(wake);
    } else {
        xTaskNotifyGive(thread);
    }
}
