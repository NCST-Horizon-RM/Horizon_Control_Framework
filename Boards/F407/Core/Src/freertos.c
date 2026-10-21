/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#define RTOS_SNAPSHOT_CAPACITY 8U

typedef struct {
  char name[configMAX_TASK_NAME_LEN];
  uint32_t number;
  uint32_t handle;
  uint32_t state;
  uint32_t priority;
  uint32_t base_priority;
  uint32_t stack_free_words;
  uint32_t runtime_count;
  uint32_t stack_total_bytes;
} RtosSnapshotEntry;

typedef struct {
  uint32_t sequence;
  uint32_t tick;
  uint32_t count;
  uint32_t overflow;
  uint32_t free_heap_bytes;
  uint32_t min_free_heap_bytes;
  uint32_t runtime_total;
  RtosSnapshotEntry tasks[RTOS_SNAPSHOT_CAPACITY];
} RtosSnapshot;

volatile RtosSnapshot g_rtos_snapshot;
static TaskStatus_t rtos_task_status[RTOS_SNAPSHOT_CAPACITY];
/* USER CODE END Variables */
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = {
  .name = "Motor",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for Command */
osThreadId_t CommandHandle;
const osThreadAttr_t Command_attributes = {
  .name = "Command",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for RtosSnapshot */
osThreadId_t RtosSnapshotHandle;
const osThreadAttr_t RtosSnapshot_attributes = {
  .name = "RtosSnapshot",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint32_t RtosStackSizeBytes(TaskHandle_t handle);

/* USER CODE END FunctionPrototypes */

void IMU_Task(void *argument);
void Motor_Task(void *argument);
void Command_Task(void *argument);
void StartTask01(void *argument);
void StartTask02(void *argument);
void RtosSnapshotTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of IMU */
  IMUHandle = osThreadNew(IMU_Task, NULL, &IMU_attributes);

  /* creation of Motor */
  MotorHandle = osThreadNew(Motor_Task, NULL, &Motor_attributes);

  /* creation of Command */
  CommandHandle = osThreadNew(Command_Task, NULL, &Command_attributes);

  /* creation of Task01 */
  Task01Handle = osThreadNew(StartTask01, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

  /* creation of RtosSnapshot */
  RtosSnapshotHandle = osThreadNew(RtosSnapshotTask, NULL, &RtosSnapshot_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_IMU_Task */
/**
  * @brief  Function implementing the IMU thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_Motor_Task */
/**
* @brief Function implementing the Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task */
__weak void Motor_Task(void *argument)
{
  /* USER CODE BEGIN Motor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Task */
}

/* USER CODE BEGIN Header_Command_Task */
/**
* @brief Function implementing the Command thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Command_Task */
__weak void Command_Task(void *argument)
{
  /* USER CODE BEGIN Command_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Command_Task */
}

/* USER CODE BEGIN Header_StartTask01 */
/**
* @brief Function implementing the Task01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask01 */
__weak void StartTask01(void *argument)
{
  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
__weak void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_RtosSnapshotTask */
/**
* @brief Function implementing the RtosSnapshot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RtosSnapshotTask */
__weak void RtosSnapshotTask(void *argument)
{
  /* USER CODE BEGIN RtosSnapshotTask */
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(125U));

    vTaskSuspendAll();

    UBaseType_t expected = uxTaskGetNumberOfTasks();
    UBaseType_t count = 0;
    uint32_t total_runtime = 0;
    g_rtos_snapshot.sequence++;
    __DMB();

    if (expected <= RTOS_SNAPSHOT_CAPACITY) {
      count = uxTaskGetSystemState(
          rtos_task_status,
          RTOS_SNAPSHOT_CAPACITY,
          &total_runtime);
    }

    g_rtos_snapshot.tick = xTaskGetTickCount();
    g_rtos_snapshot.count = count;
    g_rtos_snapshot.overflow = (count != expected) ? 1U : 0U;
    g_rtos_snapshot.free_heap_bytes = xPortGetFreeHeapSize();
    g_rtos_snapshot.min_free_heap_bytes =
        xPortGetMinimumEverFreeHeapSize();
    g_rtos_snapshot.runtime_total = total_runtime;

    for (UBaseType_t index = 0; index < count; index++) {
      const TaskStatus_t *source = &rtos_task_status[index];
      volatile RtosSnapshotEntry *target =
          &g_rtos_snapshot.tasks[index];

      target->number = source->xTaskNumber;
      target->handle = (uint32_t)(uintptr_t)source->xHandle;
      target->state = (uint32_t)source->eCurrentState;
      target->priority = source->uxCurrentPriority;
      target->base_priority = source->uxBasePriority;
      target->stack_free_words = source->usStackHighWaterMark;
      target->runtime_count = source->ulRunTimeCounter;
      target->stack_total_bytes = RtosStackSizeBytes(source->xHandle);

      uint32_t name_index = 0;
      while (name_index < configMAX_TASK_NAME_LEN - 1U &&
             source->pcTaskName[name_index] != '\0') {
        target->name[name_index] = source->pcTaskName[name_index];
        name_index++;
             }
      target->name[name_index] = '\0';
    }

    __DMB();
    g_rtos_snapshot.sequence++;

    (void)xTaskResumeAll();
  }
  /* USER CODE END RtosSnapshotTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static uint32_t RtosStackSizeBytes(TaskHandle_t handle)
{
  if (handle == (TaskHandle_t)IMUHandle) return IMU_attributes.stack_size;
  if (handle == (TaskHandle_t)MotorHandle) return Motor_attributes.stack_size;
  if (handle == (TaskHandle_t)CommandHandle) return Command_attributes.stack_size;
  if (handle == (TaskHandle_t)Task01Handle) return Task01_attributes.stack_size;
  if (handle == (TaskHandle_t)Task02Handle) return Task02_attributes.stack_size;
  if (handle == (TaskHandle_t)RtosSnapshotHandle) return RtosSnapshot_attributes.stack_size;
  if (handle == xTaskGetIdleTaskHandle()) return configMINIMAL_STACK_SIZE * sizeof(StackType_t);
  if (handle == xTimerGetTimerDaemonTaskHandle()) return configTIMER_TASK_STACK_DEPTH * sizeof(StackType_t);
  return 0;
}

/* USER CODE END Application */

