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
#include "freertos.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* Maximum length of a single log message, storage for the "thing to be
 * printed" that is queued from the calling task up to the LoggerTask. */
#define LOG_MSG_MAX_LEN   64U

/* One entry stored in LoggingQueueHandle. Kept as a plain fixed-size struct
 * (no pointers) so producer tasks can safely queue-by-copy and immediately
 * reuse their own stack buffer. */
typedef struct
{
  char message[LOG_MSG_MAX_LEN];
} LogMessage_t;

/* One row of the "clipboard" that lets Logger_Print() rate-limit each
 * calling task independently, at that task's own requested frequency. */
typedef struct
{
  osThreadId_t taskId;          /* which task this row belongs to */
  uint32_t     periodMs;        /* that task's own requested frequency */
  uint32_t     nextAllowedTick; /* earliest tick this task is allowed to print again */
} LoggerSource_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LOGGER_QUEUE_LENGTH                16U      /* entries held in LoggingQueue */
#define LOGGER_MAX_SOURCES                 8U       /* distinct tasks Logger_Print can track at once */
#define LOGGER_UART_TX_TIMEOUT_MS           20U      /* HAL_UART_Transmit timeout */

/* CYBLE_SLEEP_Pin drives the Bluetooth (Cypress BLE) module's sleep input.
 * NOTE: polarity assumed active-high = sleep, active-low = awake based on
 * the default GPIO_PIN_RESET init state in gpio.c. Verify against your
 * module's datasheet and flip these two macros if it turns out inverted. */
#define BLE_SLEEP_ASSERT()    HAL_GPIO_WritePin(CYBLE_SLEEP_GPIO_Port, CYBLE_SLEEP_Pin, GPIO_PIN_SET)
#define BLE_SLEEP_RELEASE()   HAL_GPIO_WritePin(CYBLE_SLEEP_GPIO_Port, CYBLE_SLEEP_Pin, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Guards loggerActiveOutput / the UART peripherals it maps to, so
 * Logger_SetOutput() can't race with LoggerTask mid-transmit. */
osMutexId_t LoggerConfigMutexHandle;

/* Guards the loggerSources[] table below, since multiple producer tasks
 * can call Logger_Print() concurrently. */
osMutexId_t LoggerSourcesMutexHandle;

static volatile LogOutput_t loggerActiveOutput  = LOG_OUTPUT_STLINK;
static volatile uint32_t    loggerDroppedCount  = 0U;

/* The "clipboard": one row per distinct task that has ever called
 * Logger_Print(), so each task's own requested frequency is remembered
 * and enforced independently of every other task's. */
static LoggerSource_t loggerSources[LOGGER_MAX_SOURCES];
static uint8_t         loggerSourceCount = 0U;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AlgorithimTask */
osThreadId_t AlgorithimTaskHandle;
uint32_t algorithim_taskBuffer[ 1024 ];
osStaticThreadDef_t algorithim_taskControlBlock;
const osThreadAttr_t AlgorithimTask_attributes = {
  .name = "AlgorithimTask",
  .cb_mem = &algorithim_taskControlBlock,
  .cb_size = sizeof(algorithim_taskControlBlock),
  .stack_mem = &algorithim_taskBuffer[0],
  .stack_size = sizeof(algorithim_taskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
uint32_t MotorControlTaskBuffer[ 512 ];
osStaticThreadDef_t MotorControlTaskControlBlock;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .cb_mem = &MotorControlTaskControlBlock,
  .cb_size = sizeof(MotorControlTaskControlBlock),
  .stack_mem = &MotorControlTaskBuffer[0],
  .stack_size = sizeof(MotorControlTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for MotionTask */
osThreadId_t MotionTaskHandle;
uint32_t PurepursuitTaskBuffer[ 512 ];
osStaticThreadDef_t PurepursuitTaskControlBlock;
const osThreadAttr_t MotionTask_attributes = {
  .name = "MotionTask",
  .cb_mem = &PurepursuitTaskControlBlock,
  .cb_size = sizeof(PurepursuitTaskControlBlock),
  .stack_mem = &PurepursuitTaskBuffer[0],
  .stack_size = sizeof(PurepursuitTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for BnoTask */
osThreadId_t BnoTaskHandle;
const osThreadAttr_t BnoTask_attributes = {
  .name = "BnoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for HMITask */
osThreadId_t HMITaskHandle;
const osThreadAttr_t HMITask_attributes = {
  .name = "HMITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LoggerTask */
osThreadId_t LoggerTaskHandle;
const osThreadAttr_t LoggerTask_attributes = {
  .name = "LoggerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for LoggingQueue */
osMessageQueueId_t LoggingQueueHandle;
const osMessageQueueAttr_t LoggingQueue_attributes = {
  .name = "LoggingQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void Logger_ActivateOutput(LogOutput_t output);
static void Logger_DeactivateOutput(LogOutput_t output);
static void Logger_TransmitMessage(LogOutput_t output, const LogMessage_t *msg);
static LoggerSource_t *Logger_FindOrCreateSource(osThreadId_t taskId, uint32_t period_ms);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void algorithimTask(void *argument);
void controlTask(void *argument);
void motionTask(void *argument);
void bnoTask(void *argument);
void HMIConfigTask(void *argument);
void loggerTask(void *argument);

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
  LoggerConfigMutexHandle = osMutexNew(NULL);
  LoggerSourcesMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LoggingQueue */
  /* NOTE: element type/size changed from the CubeMX default (uint16_t) to
   * LogMessage_t so it can carry formatted debug strings. If you regenerate
   * this project from the .ioc file, update the Message Queue's "Data Type"
   * in CubeMX (or re-apply this line) so it isn't reset to uint16_t. */
  LoggingQueueHandle = osMessageQueueNew (LOGGER_QUEUE_LENGTH, sizeof(LogMessage_t), &LoggingQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of AlgorithimTask */
  AlgorithimTaskHandle = osThreadNew(algorithimTask, NULL, &AlgorithimTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(controlTask, NULL, &ControlTask_attributes);

  /* creation of MotionTask */
  MotionTaskHandle = osThreadNew(motionTask, NULL, &MotionTask_attributes);

  /* creation of BnoTask */
  BnoTaskHandle = osThreadNew(bnoTask, NULL, &BnoTask_attributes);

  /* creation of HMITask */
  HMITaskHandle = osThreadNew(HMIConfigTask, NULL, &HMITask_attributes);

  /* creation of LoggerTask */
  LoggerTaskHandle = osThreadNew(loggerTask, NULL, &LoggerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_algorithimTask */
/**
* @brief Function implementing the AlgorithimTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_algorithimTask */
void algorithimTask(void *argument)
{
  /* USER CODE BEGIN algorithimTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END algorithimTask */
}

/* USER CODE BEGIN Header_controlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controlTask */
void controlTask(void *argument)
{
  /* USER CODE BEGIN controlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END controlTask */
}

/* USER CODE BEGIN Header_motionTask */
/**
* @brief Function implementing the MotionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motionTask */
void motionTask(void *argument)
{
  /* USER CODE BEGIN motionTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motionTask */
}

/* USER CODE BEGIN Header_bnoTask */
/**
* @brief Function implementing the BnoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bnoTask */
void bnoTask(void *argument)
{
  /* USER CODE BEGIN bnoTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END bnoTask */
}

/* USER CODE BEGIN Header_HMIConfigTask */
/**
* @brief Function implementing the HMITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HMIConfigTask */
void HMIConfigTask(void *argument)
{
  /* USER CODE BEGIN HMIConfigTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HMIConfigTask */
}

/* USER CODE BEGIN Header_loggerTask */
/**
* @brief Function implementing the LoggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_loggerTask */
void loggerTask(void *argument)
{
  /* USER CODE BEGIN loggerTask */
  LogMessage_t rxMsg;

  /* Only the default active output (ST-Link) starts up; everything else is
   * put to sleep/torn down so we don't have two UARTs driving in parallel. */
  Logger_ActivateOutput(loggerActiveOutput);
  for (LogOutput_t out = (LogOutput_t)0; out < LOG_OUTPUT_COUNT; out++)
  {
    if (out != loggerActiveOutput)
    {
      Logger_DeactivateOutput(out);
    }
  }

  /* Infinite loop */
  for (;;)
  {
    /* Block here until exactly one message is available - the task uses
     * zero CPU while the queue is empty, it isn't polling. Each caller's
     * own spacing was already enforced in Logger_Print() before this
     * message was ever queued, so LoggerTask just prints as they arrive. */
    if (osMessageQueueGet(LoggingQueueHandle, &rxMsg, NULL, osWaitForever) == osOK)
    {
      osMutexAcquire(LoggerConfigMutexHandle, osWaitForever);
      Logger_TransmitMessage(loggerActiveOutput, &rxMsg);
      osMutexRelease(LoggerConfigMutexHandle);
    }
  }
  /* USER CODE END loggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Powers up / wakes the given output peripheral.
  * @note   USB is stubbed out: this project has no USB_DEVICE middleware
  *         yet (no CDC class configured in CubeMX). Add USB_OTG_FS +
  *         USB_DEVICE(CDC) via CubeMX, generate MX_USB_DEVICE_Init(), then
  *         call it here.
  */
static void Logger_ActivateOutput(LogOutput_t output)
{
  switch (output)
  {
    case LOG_OUTPUT_STLINK:
      MX_USART2_UART_Init();
      break;

    case LOG_OUTPUT_BLUETOOTH:
      MX_UART4_Init();
      BLE_SLEEP_RELEASE();
      break;

    case LOG_OUTPUT_USB:
      /* TODO: MX_USB_DEVICE_Init(); once USB CDC is added to the project */
      break;

    default:
      break;
  }
}

/**
  * @brief  Puts the given output peripheral to sleep / tears it down so it
  *         is not left driving its lines while another output is selected.
  */
static void Logger_DeactivateOutput(LogOutput_t output)
{
  switch (output)
  {
    case LOG_OUTPUT_STLINK:
      HAL_UART_DeInit(&huart2);
      break;

    case LOG_OUTPUT_BLUETOOTH:
      BLE_SLEEP_ASSERT();
      HAL_UART_DeInit(&huart4);
      break;

    case LOG_OUTPUT_USB:
      /* TODO: USB CDC not yet configured in this project. */
      break;

    default:
      break;
  }
}

/**
  * @brief  Writes one already-formatted log entry out over the given output.
  */
static void Logger_TransmitMessage(LogOutput_t output, const LogMessage_t *msg)
{
  uint16_t len = (uint16_t)strnlen(msg->message, LOG_MSG_MAX_LEN);

  if (len == 0U)
  {
    return;
  }

  switch (output)
  {
    case LOG_OUTPUT_STLINK:
      HAL_UART_Transmit(&huart2, (uint8_t *)msg->message, len, LOGGER_UART_TX_TIMEOUT_MS);
      break;

    case LOG_OUTPUT_BLUETOOTH:
      HAL_UART_Transmit(&huart4, (uint8_t *)msg->message, len, LOGGER_UART_TX_TIMEOUT_MS);
      break;

    case LOG_OUTPUT_USB:
      /* TODO: CDC_Transmit_FS((uint8_t *)msg->message, len); once USB CDC exists */
      break;

    default:
      break;
  }
}

/**
  * @brief  Selects which physical interface debug logs are printed over.
  *         The two outputs not selected are put to sleep / torn down so
  *         only one interface is ever active at a time.
  * @param  output One of LOG_OUTPUT_USB / LOG_OUTPUT_STLINK / LOG_OUTPUT_BLUETOOTH.
  */
void Logger_SetOutput(LogOutput_t output)
{
  if (output >= LOG_OUTPUT_COUNT)
  {
    return;
  }

  osMutexAcquire(LoggerConfigMutexHandle, osWaitForever);

  if (output != loggerActiveOutput)
  {
    Logger_DeactivateOutput(loggerActiveOutput);
    Logger_ActivateOutput(output);
    loggerActiveOutput = output;
  }

  osMutexRelease(LoggerConfigMutexHandle);
}

/**
  * @brief  Finds this task's row in the source table, creating one on its
  *         first-ever call. This is the "clipboard lookup": it's how
  *         Logger_Print() tells different calling tasks apart.
  * @retval Pointer to the task's row, or NULL if the table is full.
  */
static LoggerSource_t *Logger_FindOrCreateSource(osThreadId_t taskId, uint32_t period_ms)
{
  for (uint8_t i = 0U; i < loggerSourceCount; i++)
  {
    if (loggerSources[i].taskId == taskId)
    {
      return &loggerSources[i];
    }
  }

  if (loggerSourceCount >= LOGGER_MAX_SOURCES)
  {
    return NULL;
  }

  loggerSources[loggerSourceCount].taskId          = taskId;
  loggerSources[loggerSourceCount].periodMs        = period_ms;
  loggerSources[loggerSourceCount].nextAllowedTick = osKernelGetTickCount(); /* allow immediately, first call */
  loggerSourceCount++;

  return &loggerSources[loggerSourceCount - 1U];
}

/**
  * @brief  Queues one printf-style debug message for LoggerTask to print,
  *         rate-limited to the CALLING TASK's own requested frequency.
  *         Each task gets its own independent cooldown - Task A asking for
  *         100ms and Task B asking for 200ms don't affect each other.
  *         Non-blocking by design: a stalled/full logger must never hold up
  *         a real-time task (Algorithm/Control/Motion) that calls this.
  * @param  period_ms This calling task's own print frequency, in ms.
  * @param  format printf-style format string, result truncated to LOG_MSG_MAX_LEN-1.
  * @retval osOK on success, osErrorParameter on bad args, osErrorResource if
  *         dropped (either too soon for this task's own frequency, or the
  *         queue was full - see Logger_GetDroppedCount for the latter).
  */
osStatus_t Logger_Print(uint32_t period_ms, const char *format, ...)
{
  LogMessage_t msg;
  va_list args;
  osThreadId_t caller;
  LoggerSource_t *src;
  uint32_t now;
  uint8_t allowed;
  osStatus_t status;

  if ((format == NULL) || (period_ms == 0U))
  {
    return osErrorParameter;
  }

  caller  = osThreadGetId();
  now     = osKernelGetTickCount();
  allowed = 0U;

  osMutexAcquire(LoggerSourcesMutexHandle, osWaitForever);

  src = Logger_FindOrCreateSource(caller, period_ms);
  if (src == NULL)
  {
    /* Source table full (more than LOGGER_MAX_SOURCES distinct callers):
     * fall back to always allowing this task through rather than
     * silently losing it forever. Bump LOGGER_MAX_SOURCES if this
     * happens often. */
    allowed = 1U;
  }
  else
  {
    src->periodMs = period_ms; /* caller may have changed its own rate */

    if (now >= src->nextAllowedTick)
    {
      allowed = 1U;
      src->nextAllowedTick = now + period_ms;
    }
  }

  osMutexRelease(LoggerSourcesMutexHandle);

  if (allowed == 0U)
  {
    /* Too soon for THIS caller's own frequency - drop it, not an error,
     * just this task's cooldown hasn't expired yet. */
    return osErrorResource;
  }

  va_start(args, format);
  vsnprintf(msg.message, LOG_MSG_MAX_LEN, format, args);
  va_end(args);

  status = osMessageQueuePut(LoggingQueueHandle, &msg, 0U, 0U);
  if (status != osOK)
  {
    loggerDroppedCount++;
  }

  return status;
}

/**
  * @brief  Number of log messages dropped because the queue was full,
  *         i.e. producers are outrunning the configured print frequency.
  */
uint32_t Logger_GetDroppedCount(void)
{
  return loggerDroppedCount;
}

/* USER CODE END Application */