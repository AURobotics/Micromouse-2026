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
#include "app_main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LoggingQueue */
  LoggingQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &LoggingQueue_attributes);

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
  StartDefaultTask_run(argument);
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
  algorithmTask_run(argument);
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
  controlTask_run(argument);
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
  motionTask_run(argument);
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
  bnoTask_run(argument);
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
  HMIConfigTask_run(argument);
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
  loggerTask_run(argument);
  /* USER CODE END loggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

