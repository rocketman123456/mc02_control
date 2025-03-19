/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for imuTempCtrl */
osThreadId_t imuTempCtrlHandle;
uint32_t imuTempCtrlBuffer[ 512 ];
osStaticThreadDef_t imuTemoCtrlControlBlock;
const osThreadAttr_t imuTempCtrl_attributes = {
  .name = "imuTempCtrl",
  .cb_mem = &imuTemoCtrlControlBlock,
  .cb_size = sizeof(imuTemoCtrlControlBlock),
  .stack_mem = &imuTempCtrlBuffer[0],
  .stack_size = sizeof(imuTempCtrlBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for can1CtrlTask */
osThreadId_t can1CtrlTaskHandle;
uint32_t can1CtrlTaskBuffer[ 512 ];
osStaticThreadDef_t can1CtrlTaskControlBlock;
const osThreadAttr_t can1CtrlTask_attributes = {
  .name = "can1CtrlTask",
  .cb_mem = &can1CtrlTaskControlBlock,
  .cb_size = sizeof(can1CtrlTaskControlBlock),
  .stack_mem = &can1CtrlTaskBuffer[0],
  .stack_size = sizeof(can1CtrlTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can2CtrlTask */
osThreadId_t can2CtrlTaskHandle;
uint32_t can2CtrlTaskBuffer[ 128 ];
osStaticThreadDef_t can2CtrlTaskControlBlock;
const osThreadAttr_t can2CtrlTask_attributes = {
  .name = "can2CtrlTask",
  .cb_mem = &can2CtrlTaskControlBlock,
  .cb_size = sizeof(can2CtrlTaskControlBlock),
  .stack_mem = &can2CtrlTaskBuffer[0],
  .stack_size = sizeof(can2CtrlTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for voltReadTask */
osThreadId_t voltReadTaskHandle;
uint32_t voltReadTaskBuffer[ 128 ];
osStaticThreadDef_t voltReadTaskControlBlock;
const osThreadAttr_t voltReadTask_attributes = {
  .name = "voltReadTask",
  .cb_mem = &voltReadTaskControlBlock,
  .cb_size = sizeof(voltReadTaskControlBlock),
  .stack_mem = &voltReadTaskBuffer[0],
  .stack_size = sizeof(voltReadTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for motorDataMutex */
osMutexId_t motorDataMutexHandle;
osStaticMutexDef_t motorDataMutexControlBlock;
const osMutexAttr_t motorDataMutex_attributes = {
  .name = "motorDataMutex",
  .cb_mem = &motorDataMutexControlBlock,
  .cb_size = sizeof(motorDataMutexControlBlock),
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
osStaticSemaphoreDef_t imuBinarySemControlBlock;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01",
  .cb_mem = &imuBinarySemControlBlock,
  .cb_size = sizeof(imuBinarySemControlBlock),
};
/* Definitions for can1BinarySem */
osSemaphoreId_t can1BinarySemHandle;
osStaticSemaphoreDef_t can1BinarySemControlBlock;
const osSemaphoreAttr_t can1BinarySem_attributes = {
  .name = "can1BinarySem",
  .cb_mem = &can1BinarySemControlBlock,
  .cb_size = sizeof(can1BinarySemControlBlock),
};
/* Definitions for can2BinarySem */
osSemaphoreId_t can2BinarySemHandle;
osStaticSemaphoreDef_t can2BinarySemControlBlock;
const osSemaphoreAttr_t can2BinarySem_attributes = {
  .name = "can2BinarySem",
  .cb_mem = &can2BinarySemControlBlock,
  .cb_size = sizeof(can2BinarySemControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_TempCtrlTask(void *argument);
void CAN1_CtrlTask(void *argument);
void CAN2_CtrlTask(void *argument);
void VoltReadTask(void *argument);

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
  /* Create the mutex(es) */
  /* creation of motorDataMutex */
  motorDataMutexHandle = osMutexNew(&motorDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  imuBinarySem01Handle = osSemaphoreNew(1, 0, &imuBinarySem01_attributes);

  /* creation of can1BinarySem */
  can1BinarySemHandle = osSemaphoreNew(1, 0, &can1BinarySem_attributes);

  /* creation of can2BinarySem */
  can2BinarySemHandle = osSemaphoreNew(1, 0, &can2BinarySem_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of imuTempCtrl */
  imuTempCtrlHandle = osThreadNew(IMU_TempCtrlTask, NULL, &imuTempCtrl_attributes);

  /* creation of can1CtrlTask */
  can1CtrlTaskHandle = osThreadNew(CAN1_CtrlTask, NULL, &can1CtrlTask_attributes);

  /* creation of can2CtrlTask */
  can2CtrlTaskHandle = osThreadNew(CAN2_CtrlTask, NULL, &can2CtrlTask_attributes);

  /* creation of voltReadTask */
  voltReadTaskHandle = osThreadNew(VoltReadTask, NULL, &voltReadTask_attributes);

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
__weak void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_TempCtrlTask */
/**
* @brief Function implementing the imuTempCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_TempCtrlTask */
__weak void IMU_TempCtrlTask(void *argument)
{
  /* USER CODE BEGIN IMU_TempCtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_TempCtrlTask */
}

/* USER CODE BEGIN Header_CAN1_CtrlTask */
/**
* @brief Function implementing the can1CtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN1_CtrlTask */
__weak void CAN1_CtrlTask(void *argument)
{
  /* USER CODE BEGIN CAN1_CtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN1_CtrlTask */
}

/* USER CODE BEGIN Header_CAN2_CtrlTask */
/**
* @brief Function implementing the can2CtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN2_CtrlTask */
__weak void CAN2_CtrlTask(void *argument)
{
  /* USER CODE BEGIN CAN2_CtrlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN2_CtrlTask */
}

/* USER CODE BEGIN Header_VoltReadTask */
/**
* @brief Function implementing the voltReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VoltReadTask */
__weak void VoltReadTask(void *argument)
{
  /* USER CODE BEGIN VoltReadTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END VoltReadTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

