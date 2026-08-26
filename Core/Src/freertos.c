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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_can_recv.h"
#include "task_gimbal.h"
#include "task_zdt.h"
#include "task_ibus.h"
#include "task_chassis.h"
#include "task_flash_lcd.h"
#include "task_cmd_uart.h"
#include "task_safety.h"
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

/* USER CODE END Variables */
/* Definitions for can_handle */
osThreadId_t can_handleHandle;
const osThreadAttr_t can_handle_attributes = {
  .name = "can_handle",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for MotorSendTask */
osThreadId_t MotorSendTaskHandle;
const osThreadAttr_t MotorSendTask_attributes = {
  .name = "MotorSendTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ZDT_Control */
osThreadId_t ZDT_ControlHandle;
const osThreadAttr_t ZDT_Control_attributes = {
  .name = "ZDT_Control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_lcd_loop */
osThreadId_t task_lcd_loopHandle;
const osThreadAttr_t task_lcd_loop_attributes = {
  .name = "task_lcd_loop",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task_ibus_recv */
osThreadId_t task_ibus_recvHandle;
const osThreadAttr_t task_ibus_recv_attributes = {
  .name = "task_ibus_recv",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_chassis_lo */
osThreadId_t task_chassis_loHandle;
const osThreadAttr_t task_chassis_lo_attributes = {
  .name = "task_chassis_lo",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CmdUart */
osThreadId_t CmdUartHandle;
const osThreadAttr_t CmdUart_attributes = {
  .name = "CmdUart",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SafetyTaskHandl */
osThreadId_t SafetyTaskHandlHandle;
const osThreadAttr_t SafetyTaskHandl_attributes = {
  .name = "SafetyTaskHandl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for can_rx_queueHandle */
osMessageQueueId_t can_rx_queueHandleHandle;
const osMessageQueueAttr_t can_rx_queueHandle_attributes = {
  .name = "can_rx_queueHandle"
};
/* Definitions for ibus_rx_queue */
osMessageQueueId_t ibus_rx_queueHandle;
const osMessageQueueAttr_t ibus_rx_queue_attributes = {
  .name = "ibus_rx_queue"
};
/* Definitions for key_6220 */
osMutexId_t key_6220Handle;
const osMutexAttr_t key_6220_attributes = {
  .name = "key_6220"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);

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
  /* creation of key_6220 */
  key_6220Handle = osMutexNew(&key_6220_attributes);

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
  /* creation of can_rx_queueHandle */
  can_rx_queueHandleHandle = osMessageQueueNew (16, 16, &can_rx_queueHandle_attributes);

  /* creation of ibus_rx_queue */
  ibus_rx_queueHandle = osMessageQueueNew (16, 18, &ibus_rx_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  task_cmd_uart_init(); 
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of can_handle */
  can_handleHandle = osThreadNew(StartDefaultTask, NULL, &can_handle_attributes);

  /* creation of MotorSendTask */
  MotorSendTaskHandle = osThreadNew(StartTask02, NULL, &MotorSendTask_attributes);

  /* creation of ZDT_Control */
  ZDT_ControlHandle = osThreadNew(StartTask03, NULL, &ZDT_Control_attributes);

  /* creation of task_lcd_loop */
  task_lcd_loopHandle = osThreadNew(StartTask04, NULL, &task_lcd_loop_attributes);

  /* creation of task_ibus_recv */
  task_ibus_recvHandle = osThreadNew(StartTask05, NULL, &task_ibus_recv_attributes);

  /* creation of task_chassis_lo */
  task_chassis_loHandle = osThreadNew(StartTask06, NULL, &task_chassis_lo_attributes);

  /* creation of CmdUart */
  CmdUartHandle = osThreadNew(StartTask07, NULL, &CmdUart_attributes);

  /* creation of SafetyTaskHandl */
  SafetyTaskHandlHandle = osThreadNew(StartTask08, NULL, &SafetyTaskHandl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the MotorReceiveTas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  task_can_recv(argument);
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the MotorSendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  task_gimbal_loop(argument);
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the ZDT_Receive_Sen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  task_zdt_loop(argument);
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Flash_Save_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  task_flash_lcd_loop(argument);
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the ZDT_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  task_ibus_recv(argument);
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the myTask06_3508 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  task_chassis_loop(argument);
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the CmdUart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  task_cmd_uart_loop(argument);   /* 上位机命令解析（内部是 while(1)）*/
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the SafetyTaskHandl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Infinite loop */
  for(;;)
  {
    task_safety_loop(argument);
    osDelay(1);
  }
  /* USER CODE END StartTask08 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
 
/* USER CODE END Application */

