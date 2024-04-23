/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
extern bool is_uart_receive;
extern UART_HandleTypeDef huart1;
bool task_send=false;
bool uart_flag=false;
/* USER CODE END Variables */
/* Definitions for led0 */
osThreadId_t led0Handle;
const osThreadAttr_t led0_attributes = {
  .name = "led0",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led1 */
osThreadId_t led1Handle;
const osThreadAttr_t led1_attributes = {
  .name = "led1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_flag=true;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

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
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (5, sizeof(uint8_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of led0 */
  led0Handle = osThreadNew(StartDefaultTask, NULL, &led0_attributes);

  /* creation of led1 */
  led1Handle = osThreadNew(StartTask02, NULL, &led1_attributes);

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
	uint32_t pretime=HAL_GetTick();
	uint8_t receive_val1;
	uint8_t receive_val2;
	uint8_t receive_queue_priority1=0;
	uint8_t receive_queue_priority2=1;
  /* Infinite loop */
  for(;;)
  {

	if(HAL_GetTick()-pretime>50){
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_3);
		pretime=HAL_GetTick();
	}
	if(task_send){
		osMessageQueueGet(myQueue01Handle,&receive_val1,&receive_queue_priority1,10);
		printf("task1-1) current receive val:%c, 0x%X\n",receive_val1,receive_val1);
		osMessageQueueGet(myQueue01Handle,&receive_val2,&receive_queue_priority2,10);
		printf("task1-2) current receive val:%c, 0x%X\n",receive_val2,receive_val2);
		task_send=false;
	}
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the led1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint32_t pretime=HAL_GetTick();
	uint8_t send_val[2];
  /* Infinite loop */
  for(;;)
  {

	  HAL_UART_Receive_DMA(&huart1,&send_val[0],2);
	  if(uart_flag){
		  printf("\x1B[2J");
		  printf("task2-1) transmit val:%c, 0x%X\n",send_val[0],send_val[0]);
		  printf("task2-2) transmit val:%c, 0x%X\n",send_val[1],send_val[1]);
		  is_uart_receive=false;
		  osMessageQueuePut(myQueue01Handle,&send_val[0],0,10);
		  osMessageQueuePut(myQueue01Handle,&send_val[1],1,10);
		  task_send=true;
		  uart_flag=false;
	  }
	  if(HAL_GetTick()-pretime>300){
	  	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
	  	pretime=HAL_GetTick();
	}

    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

