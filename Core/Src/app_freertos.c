/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
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
#include <odometry.h>
#include <zetta_protocol.h>
#include "crc.h"
#include "usart.h"
#include <bsp.h> 
#include <navigation.h>
extern OdometryTypedef odo;
Zetta_t hzettatx;
Zetta_t hzettarx;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    // HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
    zetta_transmit_cplt_clb(&hzettatx);
}
void uart_send(void *data, uint8_t size)
{
  HAL_UART_Transmit_DMA(&huart2, data, size);
}

uint32_t crc_compute(uint32_t *data, uint32_t size)
{
  __HAL_CRC_DR_RESET(&hcrc);
  return HAL_CRC_Calculate(&hcrc, data, size);
}
ZettaInterface_t iface = {
    .send = uart_send,
    .computeCRC = crc_compute,
};

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
/* Definitions for OdomTransmit */
osThreadId_t OdomTransmitHandle;
const osThreadAttr_t OdomTransmit_attributes = {
  .name = "OdomTransmit",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for CommandsGet */
osThreadId_t CommandsGetHandle;
const osThreadAttr_t CommandsGet_attributes = {
  .name = "CommandsGet",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Navigation */
osThreadId_t NavigationHandle;
const osThreadAttr_t Navigation_attributes = {
  .name = "Navigation",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void OdomTransmitTask(void *argument);
void CommandsGetTask(void *argument);
void NavigationTask(void *argument);

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
  /* creation of OdomTransmit */
  OdomTransmitHandle = osThreadNew(OdomTransmitTask, NULL, &OdomTransmit_attributes);

  /* creation of CommandsGet */
  CommandsGetHandle = osThreadNew(CommandsGetTask, NULL, &CommandsGet_attributes);

  /* creation of Navigation */
  NavigationHandle = osThreadNew(NavigationTask, NULL, &Navigation_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_OdomTransmitTask */
/**
 * @brief  Function implementing the OdomTransmit thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_OdomTransmitTask */
void OdomTransmitTask(void *argument)
{
  /* USER CODE BEGIN OdomTransmitTask */
  /* Infinite loop */
  zetta_init(&hzettatx, iface);

  for (;;)
  {
    zetta_send(&hzettatx, MSG_PUBLISH, &odo, sizeof(OdometryTypedef));

    osDelay(20);
  }
  /* USER CODE END OdomTransmitTask */
}

/* USER CODE BEGIN Header_CommandsGetTask */
/**
 * @brief Function implementing the CommandsGet thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CommandsGetTask */
void CommandsGetTask(void *argument)
{
  /* USER CODE BEGIN CommandsGetTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END CommandsGetTask */
}

/* USER CODE BEGIN Header_NavigationTask */
/**
 * @brief Function implementing the Navigation thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_NavigationTask */
void NavigationTask(void *argument)
{
  /* USER CODE BEGIN NavigationTask */
  /* Infinite loop */

  OdoInit(&odo);
  for (;;)
  {
    // test_uart();
    // test_encoder();
    uint8_t btn_state = read_btn_status();
    if (btn_state)
    {

      // test_max_ang_velocity();
      // command_motors(MIN_PWM_SPEED, MIN_PWM_SPEED);
      // move_angular_speed(5, &odo);
      position_control(50, &odo);
      osDelay(1000);
      angle_control(DEG2RAD(90), &odo);
      osDelay(1000);
      position_control(50, &odo);
      osDelay(1000);
      angle_control(DEG2RAD(90), &odo);
      osDelay(1000);
      position_control(50, &odo);
      osDelay(1000);
      angle_control(DEG2RAD(90), &odo);
      osDelay(1000);
      position_control(50, &odo);
      osDelay(1000);
      angle_control(DEG2RAD(90), &odo);

      btn_state = 0;
    }
    // HAL_Delay(1000);

    osDelay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END NavigationTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

