/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <delay.h>
#include <navigation.h>
#include <odometry.h>
#include <test.h>
#include <motor.h>
#include <navigation.h>
#include <debug_driver.h>
#include <encoder.h>
#include <pwm.h>
#include <bsp.h>
#include <zetta_protocol.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile uint32_t odo_time = 0, curr_odo_time = 0, prev_odo_time = 0;
OdometryTypedef odo = {0};
Zetta_t hzettatx;
Zetta_t hzettarx;
#pragma pack(push, 1)
typedef struct
{
  uint8_t nav_state; // 0 , stop , 1 position , 2 angle
  float cmd;
} NavCommand_t;
#pragma pack(pop)

volatile NavCommand_t nav_data = {0};
volatile uint8_t zetta_rx_byte = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  // HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);
  zetta_transmit_cplt_clb(&hzettatx);
}
void uart_send(void *data, uint8_t size)
{
  HAL_UART_Transmit_DMA(&huart2, data, size);
}

uint32_t stm32_crc(uint32_t *data, uint32_t size)
{
  __HAL_CRC_DR_RESET(&hcrc);
  uint32_t crc = 0;
  crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)data, size);
  return crc;
}

ZettaInterface_t iface = {
    .send = uart_send,
    .computeCRC = stm32_crc,
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (zetta_ParseByte(&hzettarx, zetta_rx_byte) == ZETTA_OK)
  {
    TurOn_led();
    Zetta_GetPayload(&hzettarx, (void *)&nav_data);
    // process buffer
  }

  zetta_recieve_cplt_clb(&hzettarx);
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)&zetta_rx_byte, 1);
}

volatile uint8_t btn_state = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_CRC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  // set urt2 txtfifo threshold to full
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }

  init_motors();
  timer1_LeftEncoder_start();
  timer3_RightEncoder_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // OdoInit(&odo);
  zetta_init(&hzettarx, iface);
  zetta_init(&hzettatx, iface);
  OdoInit(&odo);
  nav_data.nav_state = 3;
  nav_data.cmd = 0.0f;
  HAL_UART_Receive_DMA(&huart2, (uint8_t *)&zetta_rx_byte, 1);

  while (1)
  {
    // switch (nav_data.nav_state)
    // {
    // case 0: // RESET
    //   /* code */
    //   command_motors(0, 0);
    //   osDelay(20);
    //   OdoInit(&odo);
    //   OdoUpdate(&odo, 0);
    //   zetta_send(&hzettatx, MSG_PUBLISH, &odo, sizeof(OdometryTypedef));
    //   nav_data.nav_state = 3 ;
    //   break;
    // case 1:
    //   /* code */
    //   position_control(nav_data.cmd, &odo);
    //   nav_data.nav_state = 3 ;
    //   break;
    // case 2:
    //   /* code */
    //   angle_control(DEG2RAD(nav_data.cmd), &odo);

    //   nav_data.nav_state = 3 ;
    //   break;

    // default:

    //   command_motors(0, 0);
    //   osDelay(20);
    //   OdoUpdate(&odo, 0);
    //   zetta_send(&hzettatx, MSG_PUBLISH, &odo, sizeof(OdometryTypedef));
    //   break;
    // }
    // test_uart();
    // test_encoder();
    btn_state = read_btn_status();
    if (btn_state)
    {
      // position_control(50, &odo);
      // delay_ms(1000);
      angle_control(DEG2RAD(90), &odo);
      // delay_ms(1000);
      // position_control(50, &odo);
      // delay_ms(1000);
      // angle_control(DEG2RAD(90), &odo);
      // delay_ms(1000);
      // position_control(50, &odo);
      // delay_ms(1000);
      // angle_control(DEG2RAD(90), &odo);
      // delay_ms(1000);
      // position_control(50, &odo);
      // delay_ms(1000);
      // angle_control(DEG2RAD(90), &odo);

      btn_state = 0;
    }
    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
