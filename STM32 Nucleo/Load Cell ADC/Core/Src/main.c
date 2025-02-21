/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include <stdbool.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADS1219_ADDR 0x45 << 1

#define ADS1219_RDATA 0x10
#define ADS1219_WREG 0x40
#define ADS1219_STARTSYNC 0x08

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirecting the printf function to print to the console (over UART2)
int _write(int file, char *data, int len) {
    // Redirect printf to UART
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Buffers for TX/RX data of I2C bus
  uint8_t txData[3] = {0};
  uint8_t rxData[3] = {0};

  // Variables and arrays for storing load cell readings
  int32_t rawLoadCell1 = 0;
  int32_t rawLoadCell2 = 0;

//  int32_t LoadCell1_Offset = 13128;
  int32_t LoadCell1_Offset = 11140;
//  int32_t LoadCell2_OffsetA = 16764472;
//  int32_t LoadCell2_OffsetB = -12744;
  int32_t LoadCell2_OffsetA = 16766245;
  int32_t LoadCell2_OffsetB = 16766245;

  int32_t rawData1[25] = {0};
  int32_t rawData2[25] = {0};
  int32_t avg1 = 0;
  int32_t avg2 = 0;

  float volts = 0;
  float kilograms = 0;
  float lsbToKg = 0.000256;

  // Configure the ADS1219 to do continuous conversion, start with AIN0/1 load cell, use internal 2.048V reference and 90 SPS data rate
  txData[0] = ADS1219_WREG;
  txData[1] = 0x16;
  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);

  // Send the START/SYNC command to start continuous conversions
  txData[0] = ADS1219_STARTSYNC;
  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 1, HAL_MAX_DELAY);

  // Average 100 measurements to get the offset for each load cell
  for (int i = 0; i < 100; i++) {
	  // Configure ADS1219 MUX to measure AIN0/1 load cell
	  txData[0] = ADS1219_WREG;
	  txData[1] = 0x16;
	  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);
	  // Wait a bit to take the measurements
	  HAL_Delay(20);
	  // Read the measurement and store it
	  txData[0] = ADS1219_RDATA;
	  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 1, HAL_MAX_DELAY);
	  rxData[0] = 0x00;
	  rxData[1] = 0x00;
	  rxData[2] = 0x00;
	  HAL_I2C_Master_Receive(&hi2c1, ADS1219_ADDR, rxData, 3, HAL_MAX_DELAY);
	  rawLoadCell1 = (rxData[0] << 16) | (rxData[1] << 8) | rxData[2];
	  avg1 += rawLoadCell1;
//		  printf("%ld\r\n", rawLoadCell1);

	  // Configure ADS1115 MUX to measure AIN2/3 load cell
	  txData[0] = ADS1219_WREG;
	  txData[1] = 0x36;
	  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);
	  // Wait a bit to take the measurements
	  HAL_Delay(20);
	  // Read the measurement and store it
	  txData[0] = ADS1219_RDATA;
	  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 1, HAL_MAX_DELAY);
	  rxData[0] = 0x00;
	  rxData[1] = 0x00;
	  rxData[2] = 0x00;
	  HAL_I2C_Master_Receive(&hi2c1, ADS1219_ADDR, rxData, 3, HAL_MAX_DELAY);
	  rawLoadCell2 = (rxData[0] << 16) | (rxData[1] << 8) | rxData[2];
	  avg2 += rawLoadCell2;
  }

  avg1 = avg1/100;
  avg2 = avg2/100;
  LoadCell1_Offset = avg1;
  LoadCell2_OffsetA = avg2;
  LoadCell2_OffsetB = avg2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Wait 100ms between starting new measurements
//	  HAL_Delay(100);

	  // Average 25 measurements (over approx. 1 second) to get the reading from each load cell.
	  for (int i = 0; i < 25; i++) {
		  // Configure ADS1219 MUX to measure AIN0/1 load cell
		  txData[0] = ADS1219_WREG;
		  txData[1] = 0x16;
		  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);
		  // Wait a bit to take the measurements
		  HAL_Delay(20);
		  // Read the measurement and store it
		  txData[0] = ADS1219_RDATA;
		  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 1, HAL_MAX_DELAY);
		  rxData[0] = 0x00;
		  rxData[1] = 0x00;
		  rxData[2] = 0x00;
		  HAL_I2C_Master_Receive(&hi2c1, ADS1219_ADDR, rxData, 3, HAL_MAX_DELAY);
		  rawLoadCell1 = (rxData[0] << 16) | (rxData[1] << 8) | rxData[2];
		  rawData1[i] = rawLoadCell1 - LoadCell1_Offset;
//		  printf("%ld\r\n", rawLoadCell1);

		  // Configure ADS1115 MUX to measure AIN2/3 load cell
		  txData[0] = ADS1219_WREG;
		  txData[1] = 0x36;
		  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);
		  // Wait a bit to take the measurements
		  HAL_Delay(20);
		  // Read the measurement and store it
		  txData[0] = ADS1219_RDATA;
		  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 1, HAL_MAX_DELAY);
		  rxData[0] = 0x00;
		  rxData[1] = 0x00;
		  rxData[2] = 0x00;
		  HAL_I2C_Master_Receive(&hi2c1, ADS1219_ADDR, rxData, 3, HAL_MAX_DELAY);
		  rawLoadCell2 = (rxData[0] << 16) | (rxData[1] << 8) | rxData[2];
		  // Applying offset on load cell B based on if it overflowed or not
		  if (rawLoadCell2 > 16700000) rawData2[i] = rawLoadCell2 - LoadCell2_OffsetA;
		  else rawData2[i] = rawLoadCell2 - LoadCell2_OffsetB;
//		  printf("%ld\r\n", rawData2[i]);
	  }
	  avg1 = 0;
	  avg2 = 0;
	  for (int i = 0; i < 25; i++) {
		  avg1 += rawData1[i];
		  avg2 += rawData2[i];
	  }
	  avg1 = avg1/25; // Load cell A is mounted in opposite direction, so sign is flipped to get the same sign when summing both
	  avg2 = avg2/-25;

	  // Printing the voltage to the console
	  kilograms = rawLoadCell1 * 2.048 * 20.0 * 32 / 8388608.0;
	  printf("Load cell A: %ld counts / %.7f mV \r\n", (avg1), ((avg1)*0.512 / 8388608.0));

	  kilograms = rawLoadCell2 * 2.048 * 20.0 * 32 / 8388608.0;
	  printf("Load cell B: %ld counts / %.7f mV \r\n", (avg2), ((avg2)*0.512 / 8388608.0));
//	  printf("Scale measurement: %ld counts / %.7f mV \r\n\n", (avg1 + avg2), ((avg1 + avg2)*0.512 / 8388608.0));
	  printf("Scale measurement: %ld counts / %.5f kg \r\n\n", (avg1 + avg2), ((avg1 + avg2)*lsbToKg));

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
