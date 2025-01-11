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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// XR20M slave address & internal register addresses
#define XR20M_ADDR 0x60 << 1
#define BUF_RHR_THR_DLL 		0x00
#define BUF_DLM_IER 			0x01
#define BUF_DLD_ISR_FCR_EFR 	0x02
#define BUF_LCR 				0x03
#define BUF_MCR_XON1 			0x04
#define BUF_LSR_XON2			0x05
#define BUF_MSR_TCR_XOFF1		0x06
#define BUF_SPR_TLR_XOFF2		0x07
#define BUF_TXLVL				0x08
#define BUF_RXLVL				0x09
#define BUF_IODIR				0x0A
#define BUF_IOSTATE				0x0B
#define BUF_IOINTEN				0x0C
#define BUF_IOCTRL				0x0E
#define BUF_EFCR				0x0F

// Barcode length constant (standard for grocery stores is 12 numbers + end-of-line character)
#define BARCODE_LEN 			13

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Global variables - used for ISRs to raise flags
bool bufferDataFlag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void BufferRegisterWrite(uint8_t *txBytes, uint8_t regAddr, uint8_t regData, bool selChB);
void BufferRegisterRead(uint8_t *rxBytes, uint8_t regAddr, uint8_t numBytes, bool selChB);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	char rxData[BARCODE_LEN + 1] = {0};
	uint8_t rxI2C[64] = {0};
	uint8_t txI2C[64] = {0};

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	memset(rxData, 0 ,sizeof(rxData));

	// Configuring the XR20M1172 buffer on startup
	// Starting with line control register (LCR) to get access to enhanced features for config
	BufferRegisterWrite(txI2C, BUF_LCR, 0xBF, 0);
	BufferRegisterWrite(txI2C, BUF_LCR, 0xBF, 1);
	// Enabling enhanced features for both channels
	BufferRegisterWrite(txI2C, BUF_DLD_ISR_FCR_EFR, 0x10, 0);
	BufferRegisterWrite(txI2C, BUF_DLD_ISR_FCR_EFR, 0x10, 1);

	// Configure UART with LCR, also gives access to divisor registers to be configured
	BufferRegisterWrite(txI2C, BUF_LCR, 0x83, 0); // Channel A configured to 8-bit word length, no parity, 1 stop bit, divisor registers enabled
	BufferRegisterWrite(txI2C, BUF_LCR, 0x83, 1); // Channel B configured to 8-bit word length, no parity, 1 stop bit, divisor registers enabled
	// Configure clock divisors - set baud rate for channel A and B to 9600 (based on 24MHz crystal)
	BufferRegisterWrite(txI2C, BUF_RHR_THR_DLL, 0x9C, 0);
	BufferRegisterWrite(txI2C, BUF_DLD_ISR_FCR_EFR, 0x04, 0);
	BufferRegisterWrite(txI2C, BUF_DLM_IER, 0x00, 0);
	BufferRegisterWrite(txI2C, BUF_RHR_THR_DLL, 0x9C, 1);
	BufferRegisterWrite(txI2C, BUF_DLD_ISR_FCR_EFR, 0x04, 1);
	BufferRegisterWrite(txI2C, BUF_DLM_IER, 0x00, 1);
	// Re-configure LCR to change access from divisor registers to other config registers
	BufferRegisterWrite(txI2C, BUF_LCR, 0x03, 0);
	BufferRegisterWrite(txI2C, BUF_LCR, 0x03, 1);

	// Enable FIFO mode for both channels
	BufferRegisterWrite(txI2C, BUF_DLD_ISR_FCR_EFR, 0x01, 0);
	BufferRegisterWrite(txI2C, BUF_DLD_ISR_FCR_EFR, 0x01, 1);
	// Enable the trigger level registers
	BufferRegisterWrite(txI2C, BUF_MCR_XON1, 0x04, 0);
	BufferRegisterWrite(txI2C, BUF_MCR_XON1, 0x04, 1);
	// Set the trigger level to 12 characters (used for interrupt generation)
	BufferRegisterWrite(txI2C, BUF_SPR_TLR_XOFF2, 0x03, 0);
	BufferRegisterWrite(txI2C, BUF_SPR_TLR_XOFF2, 0x03, 1);
	// Enable the interrupt for the receive buffer trigger level
	BufferRegisterWrite(txI2C, BUF_DLM_IER, 0x01, 0);
	BufferRegisterWrite(txI2C, BUF_DLM_IER, 0x01, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
	  // Wait for message over UART (blocking mode)
//	  if (HAL_UART_Receive(&huart1, rxData, sizeof(rxData), HAL_MAX_DELAY) == HAL_OK) {
//		  // Print received message
//		  printf("Received: %s\n", rxData);
//
//		  // Clear memory buffer for next message
//		  memset(rxData, 0 ,sizeof(rxData));
//	  }
//		HAL_Delay(1000);
//		printf("Hello world!\r\n");

		// Check if an interrupt was received indicating data is available from the buffer
		if (bufferDataFlag) {
			bufferDataFlag = 0; // Reset the flag

			// Check the amount stored in each buffer - if a full barcode is stored (12 characters + end-of-line character), read it from the FIFO buffer
			BufferRegisterRead(rxI2C, BUF_RXLVL, 1, 0);
			// Keep reading barcodes as long as the buffer contains enough characters
			while (rxI2C[0] >= BARCODE_LEN) {
				BufferRegisterRead(rxI2C, BUF_RHR_THR_DLL, BARCODE_LEN, 0);
				// Format the recevied data into a character array
				for (int i = 0; i < BARCODE_LEN; i++) {
					rxData[i] = (char)rxI2C[i];
				}
				rxData[BARCODE_LEN] = '\0'; // Add null-terminator to end the string

				printf("Received barcode from scanner A: %s\n", rxData);
				// Re-read the buffer level to check if any barcodes remaining
				BufferRegisterRead(rxI2C, BUF_RXLVL, 1, 0);
			}
			BufferRegisterRead(rxI2C, BUF_RXLVL, 1, 1);
			while (rxI2C[0] >= BARCODE_LEN) {
				BufferRegisterRead(rxI2C, BUF_RHR_THR_DLL, BARCODE_LEN, 1);
				for (int i = 0; i < BARCODE_LEN; i++) {
					rxData[i] = (char)rxI2C[i];
				}
				rxData[BARCODE_LEN] = '\0';

				printf("Received barcode from scanner B: %s\n", rxData);
				// Re-read the buffer level to check if any barcodes remaining
				BufferRegisterRead(rxI2C, BUF_RXLVL, 1, 1);
			}
		}

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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * Helper function to handle writing to one of the buffer's internal UART registers
 * @param txBytes Pointer to 8-bit integer array to transmit over I2C
 * @param regAddr Internal register address to write to
 * @param regData Data to write to the internal register
 * @param selChB Boolean for selecting channel A or B to write to
 */
void BufferRegisterWrite(uint8_t *txBytes, uint8_t regAddr, uint8_t regData, bool selChB) {
	// Construct address by shifting the register address and channel select appropriately
	uint8_t addr = (regAddr << 3) | (selChB << 1);

	// Send an I2C write to the buffer's slave address, then write the data to the register address
	txBytes[0] = addr;
	txBytes[1] = regData;
	HAL_I2C_Master_Transmit(&hi2c1, XR20M_ADDR, txBytes, 2, HAL_MAX_DELAY);
}

/**
 * Helper function to handle reading from one of the buffer's internal UART registers
 * @param rxBytes Pointer to 8-bit integer array to receive I2C data
 * @param regAddr Internal register address to read from
 * @param numBytes Number of bytes to read from the register
 * @param selChB Boolean for selecting channel A or B to write to
 */
void BufferRegisterRead(uint8_t *rxBytes, uint8_t regAddr, uint8_t numBytes, bool selChB) {
	// Construct address by shifting the register address and channel select appropriately
	uint8_t addr = (regAddr << 3) | (selChB << 1);

	// Send an I2C write to the buffer's slave address in order to indicate which register is being read from
	uint8_t txByte[1] = {addr};
	HAL_I2C_Master_Transmit(&hi2c1, XR20M_ADDR, txByte, 1, HAL_MAX_DELAY);
	// Read the number of bytes from the register
	HAL_I2C_Master_Receive(&hi2c1, XR20M_ADDR, rxBytes, numBytes, HAL_MAX_DELAY);
}

//void EXTI4_IRQHandler(void) {
//    // Clear the EXTI interrupt flag
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
//    // Raise a flag to indicate the buffer has data to be received
//    bufferDataFlag = 1;
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) {
    	// Raise a flag to indicate the buffer has data to be received
        bufferDataFlag = 1;
    }
}

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
