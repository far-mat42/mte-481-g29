/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "math.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADS1219_ADDR 		0x45 << 1

#define ADS1219_RDATA 		0x10
#define ADS1219_WREG 		0x40
#define ADS1219_STARTSYNC 	0x08

#define WEIGHT_THRESHOLD	0.040

#define TIM1_ARR 			19999

// Servos 1 and 3 on pitch axis, servos 2 and 4 on yaw axis
#define SERVO_DUTY_MIN 		650.0
#define SERVO_DUTY_BW		1700.0
#define SERVO_ANGLE_MAX		180

#define SERVO1_START 		0
#define SERVO1_END			93
#define SERVO1_MAX_SPEED	12

#define SERVO2_START		30
#define SERVO2_END			160
#define SERVO2_SPEED		0.75

#define SERVO3_START 		0
#define SERVO3_END			93
#define SERVO3_MAX_SPEED	12

#define SERVO4_START		30
#define SERVO4_END			160
#define SERVO4_SPEED		0.75

#define FINAL_SWEEP_COUNTS	20

#define MAX_BARCODE_LEN		20
#define MAX_BARCODES		100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
// Flags for determining which step of the automation process to run
bool servoFlag = false;
bool scannersEnable = false;
bool resetScanners = true;

bool recordWeightFlag = false;
bool scaleEnable = true;

// Placeholder/variables for handling UART transmissions
uint8_t uartRxBuffer[MAX_BARCODES][MAX_BARCODE_LEN] = {0};
uint8_t uartTxBuffer[MAX_BARCODE_LEN + 2] = {0};
uint8_t txLen = 0;
volatile uint8_t barcodeIndex = 0;
volatile uint8_t charIndex = 0;
uint8_t rxChar;

// Local database of known barcodes and their corresponding weights
// In order: Lime juice, cereal box, pasta bag, "milk"
uint8_t codesDatabase[5][MAX_BARCODE_LEN] = {"065912581221", "064100147638", "067800007141", "066800004020"};
uint16_t weightsDatabase[5] = {498, 463, 807, 1904};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void SetServoAngle (uint32_t channel, uint8_t angle);
uint8_t inList(uint8_t *string, uint8_t (*strList)[MAX_BARCODE_LEN]);
float max(float a, float b);
uint32_t maxInt(uint32_t a, uint32_t b);

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initializing variables for servo motor control
  uint8_t servoAng1 = SERVO1_START;
  uint8_t servoDir1 = 1;
  uint8_t servoSpeed1 = 0;
  float servoSpeedCalc1 = 0;

  float servoAng2 = SERVO2_START;
  uint8_t servoDir2 = 1;

  uint8_t servoAng3 = SERVO3_START;
  uint8_t servoDir3 = 1;
  uint8_t servoSpeed3 = 0;
  float servoSpeedCalc3 = 0;

  float servoAng4 = SERVO2_START;
  uint8_t servoDir4 = 1;

  bool finalSweep = false;
  uint8_t finalSweepCount =  0;

  // Buffers for TX/RX data of I2C bus
  uint8_t txData[3] = {0};
  uint8_t rxData[3] = {0};

  // Variables and arrays for storing load cell readings
  int32_t rawLoadCell1 = 0;
  int32_t rawLoadCell2 = 0;

  int32_t LoadCell1_Offset = 0;
  int32_t LoadCell2_OffsetA = 0;
  int32_t LoadCell2_OffsetB = 0;

  int32_t rawData1[25] = {0};
  int32_t rawData2[25] = {0};
  int32_t avg1 = 0;
  int32_t avg2 = 0;

  float lsbToKg = 0.000256;
  float kilograms = 0;
  float prevKilograms = 0;
  float recordedWeight = 0;
  float totalWeight = 0;
  char weightMsg[20] = {0};

  // Arrays to track barcodes for items currently in the cart and prioritize scanned barcodes
  uint8_t cartCodes[MAX_BARCODES][MAX_BARCODE_LEN] = {0};
  uint8_t cartNumItems = 0;
  uint8_t priorities[MAX_BARCODES] = {0};
  uint8_t sendCodes[MAX_BARCODES] = {0};
  float remainingWeight = 0;
  float itemWeight = 0;

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
	  HAL_Delay(15);
	  // Read the measurement and store it
	  txData[0] = ADS1219_RDATA;
	  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 1, HAL_MAX_DELAY);
	  rxData[0] = 0x00;
	  rxData[1] = 0x00;
	  rxData[2] = 0x00;
	  HAL_I2C_Master_Receive(&hi2c1, ADS1219_ADDR, rxData, 3, HAL_MAX_DELAY);
	  rawLoadCell1 = (rxData[0] << 16) | (rxData[1] << 8) | rxData[2];
	  avg1 += rawLoadCell1;

	  // Configure ADS1115 MUX to measure AIN2/3 load cell
	  txData[0] = ADS1219_WREG;
	  txData[1] = 0x36;
	  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);
	  // Wait a bit to take the measurements
	  HAL_Delay(15);
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

//  printf("Calculated the following offsets: %ld for cell A, %ld for cell B\r\n", LoadCell1_Offset, LoadCell2_OffsetA);

  // Startup for servo motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  SetServoAngle(TIM_CHANNEL_1, SERVO1_START);
  SetServoAngle(TIM_CHANNEL_2, SERVO2_START);
  SetServoAngle(TIM_CHANNEL_3, SERVO3_START);
  SetServoAngle(TIM_CHANNEL_4, SERVO4_START);
  HAL_Delay(3000);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Reset position of barcode scanners if flagged to do so
	  if (resetScanners) {
		  SetServoAngle(TIM_CHANNEL_1, SERVO1_START);
		  SetServoAngle(TIM_CHANNEL_2, SERVO2_START);
		  SetServoAngle(TIM_CHANNEL_3, SERVO3_START);
		  SetServoAngle(TIM_CHANNEL_4, SERVO4_START);

		  servoAng1 = SERVO1_START;
		  servoDir1 = 1;
		  servoSpeed1 = 0;
		  servoSpeedCalc1 = 0;
		  servoAng2 = SERVO2_START;
		  servoDir2 = 1;
		  servoAng3 = SERVO3_START;
		  servoDir1 = 1;
		  servoSpeed3 = 0;
		  servoSpeedCalc3 = 0;
		  servoAng4 = SERVO4_START;
		  servoDir4 = 1;
		  finalSweep = false;
		  finalSweepCount = 0;

		  resetScanners = false;
		  scaleEnable = true;

		  // Disable any reception from barcode scanners over UART
		  HAL_UART_DeInit(&huart1);
		  HAL_UART_DeInit(&huart6);

//		  printf("\nScan complete, resetting scanner position...\r\n");
	  }
	  // Sweep the barcode scanners across the cart
	  if (scannersEnable) {
		  // Update servo positions when timer interrupt flag raised
		  if (servoFlag) {
			  servoFlag = 0;

			  // Servo 1 & 3 (pitch axis): Update speed using a quadratic equation
			  servoSpeedCalc1 = SERVO1_MAX_SPEED*(-0.000395501*servoAng1*servoAng1 + 0.0379681*servoAng1 + 0.0896552);
			  servoSpeed1 = ((uint8_t)servoSpeedCalc1);
			  servoAng1 += servoSpeed1*servoDir1;

			  servoSpeedCalc3 = SERVO3_MAX_SPEED*(-0.000395501*servoAng3*servoAng3 + 0.0379681*servoAng3 + 0.0896552);
			  servoSpeed3 = ((uint8_t)servoSpeedCalc3);
			  servoAng3 += servoSpeed3*servoDir3;
			  // Switch direction if max/min value reached
			  if (servoAng1 >= SERVO1_END) servoDir1 = -1;
			  if (servoAng1 <= SERVO1_START) servoDir1 = 1;
			  if (servoAng3 >= SERVO3_END) servoDir3 = -1;
			  if (servoAng3 <= SERVO3_START) servoDir3 = 1;

			  SetServoAngle(TIM_CHANNEL_1, servoAng1);
			  SetServoAngle(TIM_CHANNEL_3, servoAng3);

			  // Servo 2 & 4 (yaw axis): Increment position by speed value
			  if (servoAng2 <= SERVO2_END) servoAng2 += SERVO2_SPEED*servoDir2;
			  if (servoAng4 <= SERVO4_END) servoAng4 += SERVO4_SPEED*servoDir4;
			  // If max value reached on yaw axis, reset the scanners (sweep is complete) and re-enable the scale
//			  if (servoAng2 >= SERVO2_END || servoAng4 >= SERVO4_END) {
//				  scannersEnable = false;
//				  resetScanners = true;
//			  }
			  // If max value reached on yaw axis, do one final sweep then reset the scanners and re-enable the scale
			  if (servoAng2 >= SERVO2_END || servoAng4 >= SERVO4_END) {
				  finalSweep = true;
			  }
			  if (finalSweep) {
				  finalSweepCount += 1;
			  }
			  if (finalSweepCount >= FINAL_SWEEP_COUNTS) {
				  scannersEnable = false;
				  resetScanners = true;

				  // Output all barcodes received
//				  if (barcodeIndex > 0) {
////					  printf("\nReceived %d barcodes:\r\n", barcodeIndex);
//					  for (int i = 0; i < barcodeIndex; i++) {
//						  for (int j = 0; j < MAX_BARCODE_LEN; j++) {
//							  if (uartRxBuffer[i][j] == '\0') {
//								  txLen = j-1;
//								  break;
//							  }
//							  uartTxBuffer[j] = uartRxBuffer[i][j];
//						  }
//						  HAL_UART_Transmit(&huart2, uartTxBuffer, (txLen + 1), HAL_MAX_DELAY);
//						  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
//					  }
//					  barcodeIndex = 0;
//				  }
				  // Loop through all scanned barcodes, assign priorities based on likelihood of it being a new item
				  remainingWeight = recordedWeight; // Keep track of remaining weight from measured value
				  for (int i = 0; i < barcodeIndex; i++) {
					  // Priority 1: Item exists in database but is not in cart
					  if (inList(uartRxBuffer[i], codesDatabase) && !inList(uartRxBuffer[i], cartCodes)) {
						  priorities[i] = 1;
						  // Append this item to the end of the list of cart codes
						  strncpy(cartCodes[cartNumItems], uartRxBuffer[i], MAX_BARCODE_LEN);
						  cartNumItems++;
						  // Subtract the weight of this product from the remaining weight
						  remainingWeight -= weightsDatabase[inList(uartRxBuffer[i], codesDatabase) - 1];
					  }
					  // Priority 2 or 3: Item exists in database and 1 or more of that item is in the cart, higher priority to items whose weight exactly matches measured weight (within 5% tolerance)
					  else if (inList(uartRxBuffer[i], codesDatabase) && inList(uartRxBuffer[i], cartCodes)) {
						  itemWeight = weightsDatabase[inList(uartRxBuffer[i], codesDatabase) - 1];
						  if (recordedWeight > (itemWeight - max(1000*WEIGHT_THRESHOLD, 0.05*itemWeight)) && recordedWeight < (itemWeight + max(1000*WEIGHT_THRESHOLD, 0.05*itemWeight))) priorities[i] = 2;
						  else priorities[i] = 3;
					  }
					  // If none of the above apply, reset the priority for this index
					  else priorities[i] = 0;
				  }
				  // Loop through all priorities (lower numbers first) and decide whether or not to send the barcode
				  for (int i = 0; i < barcodeIndex; i++) {
					  if (priorities[i] == 1) {
						  // All priority 1 items are sent no matter what
						  sendCodes[i] = 1;
					  }
				  }
				  for (int i = 0; i < barcodeIndex; i++) {
					  // Only send priority 2 items if the remaining weight is sufficient
					  if (priorities[i] == 2) {
						  itemWeight = weightsDatabase[inList(uartRxBuffer[i], codesDatabase) - 1];
						  if (remainingWeight > (itemWeight + max(1000*WEIGHT_THRESHOLD, itemWeight*0.05))) {
							  sendCodes[i] = 1;
							  // Append this item to the end of the list of cart codes
							  strncpy(cartCodes[cartNumItems], uartRxBuffer[i], MAX_BARCODE_LEN);
							  cartNumItems++;
							  // Subtract the weight of this product from the remaining weight
							  remainingWeight -= itemWeight;
						  }
					  }
				  }
				  for (int i = 0; i < barcodeIndex; i++) {
					  // Only send priority 3 items if the remaining weight is sufficient
					  if (priorities[i] == 3) {
						  itemWeight = weightsDatabase[inList(uartRxBuffer[i], codesDatabase) - 1];
						  if (remainingWeight > (itemWeight + max(1000*WEIGHT_THRESHOLD, itemWeight*0.05))) {
							  sendCodes[i] = 1;
							  // Append this item to the end of the list of cart codes
							  strncpy(cartCodes[cartNumItems], uartRxBuffer[i], MAX_BARCODE_LEN);
							  cartNumItems++;
							  // Subtract the weight of this product from the remaining weight
							  remainingWeight -= itemWeight;
						  }
					  }
				  }

				  // Transmit all the codes flagged to be sent to the ESP32
				  for (int i = 0; i < barcodeIndex; i++) {
					  if (sendCodes[i] == 1) {
						  // Transmit start of barcode character
						  HAL_UART_Transmit(&huart2, (uint8_t*)"C", 1, HAL_MAX_DELAY);
						  for (int j = 0; j < MAX_BARCODE_LEN; j++) {
							  uartTxBuffer[j] = uartRxBuffer[i][j];
							  if (uartRxBuffer[i][j] == '\0') {
								  txLen = j;
								  break;
							  }
						  }
						  HAL_UART_Transmit(&huart2, uartTxBuffer, (txLen + 1), HAL_MAX_DELAY);
					  }
				  }
				  // Send a dummy barcode in the event where no barcodes were found
				  HAL_UART_Transmit(&huart2, (uint8_t*)"C000000000000", 1, HAL_MAX_DELAY);
				  // Transmit end of transmission character
				  HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, HAL_MAX_DELAY);

				  barcodeIndex = 0; // Reset barcode index
			  }

			  SetServoAngle(TIM_CHANNEL_2, servoAng2);
			  SetServoAngle(TIM_CHANNEL_4, servoAng4);
		  }
	  }
	  // Average 25 measurements (over approx. 1 second) to get the reading from each load cell.
	  if (scaleEnable) {
		  prevKilograms = kilograms;
		  HAL_Delay(100);
		  for (int i = 0; i < 25; i++) {
			  // Configure ADS1219 MUX to measure AIN0/1 load cell
			  txData[0] = ADS1219_WREG;
			  txData[1] = 0x16;
			  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);
			  // Wait a bit to take the measurements
			  HAL_Delay(15);
			  // Read the measurement and store it
			  txData[0] = ADS1219_RDATA;
			  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 1, HAL_MAX_DELAY);
			  rxData[0] = 0x00;
			  rxData[1] = 0x00;
			  rxData[2] = 0x00;
			  HAL_I2C_Master_Receive(&hi2c1, ADS1219_ADDR, rxData, 3, HAL_MAX_DELAY);
			  rawLoadCell1 = (rxData[0] << 16) | (rxData[1] << 8) | rxData[2];
			  rawData1[i] = rawLoadCell1 - LoadCell1_Offset;

			  // Configure ADS1115 MUX to measure AIN2/3 load cell
			  txData[0] = ADS1219_WREG;
			  txData[1] = 0x36;
			  HAL_I2C_Master_Transmit(&hi2c1, ADS1219_ADDR, txData, 2, HAL_MAX_DELAY);
			  // Wait a bit to take the measurements
			  HAL_Delay(15);
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
		  }
		  avg1 = 0;
		  avg2 = 0;
		  for (int i = 0; i < 25; i++) {
			  avg1 += rawData1[i];
			  avg2 += rawData2[i];
		  }
		  avg1 = avg1/-25;
		  avg2 = avg2/25; // Load cell B is mounted in opposite direction, so sign is flipped to get the same sign when summing both
		  kilograms = (avg1 + avg2)*lsbToKg;
//		  printf("\nMeasured weight of %.5f kilograms\r\n", kilograms);

		  // Save the weight if required
		  if (recordWeightFlag == true) {
			  recordedWeight = kilograms;
			  totalWeight += recordedWeight;
			  // Stop the scale and start the barcode scanners
			  scannersEnable = true;
			  scaleEnable = false;
			  recordWeightFlag = false;
//			  printf("Weight of product added to cart: %.5f kilograms\r\nTotal cart weight: %.5f\r\n", recordedWeight, totalWeight);
			  // Send weight information to ESP32
//			  printf("W%.3f\0\r", recordedWeight);
			  snprintf(weightMsg, sizeof(weightMsg), "W%.3f", recordedWeight);
			  HAL_UART_Transmit(&huart2, (uint8_t*)weightMsg, strlen(weightMsg), HAL_MAX_DELAY);
			  // Transmit end of transmission character
			  HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, HAL_MAX_DELAY);

			  // Re-initialize and begin receiving from both UART channels
			  HAL_UART_Init(&huart1);
			  HAL_UART_Init(&huart6);
			  HAL_UART_Receive_IT(&huart1, &rxChar, 1);
			  HAL_UART_Receive_IT(&huart6, &rxChar, 1);
		  }

		  // Check if current weight is greater than previous weight by the established threshold (25g)
		  if (kilograms > (totalWeight + WEIGHT_THRESHOLD) && recordWeightFlag == false) {
			  recordWeightFlag = true;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
/**
 * Function to set the angle of a servo motor by calculating and setting the required PWM duty cycle
 * @param channel Which PWM channel to set (i.e. which servo motor to set the angle for)
 * @param angle Angle (between 0º and 180º) to set the servo motor to
 */
void SetServoAngle (uint32_t channel, uint8_t angle) {
	if (angle >= SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;
	uint16_t dutyCycle = ((double)angle / 180.0)*SERVO_DUTY_BW + SERVO_DUTY_MIN;

	switch(channel) {
		case TIM_CHANNEL_1:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyCycle);
			break;
		case TIM_CHANNEL_2:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyCycle);
			break;
		case TIM_CHANNEL_3:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);
		case TIM_CHANNEL_4:
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, dutyCycle);
		default:
			break;
	}
}

/**
 * Function to check if a string exists in a list of strings
 * @param string String to look for in the list
 * @param strList List of strings to check
 * @retval Position where item was found (0 if not found) (not to be confused with index)
 */
uint8_t inList(uint8_t *string, uint8_t (*strList)[MAX_BARCODE_LEN]) {
	for (int i = 0; i < sizeof(strList); i++) {
		if (strcmp(string, strList[i]) == 0) return i+1;
	}
	return false;
}

uint32_t maxInt(uint32_t a, uint32_t b) {
    if (a > b) return a;
    return b;
}

float max(float a, float b) {
    if (a > b) return a;
    return b;
}

void TIM2_IRQHandler(void) {
  servoFlag = 1;
  HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {  // Check if it's USART1
        if (rxChar == '\n' || rxChar == '\r') {  // End of message determined by newline or return
            uartRxBuffer[barcodeIndex][charIndex] = '\0';  // Null-terminate the message
            barcodeIndex++;  // Move to the next barcode slot
            charIndex = 0;  // Reset character index

            if (barcodeIndex >= MAX_BARCODES) {  // Prevent buffer overflow
                barcodeIndex = 0;  // Overwrite old messages (circular buffer)
            }
        } else {  // Store character
            if (charIndex < MAX_BARCODE_LEN - 1) {  // Prevent overflow
                uartRxBuffer[barcodeIndex][charIndex++] = rxChar;
            }
        }

        // Restart reception for the next character
        HAL_UART_Receive_IT(&huart1, &rxChar, 1);
    }
    if (huart->Instance == USART6) {  // Check if it's USART6
        if (rxChar == '\n' || rxChar == '\r') {  // End of message determined by newline or return
        	uartRxBuffer[barcodeIndex][charIndex] = '\0';  // Null-terminate the message
            barcodeIndex++;  // Move to the next barcode slot
            charIndex = 0;  // Reset character index

            if (barcodeIndex >= MAX_BARCODES) {  // Prevent buffer overflow
                barcodeIndex = 0;  // Overwrite old messages (circular buffer)
            }
        } else {  // Store character
            if (charIndex < MAX_BARCODE_LEN - 1) {  // Prevent overflow
            	uartRxBuffer[barcodeIndex][charIndex++] = rxChar;
            }
        }

        // Restart reception for the next character
        HAL_UART_Receive_IT(&huart6, &rxChar, 1);
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
