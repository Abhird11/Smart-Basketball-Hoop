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
#include <string.h>

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
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

// LSM9DS1 I2C address for Accelerometer/Gyroscope
//#define LSM9DS1_AG_ADDR 0x6B << 1  // Shifted for 7-bit address; FIX: check this addy
#define LSM9DS1_ADDR_WRITE  (0x6A << 1)  // Device write register
#define LSM9DS1_ADDR_READ   ((0x6A << 1) | 0x01)  // Device read register
#define REG_ACCEL 0x28	// X acceleration data register
//ultrasonic defines
#define TRIGGER_PIN 6                   // Assume trigger is connected to PB6
#define ECHO_PIN 7                      // Assume echo is connected to PB7

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART5_UART_Init(void);
void setup_gpio(void);
uint32_t measure_distance(void);
/* USER CODE BEGIN PFP */
// Function prototypes
//void LSM9DS1_Init(void);
//void LSM9DS1_ReadAccel(int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);

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
  uint8_t buf[30];
  uint8_t config[2];
  //uint8_t accel_data[2]; moving to 3 axises
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;

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
  MX_I2C1_Init();
  MX_USART5_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef ret;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //ultrasomic setup
  //setup gpio for ultrasonic
  setup_gpio(); //calls initc //add to cube from here removing usart
  int max = 0;    //track max distance
  int min = 1000; //track min distance

  while (1) {
	  char* score;
      //get the distance
      uint32_t distance = measure_distance();

      if(distance > max){
          //track max distance
          max = distance;
      }
      if(distance < min){
          //track min distance
          min = distance;
      }

      if(distance == 0){
          //sees ball/object close to it
    	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  // Turn LED on
          score = "score: 1"; //we scored
      }
      else{
          //does not see ball/object close to it
    	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);  // Turn LED off
          score = "score: 0"; //did not score
      }

      //send scoring info
      //char* data = score;
      //while(*data) {
          //send_data(*data++); //'score or no score'
      //}

      HAL_Delay(10);
  }

  //while (1)
  //{
    /* USER CODE END WHILE */
	/*
	if (HAL_I2C_IsDeviceReady(&hi2c1, LSM9DS1_ADDR_WRITE, 100, 1000) != HAL_OK) {
		strcpy((char*) buf, "Device not ready\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);
		continue;
	}

	config[0] = 0x20;
	config[1] = 0x60;

	if (HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_ADDR_WRITE, config, 2, 10000) != HAL_OK) {
		strcpy((char*) buf, "Failed to configure\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);
		continue;
	}
	//read 6 bytes (X, Y, Z data) starting from OUT_X_L_XL (REG_ACCEL)
	uint8_t accel_data[6];
	if (HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR_READ, REG_ACCEL, I2C_MEMADD_SIZE_8BIT, accel_data, 6, HAL_MAX_DELAY) != HAL_OK) {
		strcpy((char*) buf, "Failed to read data\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);
		continue;
	}

	//combine the high and low bytes for each axis
	accel_x = (int16_t)(accel_data[1] << 8 | accel_data[0]);
	accel_y = (int16_t)(accel_data[3] << 8 | accel_data[2]);
	accel_z = (int16_t)(accel_data[5] << 8 | accel_data[4]);

	//OPTIONAL: analze data (rim/backboard detection algorithm) on STM before packing and sending to Laptop
	//can use timing (which changed first) and magnitude to decide which it hit (or both)
	//might be able to use axises to see if hitting rim caused backboard move or vice versa

	//pack acceleration values
	//sprintf((char*) buf, "%d\r\n", accel_x); moving to 3 axises
	sprintf((char*) buf, "X: %d, Y: %d, Z: %d\r\n", accel_x, accel_y, accel_z);
	HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  // Turn LED on


	HAL_Delay(100);
	*/

    /* USER CODE BEGIN 3 */
  //}
  /* USER CODE END 3 */
}
//config GPIO clock and pins
void setup_gpio(void)  //add to cube
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock

    // Configure PB6 and PB15 as output (trigger)
    GPIOB->MODER &= 0x3FFFCFFF; //clear
    GPIOB->MODER |= 0x40001000; //set

    // Configure PB7 as input (echo)
    GPIOB->MODER &= 0xFFFF3FFF;
}
//function to calculate distance to nearest object - used to see if ball passed by
uint32_t measure_distance(void)  //add to cube
{
    //initilaize time to 0
    uint32_t time = 0;

    // Send 10us pulse to trigger
    GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
    HAL_Delay(0.01);
    GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low

    //wait for echo pin to go high
    while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 0){
        GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
        HAL_Delay(0.01);
        GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low
    }

    //now measure time it is high and that indicates how long wave travled, menaing how far it traveled before hitting

    // Measure how long echo pin stays high
    while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 1) {
        time++;
        HAL_Delay(0.001);
    }

    float scalar = 1;

    float result = ((time * 0.0343) / 2.0) * scalar;
    // Convert time to distance in cm: (us/1e6)= 1s * 343 = m, * 100 = cm: (time / 1E6) * 343 gives m / s * 100 gives cm/s
    return result; //use scalar if needed
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
