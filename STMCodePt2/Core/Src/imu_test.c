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
#include <stdio.h>

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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

// LSM9DS1 I2C address for Accelerometer/Gyroscope
//#define LSM9DS1_AG_ADDR 0x6B << 1  // Shifted for 7-bit address; FIX: check this addy
#define LSM9DS1_ADDR_WRITE  (0x6A << 1)  // Device write register
#define LSM9DS1_ADDR_READ   ((0x6A << 1) | 0x01)  // Device read register
#define REG_ACCEL 0x28	// X acceleration data register
//ultrasonic defines
#define TRIGGER_PIN 1                  // Assume trigger is connected to PB6
#define ECHO_PIN 2                     // Assume echo is connected to PB7

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C2_Init(void);

float measure_distance(void);
extern void nano_wait(int);
void delay_us(uint32_t us);
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
  MX_USART6_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef ret;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //ultrasomic setup
  uint8_t buf[30];
  uint8_t config[2];
  //uint8_t accel_data[2]; moving to 3 axises
  uint8_t accel_data[2];
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;

//  while(1){
//
//	    	if (HAL_I2C_IsDeviceReady(&hi2c2, LSM9DS1_ADDR_WRITE, 100, 1000) != HAL_OK) {
//	    		strcpy((char*) buf, "Device not ready\r\n");
//	    		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
//	    		HAL_Delay(500);
//	    		continue;
//	    	}
//
//	    	config[0] = 0x20;
//	    	config[1] = 0x60;
//
//	    	if (HAL_I2C_Master_Transmit(&hi2c2, LSM9DS1_ADDR_WRITE, config, 2, 10000) != HAL_OK) {
//	    		strcpy((char*) buf, "Failed to configure\r\n");
//	    		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
//	    		HAL_Delay(500);
//	    		continue;
//	    	}
//	    	if (HAL_I2C_Mem_Read(&hi2c2, LSM9DS1_ADDR_READ, REG_ACCEL, I2C_MEMADD_SIZE_8BIT, accel_data, 2, HAL_MAX_DELAY) != HAL_OK) {
//	    		strcpy((char*) buf, "Failed to read data\r\n");
//	    		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
//	    		HAL_Delay(500);
//	    		continue;
//	    	}
//
//	    	accel_x = (int16_t)(accel_data[1] << 8 | accel_data[0]);
//	    	sprintf((char*) buf, "%d\r\n", accel_x);
//	    	HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
//
//	    	HAL_Delay(100);
//  }

  while (1) {
	  GPIOC->ODR |= (1 << 9);
	  GPIOB->ODR |= (1 << 12);
	  //delay_us(1000000); //1000000 is 6.01 seconds
	  HAL_Delay(500);
	  GPIOC->ODR &= ~(1 << 9);
	  GPIOB->ODR &= ~(1 << 12);
	  //delay_us(1000000);
	  HAL_Delay(500);
	  strcpy((char*) buf, "test\r\n");
	  //HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
	  HAL_Delay(500);

	  if (HAL_I2C_IsDeviceReady(&hi2c1, LSM9DS1_ADDR_WRITE, 100, 1000) != HAL_OK) {
		  strcpy((char*) buf, "Device not ready\r\n");
		  HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		  HAL_Delay(500);

	  	  //handle hal_busy if device not ready
	  	  //HAL_I2C_DeInit(&hi2c1);
	      delay_us(1000000); // Small delay to ensure proper deinit
	      HAL_I2C_Init(&hi2c1);
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
	  if (HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR_READ, REG_ACCEL, I2C_MEMADD_SIZE_8BIT, accel_data, 2, HAL_MAX_DELAY) != HAL_OK) {
		  strcpy((char*) buf, "Failed to read data\r\n");
		  HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		  HAL_Delay(500);
		  continue;
	  }

	  accel_x = 1;//(int16_t)(accel_data[1] << 8 | accel_data[0]);
	  accel_y = 2;//(int16_t)(accel_data[3] << 8 | accel_data[2]); // Y-axis
	  accel_z = 3;//(int16_t)(accel_data[5] << 8 | accel_data[4]); // Z-axis
	  int score = 0;
	  sprintf((char*) buf, "%d %d %d %d %d %d %d\r\n", score, accel_x, accel_y, accel_z, accel_x, accel_y, accel_z);
	  //sprintf((char*) buf, "%d\r\n", accel_x);
	  HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);



	  HAL_Delay(100);

	  //ultrasonic shot sensing + sending uart
//  while(1){
//	  int send_data = 0;
//	  float distance = measure_distance();
//	  strcpy((char*) buf, "Device not ready\r\n");
//	  if(distance <= 1.5){
//		  //turn on
//		  //GPIOC->ODR |= (1 << 9);
//		  GPIOB->ODR |= (1 << 12);
//		  send_data = 1;
//		  delay_us(100000);
//	  }
//	  else{
//		  //turn off
//		  GPIOB->ODR &= ~(1 << 12);
//	  }
//	  //HAL_Delay(500);
//	  if(send_data == 1){
//		  strcpy((char*) buf, "Score!\r\n");
//		  HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
//		  //send IMU data for backboard/rim or swish
//		  GPIOB->ODR |= (1 << 12);
//		  delay_us(100000);
//
//		  //sent = 1
//		  //send_data = 0;
//	  }
//	  else{
//		  strcpy((char*) buf, "0\r\n");
//		  HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
//		  //turn off
//		  GPIOB->ODR &= ~(1 << 12);
//	  }
//	  delay_us(10000);

	  //timing test
//	  GPIOB->ODR |= (1 << 12);
//	  delay_us(1000000); //1000000 is 6.01 seconds
//	  GPIOB->ODR &= ~(1 << 12);
//	  delay_us(1000000);

  }


  //while (1)
  //{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //}
  /* USER CODE END 3 */
}
//delay function in us using nano_wait assembly code (from 362 credit to Niraj Menon)
void delay_us(uint32_t us)
{
    nano_wait(us*1000); //us->ns
}
//dunction to calculate distance to nearest object - used to see if ball passed by
float measure_distance(void)  //add to cube
{
    //initilaize time to 0
    float time = 0.0;

    // Send 10us pulse to trigger
    // GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
    // delay_us(10);
    // GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low

    GPIOA->BSRR = 1 << TRIGGER_PIN;         //set PA1 high
    delay_us(10);
    GPIOA->BRR = 1 << TRIGGER_PIN;         // Set PA1 low

    //wait for echo pin to go high
    // while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 0){
    //     GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
    //     delay_us(10);
    //     GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low
    // }


    while ((((GPIOA->IDR) >> ECHO_PIN) & 0x1) == 0){
        GPIOA->BSRR = 1 << TRIGGER_PIN;         //set PA1 high
        delay_us(10);
        GPIOA->BRR = 1 << TRIGGER_PIN;         // Set PA2 low
    }

    //now measure time it is high and that indicates how long wave travled, menaing how far it traveled before hitting

    // Measure how long echo pin stays high
    // while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 1) {
    //     time++;
    //     delay_us(1);
    // }

    while ((((GPIOA->IDR) >> ECHO_PIN) & 0x1) == 1) {
        time+=1.0;
        delay_us(1);
    }

    float scalar = 1.0;
    // Convert time to distance in cm: (us/1e6)= 1s * 343 = m, * 100 = cm: (time / 1E6) * 343 gives m / s * 100 gives cm/s
    float distance_func = ((time * 0.0343) / 2.0) * scalar; //use scalar if needed
    return distance_func;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
