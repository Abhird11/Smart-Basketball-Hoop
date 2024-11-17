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
#define TRIGGER_PIN 1                  // Assume trigger is connected to PB6
#define ECHO_PIN 2                     // Assume echo is connected to PB7
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART5_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
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
  uint8_t BUFFER_SIZE = 60;
  uint8_t buf[BUFFER_SIZE];
  uint8_t config[2];
  uint8_t accel_data_1[6];
  uint8_t accel_data_2[6];
  int16_t accel_x_rim;
  int16_t accel_y_rim;
  int16_t accel_z_rim;
  int16_t accel_rim_mag;
  int16_t accel_x_back;
  int16_t accel_y_back;
  int16_t accel_z_back;
  int16_t accel_back_mag;
  float distance;
  char rim_flag;
  char back_flag;
  char dist_flag;
  float DIST_THRESHOLD;
  int16_t RIM_THRESHOLD;
  int16_t BACK_THRESHOLD;
  float SHOT_TIME;
  float dist_time;
  float rim_time;
  float back_time;

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
  MX_I2C2_Init();
  MX_USART6_UART_Init();

  float measure_distance(void);
  extern void nano_wait(int);
  void delay_us(uint32_t us);

  /* USER CODE BEGIN 2 */
  DIST_THRESHOLD = 2;
  RIM_THRESHOLD = 50;
  BACK_THRESHOLD = 50;
  SHOT_TIME = 2;
  HAL_StatusTypeDef ret;
  config[0] = 0x20;
  config[1] = 0x60;

  accel_x_rim = 0;
  accel_y_rim = 0;
  accel_z_rim = 0;
  accel_rim_mag = 0;
  accel_x_back = 0;
  accel_y_back = 0;
  accel_z_back = 0;
  accel_back_mag = 0;
  distance = 100;
  rim_flag = 0;
  back_flag = 0;
  dist_flag = 0;
  dist_time = 0;
  rim_time = 0;
  back_time = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	GPIOC->ODR |= (1 << 9);
	GPIOB->ODR |= (1 << 12);
	//delay_us(1000000); //1000000 is 6.01 seconds
	HAL_Delay(500);
	GPIOC->ODR &= ~(1 << 9);
	GPIOB->ODR &= ~(1 << 12);

	// Make sure IMU 1 is ready before proceeding
	if (HAL_I2C_IsDeviceReady(&hi2c1, LSM9DS1_ADDR_WRITE, 100, 1000) != HAL_OK) {
		strcpy((char*) buf, "Device 1 not ready\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);

		if (HAL_I2C_IsDeviceReady(&hi2c1, LSM9DS1_ADDR_WRITE, 100, 1000) == HAL_BUSY) {
			// Write deinit and init bus code here
		}
		continue;
	}

	// Make sure IMU 2 is ready before proceeding
	if (HAL_I2C_IsDeviceReady(&hi2c2, LSM9DS1_ADDR_WRITE, 100, 1000) != HAL_OK) {
		strcpy((char*) buf, "Device 2 not ready\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);

		if (HAL_I2C_IsDeviceReady(&hi2c2, LSM9DS1_ADDR_WRITE, 100, 1000) == HAL_BUSY) {
			// Write deinit and init bus code here
		}
		continue;
	}

	// Configure IMU 1
	if (HAL_I2C_Master_Transmit(&hi2c1, LSM9DS1_ADDR_WRITE, config, 2, 10000) != HAL_OK) {
		strcpy((char*) buf, "Failed to configure 1\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);
		continue;
	}
	// Configure IMU 2
	if (HAL_I2C_Master_Transmit(&hi2c2, LSM9DS1_ADDR_WRITE, config, 2, 10000) != HAL_OK) {
		strcpy((char*) buf, "Failed to configure 2\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);
		continue;
	}

	// Read from IMU 1
	if (HAL_I2C_Mem_Read(&hi2c1, LSM9DS1_ADDR_READ, REG_ACCEL, I2C_MEMADD_SIZE_8BIT, accel_data_1, 6, HAL_MAX_DELAY) != HAL_OK) {
		strcpy((char*) buf, "Failed to read data from IMU 1\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);
		continue;
	}

	// Read from IMU 2
	if (HAL_I2C_Mem_Read(&hi2c2, LSM9DS1_ADDR_READ, REG_ACCEL, I2C_MEMADD_SIZE_8BIT, accel_data_2, 6, HAL_MAX_DELAY) != HAL_OK) {
		strcpy((char*) buf, "Failed to read data from IMU 2\r\n");
		HAL_UART_Transmit(&huart5, buf, strlen((char*) buf), 10000);
		HAL_Delay(500);
		continue;
	}

	// Parse data values into variables
	accel_x_rim = (int16_t)(accel_data_1[1] << 8 | accel_data_2[0]);
	accel_y_rim = (int16_t)(accel_data_1[3] << 8 | accel_data_2[2]);
	accel_z_rim = (int16_t)(accel_data_1[5] << 8 | accel_data_2[4]);
	accel_x_back = (int16_t)(accel_data_2[1] << 8 | accel_data_2[0]);
	accel_y_back = (int16_t)(accel_data_2[3] << 8 | accel_data_2[2]);
	accel_z_back = (int16_t)(accel_data_2[5] << 8 | accel_data_2[4]);
	distance = (distance + measure_distance()) / 2;

	// Process data
	if (distance < DIST_THRESHOLD) {
		dist_flag = 1;
		dist_time = 0;
	}

	accel_back_mag = (accel_back_mag + accel_x_back + accel_y_back + accel_z_back) / 2;
		if (accel_back_mag > BACK_THRESHOLD) {
			back_flag = 1;
			back_time = 0;
		}

	accel_rim_mag = (accel_rim_mag + accel_x_rim + accel_y_rim + accel_z_rim) / 2;
	if (accel_rim_mag - accel_back_mag > RIM_THRESHOLD || accel_back_mag - accel_rim_mag > RIM_THRESHOLD) {
		rim_flag = 1;
		rim_time = 0;
	}

	// Reset flags if time passed
	if (dist_time > SHOT_TIME) {
		dist_flag = 0;
		dist_time = 0;
	}

	if (rim_time > SHOT_TIME) {
		rim_flag = 0;
		rim_time = 0;
	}

	if (back_time > SHOT_TIME) {
		back_flag = 0;
		back_time = 0;
	}

	// Place flags into buffer
	buf[0] = dist_flag;
	buf[1] = rim_flag;
	buf[2] = back_flag;

	// Send buffer data via UART
	HAL_UART_Transmit(&huart5, buf, 3, 10000);
	HAL_Delay(100);

	// NOTE: TESTING CODE TO SEE RAW VALUES

//	snprintf(buf, 60, "%d\r\n", distance*1000);
//	HAL_UART_Transmit(&huart5, buf, BUFFER_SIZE, 10000);
//	snprintf(buf, 60, "%d %d %d\r\n", accel_x_rim, accel_y_rim, accel_z_rim);
//	HAL_UART_Transmit(&huart5, buf, BUFFER_SIZE, 10000);
//	snprintf(buf, 60, "%d %d %d\r\n", accel_x_back, accel_y_back, accel_z_back);
//	HAL_UART_Transmit(&huart5, buf, BUFFER_SIZE, 10000);

	// Increment times
	dist_time += 1;
	rim_time += 1;
	back_time += 1;
	HAL_Delay(100);
  }
  /* USER CODE END 3 */
}
//delay function in us using nano_wait assembly code (from 362 credit to Niraj Menon)
void delay_us(uint32_t us)
{
    nano_wait(us*1000); //us->ns
}
//function to calculate distance to nearest object - used to see if ball passed by
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
