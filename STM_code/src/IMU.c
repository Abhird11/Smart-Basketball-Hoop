/*
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"

#define IMU_REG_OUT 0 //FIX: what is it?
#define IMU_ADDRESS 0
#define IMU_REG_CTRL1 0
#define IMU_CTRL1_VALUE 0

void setup_gpio(void);
void delay_us(uint32_t us);
void I2C1_Init(void);
void IMU_Init(void);
uint8_t IMU_Read_Register(uint8_t reg);
void I2C1_Write(uint8_t address, uint8_t reg, uint8_t data);
extern void nano_wait(int);

// I2C handle
I2C_HandleTypeDef hi2c1; //FIX: why?



int main(void)
{
    // Initialize the GPIO pins
    setup_gpio();

    // Initialize the I2C1 peripheral
    I2C1_Init();

     // Initialize the IMU
    IMU_Init();

    // Main application code here

    while (1)
    {
        // Example: Read a register from the IMU
        uint8_t data = IMU_Read_Register(IMU_REG_OUT);

        // Process the data
    }
}

void GPIO_Init(void)
{
    // Enable the clock for GPIOB
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Configure PB8 and PB9 for I2C1 SCL and SDA
    GPIOB->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);  // Clear mode bits
    GPIOB->MODER |= (GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1);  // Set alternate function mode

    // Set alternate function to AF1 (I2C1)
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFRH0 | GPIO_AFRH_AFRH1);  // Clear alternate function bits
    GPIOB->AFR[1] |= (1 << ((8 - 8) * 4)) | (1 << ((9 - 8) * 4));  // Set alternate function AF1 for PB8 and PB9

    // Set output type to open-drain (1)
    GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;

    // Set pull-up resistors
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);  // Clear pull-up/pull-down bits
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0);  // Enable pull-up

    // Set output speed to high (11)
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);
}

void I2C1_Init(void)
{
    // Enable the clock for I2C1
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000; // 100kHz
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        // Initialization Error
        Error_Handler();
    }
}

void I2C1_Write(uint8_t address, uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, (address << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

uint8_t I2C1_Read(uint8_t address, uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c1, (address << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    return data;
}

void Error_Handler(void)
{
    // User can add their own implementation to report the error return state
    while (1)
    {
    }
}

void IMU_Init(void)
{
    // Example: Initialize the IMU by writing to its registers
    I2C1_Write(IMU_ADDRESS, IMU_REG_CTRL1, IMU_CTRL1_VALUE);
}

uint8_t IMU_Read_Register(uint8_t reg)
{
    return I2C1_Read(IMU_ADDRESS, reg);
}
*/
