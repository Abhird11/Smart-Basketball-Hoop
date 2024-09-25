/*
#include "stm32f0xx.h"
//#include "stm32f0xx_hal.h"

#define TRIGGER_PIN 6                   // Assume trigger is connected to PB6
#define ECHO_PIN 7                      // Assume echo is connected to PB7

void setup_gpio(void);
void delay_us(uint32_t us);
void togglexn(GPIO_TypeDef *port, int n);
void turnon(GPIO_TypeDef *port, int n);
void turnoff(GPIO_TypeDef *port, int n);
void initc();
uint32_t measure_distance(void);
extern void nano_wait(int);

int main(void)
{
    setup_gpio();
    // Initialize the HAL Library
    //HAL_Init();

    // Slowly blinking
    
    //for(;;) {
    //    togglexn(GPIOC, 9);
    //    delay_us(500000);
    //}

    int max = 0;
    int counter = 0;
    
    while (1) {
        //get the distance 
        uint32_t distance = measure_distance();

        counter++;
        if(distance > max){
            //track max distance
            max = distance;
        }
        
        //delay 10000us = 0.01 second between measurements
        delay_us(10000);  

        if(distance == 0){
            //sees ball/object close to it
            turnon(GPIOC, 9);
        }
        else{
            //does not see ball/object close to it
            turnoff(GPIOC, 9);
        }  
    }
}
//setup LED as output
void initc() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  //enable C clock

    GPIOC->MODER &= 0xFFF3FFFF;         //set pin PC9 to 00 first
    GPIOC->MODER |= 0x00040000;         //set pin 9 as output
}
//toggle output pin
void togglexn(GPIO_TypeDef *port, int n) {
    port->ODR ^= (1 << n);

}
//turn on output pin
void turnon(GPIO_TypeDef *port, int n) {
    port->ODR |= (1 << n);
}
//turn off output pin
void turnoff(GPIO_TypeDef *port, int n) {
    port->ODR &= ~(1 << n);
}
//config GPIO clock and pins
void setup_gpio(void)
{
    initc();
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
    
    // Configure PB6 as output (trigger)
    GPIOB->MODER &= 0xFFFFCFFF; //clear
    GPIOB->MODER |= 0x00001000; //set
    
    // Configure PB7 as input (echo)
    GPIOB->MODER &= 0xFFFF3FFF;
}
//delay function in us using nano_wait assembly code (from 362 credit to Niraj Menon)
void delay_us(uint32_t us) 
{
    nano_wait(us*1000); //us->ns
}
//dunction to calculate distance to nearest object - used to see if ball passed by
uint32_t measure_distance(void) 
{
    //initilaize time to 0
    uint32_t time = 0;
    
    // Send 10us pulse to trigger
    GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
    delay_us(10);
    GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low

    //wait for echo pin to go high
    while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 0){ 
        GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
        delay_us(10);
        GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low
    }
    
    //now measure time it is high and that indicates how long wave travled, menaing how far it traveled before hitting
    
    // Measure how long echo pin stays high
    while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 1) {
        time++;
        delay_us(1);
    }
    
    float scalar = 1;
    // Convert time to distance in cm: (us/1e6)= 1s * 343 = m, * 100 = cm: (time / 1E6) * 343 gives m / s * 100 gives cm/s
    return ((time * 0.0343) / 2) * scalar; //use scalar if needed 
}
*/
/* newer
#include "stm32f0xx.h"
//#include "stm32f0xx_hal.h"

#define TRIGGER_PIN 6                   // Assume trigger is connected to PB6
#define ECHO_PIN 7                      // Assume echo is connected to PB7

void setup_gpio(void);
void delay_us(uint32_t us);
void togglexn(GPIO_TypeDef *port, int n);
void turnon(GPIO_TypeDef *port, int n);
void turnoff(GPIO_TypeDef *port, int n);
void initc();
uint32_t measure_distance(void);
extern void nano_wait(int);

int main(void)
{
    setup_gpio();
    // Initialize the HAL Library
    //HAL_Init();

    // Slowly blinking
    
    //for(;;) {
    //    togglexn(GPIOC, 9);
    //    delay_us(500000);
    //}

    int max = 0;
    int counter = 0;

    //turnon(GPIOB, 15);
    
    while (1) {
        //get the distance 
        uint32_t distance = measure_distance();

        counter++;
        if(distance > max){
            //track max distance
            max = distance;
        }
        
        //delay 10000us = 0.01 second between measurements
        delay_us(10000);  

        if(distance == 0){
            //sees ball/object close to it
            turnon(GPIOC, 9);
            //we scored
        }
        else{
            //does not see ball/object close to it
            turnoff(GPIOC, 9);
            
        }  
    }
}
//setup LED as output
void initc() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  //enable C clock

    GPIOC->MODER &= 0xFFF3FFFF;         //set pin PC9 to 00 first
    GPIOC->MODER |= 0x00040000;         //set pin 9 as output
}
//toggle output pin
void togglexn(GPIO_TypeDef *port, int n) {
    port->ODR ^= (1 << n);

}
//turn on output pin
void turnon(GPIO_TypeDef *port, int n) {
    port->ODR |= (1 << n);
}
//turn off output pin
void turnoff(GPIO_TypeDef *port, int n) {
    port->ODR &= ~(1 << n);
}
//config GPIO clock and pins
void setup_gpio(void)
{
    initc();
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
    
    // Configure PB6 and PB15 as output (trigger)
    GPIOB->MODER &= 0x3FFFCFFF; //clear
    GPIOB->MODER |= 0x40001000; //set
    
    // Configure PB7 as input (echo)
    GPIOB->MODER &= 0xFFFF3FFF;
}
//delay function in us using nano_wait assembly code (from 362 credit to Niraj Menon)
void delay_us(uint32_t us) 
{
    nano_wait(us*1000); //us->ns
}
//dunction to calculate distance to nearest object - used to see if ball passed by
uint32_t measure_distance(void) 
{
    //initilaize time to 0
    uint32_t time = 0;
    
    // Send 10us pulse to trigger
    GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
    delay_us(10);
    GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low

    //wait for echo pin to go high
    while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 0){ 
        GPIOB->BSRR = 1 << TRIGGER_PIN;         //set PB6 high
        delay_us(10);
        GPIOB->BRR = 1 << TRIGGER_PIN;         // Set PB6 low
    }
    
    //now measure time it is high and that indicates how long wave travled, menaing how far it traveled before hitting
    
    // Measure how long echo pin stays high
    while ((((GPIOB->IDR) >> ECHO_PIN) & 0x1) == 1) {
        time++;
        delay_us(1);
    }
    
    float scalar = 1;
    // Convert time to distance in cm: (us/1e6)= 1s * 343 = m, * 100 = cm: (time / 1E6) * 343 gives m / s * 100 gives cm/s
    return ((time * 0.0343) / 2) * scalar; //use scalar if needed 
}
*/