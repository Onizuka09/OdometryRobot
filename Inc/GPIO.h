

#ifndef  GPIO_H_
#define GPIO_H_
// #include "stm32f401xe.h"
#include "stm32g070xx.h"
#include <stdbool.h>
#include <stdint.h>
typedef  enum {
    PIN_0 =0,
	PIN_1=1,
	PIN_2=2,
	PIN_3=3,
	PIN_4=4,
	PIN_5=5,
	PIN_6=6,
	PIN_7=7,
	PIN_8=8,
	PIN_9=9,
	PIN_10=10,
	PIN_11=11,
	PIN_12=12,
	PIN_13=13,
	PIN_14=14,
	PIN_15=15,
}PIN_Typedef;
typedef enum{
AF0 = 0U,
AF1,
AF2,
AF3,
AF4,
AF5,
AF6,
AF7,
AF8,
AF9,
AF10,
AF11,
AF12,
AF13,
AF14,
AF15
}AF_types;


typedef enum{
	OUTPUT=16,
	INPUT=17,
	TIM1_CH1_2=AF1,
	USART1_2= AF1, 
}GPIO_function;
// initializes all peripherals: enables RCC clock for peripherals  
// GPIOA ,GPIOB ...,   
void GPIO_init(); 
void GPIO_Config_UART3(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x); 
void GPIO_Config_Output(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x); 
void GPIO_Config_Input(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x); 
void GPIO_PIN_WRITE(GPIO_TypeDef* GPIOx, PIN_Typedef pinx,uint8_t state); 
bool GPIO_PIN_READ(GPIO_TypeDef* GPIOx, PIN_Typedef pinx);
void GPIO_UART_Config(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x,uint8_t AFx);

void GPIO_PWM_RightMotor_config(); 
void GPIO_PWM_LeftMotor_config(); 
void GPIO_LeftEncoder_config(); 
void GPIO_RightEncoder_config();
#endif 
