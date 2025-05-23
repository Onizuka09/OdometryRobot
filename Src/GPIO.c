#include "GPIO.h"
#include <stm32g070xx.h>
#include <stdint.h>
// initializes all peripherals: enables RCC clock for peripherals
// GPIOA ,GPIOB ...,
void GPIO_init()
{
	// clock enable
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	// other settings for GPIO conifg PULLUP PULL DOWN ...
}
// put the timer2 configs here
static void calculate_pin(uint8_t *pin, uint8_t *index)
{
	if (*pin < 8)
		*index = 0;
	else
	{
		*index = 1;
		*pin = *pin - 8;
	}
}
// configures the UART1 AND 2 
void GPIO_UART_Config(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x,uint8_t AFx)
{
	uint8_t pin = PIN_x;
	uint8_t index = 0;
	calculate_pin(&pin, &index);
	GPIOx->MODER &= ~(1U << (pin * 2));
	GPIOx->MODER |= (1U << ((pin * 2) + 1));

	// set alternate function as AF1
	GPIOx->AFR[index] |= (AFx << (pin * 4));
}
void GPIO_Config_Output(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x)
{
	uint8_t pin = PIN_x;

	GPIOx->MODER |= (1U << (pin * 2));
	GPIOx->MODER &= ~(1U << ((pin * 2) + 1));
}
void GPIO_Config_Input(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x)
{
	uint8_t pin = PIN_x;

	GPIOx->MODER &= ~(1U << (pin * 2));
	GPIOx->MODER &= ~(1U << ((pin * 2) + 1));

	GPIOx->PUPDR &= ~(1U << (pin * 2));
	GPIOx->PUPDR |= (1U << ((pin * 2) + 1));
}


bool GPIO_PIN_READ(GPIO_TypeDef *GPIOx, PIN_Typedef pinx)
{
	return (GPIOx->IDR & (1U << pinx));
}
void GPIO_PIN_WRITE(GPIO_TypeDef *GPIOx, PIN_Typedef pinx, uint8_t state)
{
	if (state)
	{
		GPIOx->ODR |= (1U << pinx);
	}
	else
	{
		GPIOx->ODR &= ~(1U << pinx);
	}
}
void GPIO_PWM_LeftMotor_config()
{
	// PC1
	// 0. Enable GPIOA 
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	uint8_t pin = PIN_1;
	uint8_t index = 0;
	// 1. Configure mode to alternate function 
	GPIOC->MODER &= ~(1U << (pin * 2));
	GPIOC->MODER |= (1U << ((pin * 2) + 1));
	// 2. set output to high speed 
	GPIOC->OSPEEDR&= ~(1U << (pin * 2));
	GPIOC->OSPEEDR |= (1U << ((pin * 2) + 1));
    // 3. Set pull-down (optional, helps with noise)
	GPIOC->PUPDR &= ~(1U << (pin * 2));
	GPIOC->PUPDR  |= (1U << ((pin * 2) + 1));
    
	calculate_pin(&pin, &index);

	// 4.  set alternate function as AF2 
	GPIOC->AFR[index] |= (AF2 << (pin * 4));
}
void GPIO_PWM_RightMotor_config()
{
	// PC2
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
	uint8_t pin = PIN_2;
	uint8_t index = 0;
	// 1. Configure mode to alternate function 
	GPIOC->MODER &= ~(1U << (pin * 2));
	GPIOC->MODER |= (1U << ((pin * 2) + 1));
	// 2. set output to high speed 
	GPIOC->OSPEEDR&= ~(1U << (pin * 2));
	GPIOC->OSPEEDR |= (1U << ((pin * 2) + 1));
    // 3. Set pull-down (optional, helps with noise)
	GPIOC->PUPDR &= ~(1U << (pin * 2));
	GPIOC->PUPDR  |= (1U << ((pin * 2) + 1));
    
	calculate_pin(&pin, &index);

	// 4.  set alternate function as AF2 
	GPIOC->AFR[index] |= (AF2 << (pin * 4));
}

void GPIO_LeftEncoder_config()
{
	// - EconderA OUTA : PA8 (D7)
	// - EncoderA OUTB : PA9 (D8)
	// configure clock for GPIOA

	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	// configure GPIOmode to alternate function
	// pin 8
	GPIOA->MODER &= ~(1U << ((8 * 2)));
	GPIOA->MODER |= (1U << ((8 * 2) + 1));
	// pin 9
	GPIOA->MODER &= ~(1U << ((9 * 2)));
	GPIOA->MODER |= (1U << ((9 * 2) + 1));
	// alternate function config set to timer
	// pin8 set to tim1 channel 1
	GPIOA->AFR[1] |= (AF2 << 0);
	// pin9 set to tim1 channel 2
	GPIOA->AFR[1] |= (AF2 << 4);
}

void GPIO_RightEncoder_config()
{
	// - EconderB OUTA : Pc7 (D9) TIM3_ch2
	// - EncoderB OUTB : PC6 (x) TIM3_ch1
	RCC->IOPENR |= RCC_IOPENR_GPIOCEN;

	// configure GPIOmode to alternate function
	// pin 6
	GPIOC->MODER &= ~(1U << ((6 * 2)));
	GPIOC->MODER |= (1U << ((6 * 2) + 1));
	// pin 7
	GPIOC->MODER &= ~(1U << ((7 * 2)));
	GPIOC->MODER |= (1U << ((7 * 2) + 1));
	// alternate function config set to timer
	// pin7 set to tim3 channel 1
	GPIOC->AFR[0] |= (AF2 << 28);
	// pin6 set to tim3 channel 2
	GPIOC->AFR[0] |= (AF2 << 24);
}