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
void GPIO_UART_Config(GPIO_TypeDef *GPIOx, PIN_Typedef PIN_x, uint8_t AFx)
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

		// 3. Set pull-down (optional, helps with noise)
	GPIOx->PUPDR &= ~(1U << (pin * 2));
	GPIOx->PUPDR |= (1U << ((pin * 2) + 1));
	// push pull 
	GPIOx->OTYPER &= ~(1U << pin );
	GPIOx->ODR =0;

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
	GPIOC->OSPEEDR &= ~(1U << (pin * 2));
	GPIOC->OSPEEDR |= (1U << ((pin * 2) + 1));
	// 3. Set pull-down (optional, helps with noise)
	GPIOC->PUPDR &= ~(1U << (pin * 2));
	GPIOC->PUPDR |= (1U << ((pin * 2) + 1));

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
	GPIOC->OSPEEDR &= ~(1U << (pin * 2));
	GPIOC->OSPEEDR |= (1U << ((pin * 2) + 1));
	// 3. Set pull-down (optional, helps with noise)
	GPIOC->PUPDR &= ~(1U << (pin * 2));
	GPIOC->PUPDR |= (1U << ((pin * 2) + 1));

	calculate_pin(&pin, &index);

	// 4.  set alternate function as AF2
	GPIOC->AFR[index] |= (AF2 << (pin * 4));
}

void GPIO_LeftEncoder_config()
{
	// - EconderA OUTA : PA8 (D7)
	// - EncoderA OUTB : PA9 (D8)
	// configure clock for GPIOA
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;	// Enable GPIOA clock
	


	// RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	uint8_t pinA = PIN_8;
	uint8_t pinB = PIN_9;
	uint8_t indexA = 0;
	uint8_t indexB = 0;
	// // configure GPIOmode to alternate function
	// // pin 8
	GPIOA->MODER &= ~ (GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk) ; 
	GPIOA->MODER |= ((0x2 << (GPIO_MODER_MODE8_Pos))| (0x2 << (GPIO_MODER_MODE9_Pos))) ;
	// // pin 9
	// GPIOA->MODER &= ~(1U << ((pinB * 2)));
	
	// // alternate function config set to timer
	calculate_pin(&pinA, &indexA);
	// // pin8 set to tim1 channel 1
	GPIOA->AFR[indexA] |= (AF2 << 4*pinA);
	// // pin9 set to tim1 channel 2
	calculate_pin(&pinB, &indexB);
	GPIOA->AFR[indexB] |= (AF2 << 4*pinB);
}

void GPIO_RightEncoder_config()
{

    
    // Configure PC6 (TIM3_CH1) and PC7 (TIM3_CH2) as alternate function
    GPIOC->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);
    GPIOC->MODER |= (0x2 << GPIO_MODER_MODE6_Pos) |  // AF mode for PC6
                    (0x2 << GPIO_MODER_MODE7_Pos);   // AF mode for PC7
    
    // Select AF1 (TIM3) for PC6 and PC7
    GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL6_Msk | GPIO_AFRL_AFSEL7_Msk);
    GPIOC->AFR[0] |= (0x1 << GPIO_AFRL_AFSEL6_Pos) |  // AF1 for PC6
                     (0x1 << GPIO_AFRL_AFSEL7_Pos);   // AF1 for PC7
    
    // Enable pull-ups (optional but recommended for encoders)
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk);
    GPIOC->PUPDR |= (0x1 << GPIO_PUPDR_PUPD6_Pos) |  // Pull-up for PC6
                    (0x1 << GPIO_PUPDR_PUPD7_Pos);   // Pull-up for PC7
}