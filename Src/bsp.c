#include "bsp.h"
#include <stdint.h>
#include "GPIO.h"
// PA5
#define LED_GPIOA_en (1U << 0) // enable the clock source (set 1 at pos 0)
#define LED_PIN 5
#define LED_PIN_MSK (1U << LED_PIN)

#define BTN_PIN 13
#define BTN_PIN_MSK (1U << 13)
#define BTN_GPIOC GPIOC

volatile uint8_t btn_status = 0;
void btn_EXTI_Config();
void Led_init()
{
    RCC->IOPENR |= LED_GPIOA_en;

    GPIOA->MODER |= (1U << 10);
    GPIOA->MODER &= ~(1U << 11);
}
void Toggle_led()
{

    GPIOA->ODR ^= LED_PIN_MSK;
}
void TurOn_led()
{
    GPIOA->ODR |= LED_PIN_MSK;
}
void TurOff_led()
{
    GPIOA->ODR &= ~LED_PIN_MSK;
}
void btn_init()
{
    GPIO_Config_Input(BTN_GPIOC, BTN_PIN); // Configure internal button as input
    // btn_EXTI_Config();
}
uint8_t read_btn_status()
{
    if ((GPIOC->IDR & (1U << BTN_PIN)))
    {
        return 0;
    }
    else
    {

        TurOn_led();
        return 1;
    }
    return 0;
}

void btn_EXTI_Config()
{
    __disable_irq();
    // Enable clock access to SYSCFG
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;
    // RCC->APBENR2 |= RCC_AHBENR_EX;

    // Connect EXTI line with PC13
    // x = 13 / 4 = 3.25 => get rid of the deciaml number => 3
    // use the formula m = 4*2 = 8 => m+3 = 11
    EXTI->EXTICR[3] |= (0x2 << 8); // PC13 (0x02)

    // Unmask EXTI13
    EXTI->IMR1 |= EXTI_IMR1_IM13;
    // Select rising edge trigger
    EXTI->RTSR1 |= EXTI_RTSR1_RT13;
    EXTI->FTSR1 &= ~EXTI_FTSR1_FT13;
    // Enable EXTI13 line in NVIC
    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
    __enable_irq();
}

// EXTI4_15_IRQn

void EXTI4_15_IRQHandler(void)
{
    __disable_irq();
    if (EXTI->RPR1 & (1U << 13))
    {
        EXTI->RPR1 |= (1U << 13); // Clear the pending bit
        TurOn_led();
        btn_status = 1;
    }
    __enable_irq();
    return;
}