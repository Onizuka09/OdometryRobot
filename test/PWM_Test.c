/*
 * main.c
 */
#include "delay.h"
#include "motor.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Register bit definitions
#define CR1_CMS ((1U << 5) | (1U << 6))
#define CR1_DIR (1U << 4)

// Helper function for GPIO configuration
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

// Variables
volatile uint64_t tick;
volatile uint8_t seconds;
volatile uint16_t duty = 0;
volatile uint8_t pwmDirection = 1; // 1 for increase, 0 for decrease

// System clock configuration
#define SYSCLK_FREQ 16000000 // 16 MHz
#define PWM_FREQ 1000        // 1 kHz PWM frequency
volatile unsigned long SystickCnt = 0;

int main(void)
{
    // Initialize system clock and peripherals
    InitSystick();

    // Enable GPIO and TIM15 clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM15EN;

    // Configure PC1 as TIM15_CH1 alternate function
    uint8_t pin = 1; // PC1
    uint8_t index = 0;
    calculate_pin(&pin, &index);

    // 1. Set alternate function mode
    GPIOC->MODER &= ~(GPIO_MODER_MODE1_Msk);
    GPIOC->MODER |= (0x2 << GPIO_MODER_MODE1_Pos);

    // 2. Set high speed
    GPIOC->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED1_Pos);

    // 3. Set pull-down (optional, helps with noise)
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1_Msk);
    GPIOC->PUPDR |= (0x2 << GPIO_PUPDR_PUPD1_Pos);

    // 4. Set alternate function AF2 (TIM15_CH1)
    GPIOC->AFR[index] &= ~(GPIO_AFRL_AFSEL1_Msk);
    GPIOC->AFR[index] |= (0x2 << GPIO_AFRL_AFSEL1_Pos);

    // Configure TIM15 for PWM
    // 1. Disable timer before configuration
    TIM15->CR1 &= ~TIM_CR1_CEN;

    // 2. Set prescaler and auto-reload for 1kHz PWM
    // PSC = (System Clock / (PWM_FREQ * ARR)) - 1
    TIM15->PSC = 16 - 1;  // No prescaler
    TIM15->ARR = 100 - 1; // 16000 - 1

    // 3. Reset counter
    TIM15->CNT = 0;

    // 4. Enable auto-reload preload
    TIM15->CR1 |= TIM_CR1_ARPE;

    // 5. Generate update event to load new settings
    TIM15->EGR |= TIM_EGR_UG;

    // Configure PWM Channel 1
    // 1. Set PWM mode 1 (OC1M = 110)
    TIM15->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM15->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);

    // 2. Enable output preload
    TIM15->CCMR1 |= TIM_CCMR1_OC1PE;

    // 3. Set initial duty cycle (start at 0%)
    TIM15->CCR1 = 0;

    // 4. Configure output polarity (active high)
    TIM15->CCER &= ~TIM_CCER_CC1P; // Active high

    // 5. Enable channel output
    TIM15->CCER |= TIM_CCER_CC1E;

    // 6. Enable main output (critical for PWM)
    TIM15->BDTR |= TIM_BDTR_MOE;

    // 7. Start the timer
    TIM15->CR1 |= TIM_CR1_CEN;

    // Main loop with PWM duty cycle sweep
    while (1)
    {
        // Fade in (0% to 100%)
    }
}
void SysTick_Handler(void)
{
    SystickCnt++;
}
void PWM_TEST()
{
    for (int d = 0; duty <= 99; d++)
    {
        TIM15->CCR1 = d;
        delay_ms(10); // Adjust speed of fade
    }

    delay_ms(500);

    // Fade out (100% to 0%)
    for (int duty = 99; d > 0; d--)
    {
        TIM15->CCR1 = d;
        delay_ms(10); // Adjust speed of fade
    }

    delay_ms(500);
}