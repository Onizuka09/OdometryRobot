#include <stm32g070xx.h>
#include "pwm.h"

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
void PWM_TIM15_CH1_SetDutyCyle(uint8_t t){ 
    TIM15->CCR1 = t ; 
}
void PWM_TIM15_CH2_SetDutyCyle(uint8_t t){ 
    TIM15->CCR2 = t ; 
} 

void PWM_TIM15_CH1_Start()
{
    TIM15->BDTR |= TIM_BDTR_MOE;

    // 7. Start the timer
    TIM15->CR1 |= TIM_CR1_CEN;
}
void PWM_TIM15_CH1_Config()
{
    // 2. Enable GPIO and TIM15 clocks
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;   // Enable GPIOC
    RCC->APBENR2 |= RCC_APBENR2_TIM15EN; // Enable TIM15

    // 3. Configure PC1 (TIM15_CH1) as alternate function
    GPIOC->MODER &= ~GPIO_MODER_MODE1_Msk;         // Clear mode bits
    GPIOC->MODER |= (0x2 << GPIO_MODER_MODE1_Pos); // Alternate function mode

    GPIOC->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED1_Pos); // High speed

    // Set AF2 (TIM15_CH1) for PC1
    GPIOC->AFR[0] &= ~GPIO_AFRL_AFSEL1_Msk;
    GPIOC->AFR[0] |= (0x2 << GPIO_AFRL_AFSEL1_Pos);

    // 4. Configure TIM15 for PWM
    TIM15->CR1 &= ~TIM_CR1_CEN; // Disable timer

    // 16kHz PWM configuration:
    TIM15->PSC = 15; // No prescaler
    TIM15->ARR = 99; // 10 khz

    TIM15->CNT = 0; // Reset counter

    // 5. Configure Channel 1 PWM
    TIM15->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;         // Clear OC1M bits
    TIM15->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos); // PWM mode 1 (110)

    TIM15->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload

    TIM15->CCER &= ~TIM_CCER_CC1P; // Active high polarity
    TIM15->CCER |= TIM_CCER_CC1E;  // Enable output

    TIM15->BDTR |= TIM_BDTR_MOE; // Master output enable

    // 6. Start timer
    // TIM15->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE; // Enable timer + auto-reload

    // 7. Configure motor control pins (H-bridge example)
}
void PWM_TIM15_CH2_Config()
{
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
    RCC->APBENR2 |= RCC_APBENR2_TIM15EN;

    // 2. Configure PC2 (TIM15_CH2) - CHANGED FROM PC1 TO PC2!
    uint8_t pin = 2; // PC2
    uint8_t index = 0;
    calculate_pin(&pin, &index);

    GPIOC->MODER &= ~(GPIO_MODER_MODE2_Msk);
    GPIOC->MODER |= (0x2 << GPIO_MODER_MODE2_Pos); // Alternate function

    GPIOC->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED2_Pos); // High speed

    GPIOC->AFR[index] &= ~(GPIO_AFRL_AFSEL2_Msk);
    GPIOC->AFR[index] |= (0x2 << GPIO_AFRL_AFSEL2_Pos); // AF2 = TIM15_CH2

    // 3. Configure TIM15
    TIM15->CR1 &= ~TIM_CR1_CEN; // Disable timer

    // Better PWM frequency (16kHz)
    TIM15->PSC = 15; // No prescaler
    TIM15->ARR = 99; // 16MHz/1000 = 16kHz

    TIM15->CNT = 0; // Reset counter

    // 4. Configure Channel 2 PWM (CRITICAL FIXES)
    TIM15->CCMR1 &= ~TIM_CCMR1_OC2M;             // Clear OC2M bits
    TIM15->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos); // PWM mode 1 (110)

    TIM15->CCMR1 |= TIM_CCMR1_OC2PE; // Enable preload

    TIM15->CCER &= ~TIM_CCER_CC2P; // Active high
    TIM15->CCER |= TIM_CCER_CC2E;  // Enable output
}