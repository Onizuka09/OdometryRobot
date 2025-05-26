#include "timer.h"
#include <stm32g070xx.h>
uint8_t max_duty_cycle = 100;
#define CR1_CEN (1U << 0) // enable timer15
#define CR1_APRE (1U << 7)
#define CR1_CMS ((1U << 5) | (1U << 6))
#define CR1_DIR (1U << 4)
#define EGR_UG (1U << 0)

#define CCER_CC1E (1U << 0)
#define CCMR_OC1Mpwm (6U << 4)
#define CCMR_OC1PE (1U << 3)

#define CCER_CC2E (1U << 4)
#define CCMR_OC2Mpwm (6U << 12)
#define CCMR_OC2PE (1U << 11)

void Timer15_init(int PSC , int ARR)
{

    /* timer15 configuration  */
    // enable clock access to timer2
    RCC->APBENR2 |= RCC_APBENR2_TIM15EN;
    // disable Timer before configuration
    TIM15->CR1 &= ~TIM_CR1_CEN;
    // set prescaler
    TIM15->PSC = PSC; // 16 MHz / 1600 = 10,000 Hz
    // set auto reload value
    TIM15->ARR = ARR; // 10,000 Hz / 10,000  = 1 Hz (1 s)
    max_duty_cycle = ARR;
    // reset counter value
    TIM15->CNT = 0;
  
    // enable APRE ( Reload register) for safety ( recommending because it introduces a smooth transitionning when changing the PWM periods ARR)
    TIM15->CR1 |= TIM_CR1_ARPE;
    // EGR enable (UG) update generation is enabled (ensures synchronizing the udpate events ( reinializes the timer and update associated register ARR and other control registers ))
    TIM15->EGR |= TIM_EGR_UG;
}
void Timer15_Enable()
{
    // 6. Enable main output (critical for PWM)
    TIM15->BDTR |= TIM_BDTR_MOE;

    // 7. Start the timer
    TIM15->CR1 |= TIM_CR1_CEN;
}
void Timer15_PWM_channel1_config()
{

    // 1. Set PWM mode 1 (OC1M = 110)
    TIM15->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM15->CCMR1 |= (0x6 << TIM_CCMR1_OC1M_Pos);

    // 2. Enable output preload
    TIM15->CCMR1 |= TIM_CCMR1_OC1PE;

    // 3. Set initial duty cycle (start at 0%)
    TIM15->CCR1 = 0;

    // 4. Configure output polarity (active LOW)
    TIM15->CCER |= TIM_CCER_CC1P;

    // 5. Enable channel output
    TIM15->CCER |= TIM_CCER_CC1E;
}

void Timer15_PWM_channel2_config()
{

    // 1. Set PWM mode 1 (OC1M = 110)
    // TIM15->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
    TIM15->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos);

    // 2. Enable output preload
    TIM15->CCMR1 |= TIM_CCMR1_OC2PE;

    // 3. Set initial duty cycle (start at 0%)
    TIM15->CCR2 = 0;

    // 4. Configure output polarity (active LOW)
    TIM15->CCER |= TIM_CCER_CC2P;

    // 5. Enable channel output
    TIM15->CCER |= TIM_CCER_CC2E;
}
void Timer15_set_dutyCycle_ch1(uint8_t speed)
{

    TIM15->CCR1 = speed;
}
void Timer15_set_dutyCycle_ch2(uint8_t speed)
{

    TIM15->CCR2 = speed;
}
void timer1_LeftEncoder_confifg()
{
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // Enable TIM1 clock (APB2 for STM32G0)

    TIM1->PSC = 0;                              // No prescaler (1:1)
    TIM1->ARR = 0xFFFF;                         // Auto-reload = max 16-bit value
    TIM1->CCMR1 = (0x1 << TIM_CCMR1_CC1S_Pos) | // CC1S = 01 (TI1)
                  (0x1 << TIM_CCMR1_CC2S_Pos);  // CC2S = 01 (TI2)
    // Input filter (4 samples)
    TIM1->CCMR1 = (0x2 << TIM_CCMR1_IC1F_Pos) | (0x2 << TIM_CCMR1_IC2F_Pos);
    TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Rising edge polarity
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);  // Enable CH1 & CH2 inputs
    TIM1->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;  // Encoder mode 3 (both edges)
    TIM1->CR1 |= TIM_CR1_CEN;                       // Start timer
}
void timer3_RightEncoder_confifg()
{
    // Enable TIM3 clock
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;

    // Reset TIM3 to defaults
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;

    // No prescaler, full 16-bit counter
    TIM3->PSC = 0;
    TIM3->ARR = 0xFFFF;

    // Configure input capture mapping
    // TI1 -> CH1, TI2 -> CH2
    TIM3->CCMR1 = (0x1 << TIM_CCMR1_CC1S_Pos) | // CC1S = 01 (TI1)
                  (0x1 << TIM_CCMR1_CC2S_Pos);  // CC2S = 01 (TI2)

    // Configure filters (N=4 samples at fCK_INT)
    TIM3->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos) | // IC1F = 0010 (N=4)
                   (0x2 << TIM_CCMR1_IC2F_Pos);  // IC2F = 0010 (N=4)

    // Configure polarity and enable inputs
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Rising edge
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;    // Enable inputs

    // Configure encoder mode (both edges)
    TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1; // Encoder mode 3

    // Enable counter
    TIM3->CR1 |= TIM_CR1_CEN;
}