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

void Timer15_init(){

    /* timer15 configuration  */
    // enable clock access to timer2
    RCC->APBENR2 |= RCC_APBENR2_TIM15EN;
    // disable Timer before configuration
    TIM15->CR1 &= ~TIM_CR1_CEN;
    // set prescaler
    TIM15->PSC = 16 - 1; // 16 MHz / 1600 = 10,000 Hz
    // set auto reload value
    TIM15->ARR = 100 - 1; // 10,000 Hz / 10,000  = 1 Hz (1 s)
    max_duty_cycle = 100;
    // reset counter value
    TIM15->CNT = 0;
    // edge eligned mode Timer 15 is already configured to be edge eligned
    // TIM15->CR1 &= ~CR1_CMS;
    // direction DIR (upcount)
    // TIM15->CR1 &= ~CR1_DIR;
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
    TIM15->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
    TIM15->CCMR1 |= (0x6 << TIM_CCMR1_OC2M_Pos);

    // 2. Enable output preload
    TIM15->CCMR1 |= TIM_CCMR1_OC2PE;

    // 3. Set initial duty cycle (start at 0%)
    TIM15->CCR2 = 0;

    // 4. Configure output polarity (active LOW)
    TIM15->CCER |= TIM_CCER_CC1P; 

    // 5. Enable channel output
    TIM15->CCER |= TIM_CCER_CC2E;
}
void Timer15_set_dutyCycle_ch1(uint8_t speed)
{
    if (speed > max_duty_cycle)
        speed = max_duty_cycle;
    if (speed < 0)
        speed = 0;
    TIM15->CCR1 = speed;
}
void Timer15_set_dutyCycle_ch2( uint8_t speed)
{
    if (speed > max_duty_cycle)
        speed = max_duty_cycle;
    if (speed < 0)
        speed = 0;
    TIM15->CCR2 = speed;
}
void timer1_LeftEncoder_confifg()
{
    RCC->APBENR1 |= RCC_APBENR2_TIM1EN;
    // selecting encoder mode
    TIM1->SMCR |= (0x3 << 0);
    // configure polarity
    // i am goning to leave it to 0 normal no inversion
    // 	(Optional) Set input filtering (IC1F, IC2F).
    // Set Auto-Reload (ARR) to desired max value.
    TIM1->ARR = 65535 - 1;
    // Enable the counter (CEN=1 in TIMx_CR1).
    TIM1->CR1 |= (1U << 0);
}
void timer3_RightEncoder_confifg()
{
    // enable timer 3
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    // selecting encoder mode
    TIM3->SMCR |= (0x3 << 0);
    // configure polarity
    // i am goning to leave it to 0 normal no inversion
    // 	(Optional) Set input filtering (IC1F, IC2F).
    // Set Auto-Reload (ARR) to desired max value.
    TIM3->ARR = 65535 - 1;
    // Enable the counter (CEN=1 in TIMx_CR1).
    TIM3->CR1 |= (1U << 0);
}