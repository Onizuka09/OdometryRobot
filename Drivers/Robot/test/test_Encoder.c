#include <stm32g070xx.h>

void SystemInit(){ 

}
void GPIO_Init(void);
void TIM3_Encoder_Init(void);
 int32_t current_count = 0 ; 
int main(void)
{
    // Initialize system

    GPIO_Init();
    TIM3_Encoder_Init();
    

    while(1)
    {
        current_count = TIM3->CNT >> 2;
        
        // Detect count changes

    }
}

void GPIO_Init(void)
{
    // Enable GPIOC clock
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
    
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

void TIM3_Encoder_Init(void)
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
    TIM3->CCMR1 = (0x1 << TIM_CCMR1_CC1S_Pos) |  // CC1S = 01 (TI1)
                  (0x1 << TIM_CCMR1_CC2S_Pos);   // CC2S = 01 (TI2)
    
    // Configure filters (N=4 samples at fCK_INT)
    TIM3->CCMR1 |= (0x2 << TIM_CCMR1_IC1F_Pos) |  // IC1F = 0010 (N=4)
                   (0x2 << TIM_CCMR1_IC2F_Pos);   // IC2F = 0010 (N=4)
    
    // Configure polarity and enable inputs
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); // Rising edge
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;    // Enable inputs
    
    // Configure encoder mode (both edges)
    TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;  // Encoder mode 3
    
    // Enable counter
    TIM3->CR1 |= TIM_CR1_CEN;
    
    // Optional: Enable overflow interrupt if needed
    //TIM3->DIER |= TIM_DIER_UIE;
    //NVIC_EnableIRQ(TIM3_IRQn);
    //NVIC_SetPriority(TIM3_IRQn, 0);
}