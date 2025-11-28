#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "delay.h"
#include "motor.h"
#include "pwm.h"
#include "led.h"
#include "Debug_dirver.h"
#include "odometry.h"
#include "system_clk.h"
#include "odometry.h"

volatile bool btn = false;
void PC13_EXTI_Config();
int32_t time = 0 ; 
volatile uint32_t odo_time=0, curr_odo_time=0, prev_odo_time = 0 ; 
OdometryTypedef odo = {0}; 

int main(void)
{
    // Initialize GPIOA, GPIOB, and GPIOC

    systemClock_Config();

    GPIO_init();
    Led_init();
    InitSystick();
    // debug_uart_init();

    GPIO_Config_Output(GPIOA, PIN_5);
    GPIO_PIN_WRITE(GPIOA, PIN_5, 0);
    GPIO_Config_Input(GPIOC, PIN_13); // Configure internal button as input
    PC13_EXTI_Config();
    init_motors();
    // Configure encoder interrupts
    GPIO_LeftEncoder_config();
    GPIO_RightEncoder_config();
    timer1_LeftEncoder_confifg();
    timer3_RightEncoder_confifg();
    OdoInit(&odo); 

    while (1)
    {
        curr_odo_time =  get_current_time_ms();     
        // odo_time = curr_odo_time -prev_odo_time ;
        if (get_current_time_ms() - odo_time >=PERIOD) {

            OdoUpdate(&odo,10); 
            odo_time = get_current_time_ms(); 
        }
        prev_odo_time =   0 ; 

        if (get_current_time_ms() - time >= 1000 ) {
            Toggle_led();
            
            time = get_current_time_ms();  
        }
        // delay_ms(1000);        
        //cntA = TIM1->CNT >> 2;
        //cntB = TIM3->CNT >> 2;


        // avrage_dist = get_curent_distance(cntA,cntB);


        // print_console("hello world \n");
    }

    return 0;
}

void PC13_EXTI_Config()
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
    if (EXTI->RPR1 & (1U << 13))
    {
        EXTI->RPR1 |= (1U << 13); // Clear the pending bit
        GPIOA->ODR ^= (1U << PIN_5);
        btn = true;
    }
    return;
}

