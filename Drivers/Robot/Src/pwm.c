#include <stm32g070xx.h>
#include "pwm.h"
#include <main.h>
#include <tim.h>
void PWM_TIM15_CH1_SetDutyCyle(uint8_t t){ 

    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_1,t) ; 
}
void PWM_TIM15_CH2_SetDutyCyle(uint8_t t){ 
    __HAL_TIM_SET_COMPARE(&htim15,TIM_CHANNEL_2,t) ; 
} 

void PWM_TIM15_Start()
{
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1 );
    HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2 );
}
