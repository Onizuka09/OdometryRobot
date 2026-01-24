#include "encoder.h"
#include <stm32g070xx.h>
#include <main.h> 
#include <tim.h> 
void timer1_LeftEncoder_start(){
    HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL); 
}

void timer3_RightEncoder_start(){ 

    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); 
}
// uint16_t timer1_LeftEncoder_Read(){
//    return (uint16_t) __HAL_TIM_GET_COUNTER(&htim1) ;
// }

// uint16_t timer3_RightEncoder_Read() { 

//     return (uint16_t) __HAL_TIM_GET_COUNTER(&htim3) ;
// }
uint16_t timer1_LeftEncoder_Read(){
    return (int16_t) TIM1->CNT  ; 
}

uint16_t timer3_RightEncoder_Read() { 
    return (int16_t)TIM3->CNT ; 
}