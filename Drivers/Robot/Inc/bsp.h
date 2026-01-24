#ifndef BSP_H__
#define BSP_H__

#include <stm32g0xx.h> 
#include<stm32g070xx.h>

void Toggle_led();
void TurOn_led();
void TurOff_led();


uint8_t read_btn_status(); 

#endif