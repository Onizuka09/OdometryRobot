#ifndef ENCODER_H_
#define ENCODER_H_
#include <stdint.h>
#include "stm32g070xx.h"

void timer1_LeftEncoder_start();

void timer3_RightEncoder_start();
uint16_t timer1_LeftEncoder_Read(); 

uint16_t timer3_RightEncoder_Read(); 
#endif
