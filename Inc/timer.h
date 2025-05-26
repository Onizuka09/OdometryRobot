#ifndef TIMER_H_
#define TIMER_H_
#include <stdint.h>
#include "stm32g070xx.h"

void Timer15_init(int PSC, int ARR);
void Timer15_Enable(); 
void Timer15_PWM_channel1_config();
void Timer15_PWM_channel2_config();

// set duty cycles 
void Timer15_set_dutyCycle_ch1(uint8_t speed);
void Timer15_set_dutyCycle_ch2(uint8_t speed);

void timer1_LeftEncoder_confifg();

void timer3_RightEncoder_confifg();
#endif
