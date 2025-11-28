/*
 * delay.c
 *
 *  Created on: Mar 19, 2024
 *      Author: moktar
 */

#include <stm32g070xx.h>
#define SYSTICK_LOAD_VAl 16000
#define CTRL_CLK (1U << 2)
#define CTRL_EN (1U << 0)
#define CTRL_COUNTFLAG (1U << 16)
static volatile uint32_t SystickCnt = 0  ;

void InitSystick()
{
	// reload with number of clocks per millisecond
	SysTick->LOAD = SYSTICK_LOAD_VAl;
	// reset counter falg value
	SysTick->VAL = 0;
	// the interrupt is enabled here 
	SysTick->CTRL = 0x7; // Enable Systick, exception,and use processor clock
	NVIC_EnableIRQ(SysTick_IRQn);
	NVIC_SetPriority(SysTick_IRQn,0);
}

void delay_ms(uint32_t d)
{
	uint32_t start = SystickCnt;
	while (SystickCnt - start < d)
	{
	};
}

void SysTick_Handler(){
	SystickCnt ++ ; 
}

uint32_t get_current_time_ms(void)
{
    return SystickCnt;
}