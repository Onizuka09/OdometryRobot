/*
 * delay.c
 *
 *  Created on: Mar 19, 2024
 *      Author: moktar
 */

#include <stm32g070xx.h>
#include <main.h>
#include "delay.h"

void delay_ms(uint32_t d)
{
     HAL_Delay(d);
}

uint32_t get_current_time_ms(void)
{
     return HAL_GetTick();
}