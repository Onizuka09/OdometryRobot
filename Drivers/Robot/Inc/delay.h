#ifndef _DELAY_H_
#define _DELAY_H_
/*
 * delay.h
 *
 *  Created on: Mar 19, 2024
 *      Author: moktar
 */


#include <stdint.h>
#include <main.h>

void delay_ms(uint32_t d);

uint32_t get_current_time_ms(void) ;

#endif /* DELAY_H_ */
