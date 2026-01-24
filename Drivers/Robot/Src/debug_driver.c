/*
 * USART_Debug_dirver.c
 *
 *  Created on: Mar 17, 2024
 *      Author: moktar
 */

#include "debug_driver.h"
#include <main.h>
#include <stdint.h>
#include <string.h>
#include <usart.h>
#include <stdio.h> 
int __io_putchar(int ch)
{
	debug_uart_write((ch & 0xFF));
	return ch;
}




void debug_uart_write(char data)
{
	HAL_UART_Transmit(&huart2,(uint8_t*) &data,sizeof(data), HAL_MAX_DELAY);
}
void debug_uart_read(int8_t *data)
{
	HAL_UART_Receive(&huart2,(uint8_t*)data,sizeof(*data),HAL_MAX_DELAY);
}

void print_console(char *data)
{
	int size = strlen(data);
	for (int i = 0; i < size; i++)
	{
		debug_uart_write(data[i]);
	}
	debug_uart_write('\0');
	debug_uart_write('\r');
}