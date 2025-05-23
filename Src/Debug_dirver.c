/*
 * USART_Debug_dirver.c
 *
 *  Created on: Mar 17, 2024
 *      Author: moktar
 */

#include "Debug_dirver.h"
#include "GPIO.h"
#include "USART.h"
#include <stdint.h>
#include <string.h>

int __io_putchar(int ch)
{
	debug_uart_write((ch & 0xFF));
	return ch;
}

UARTx_struct UART2_struct = {
	.GPIOx_Rx = GPIOA,
	.PINx_Rx = PIN_3,
	.GPIOx_Tx = GPIOA,
	.PINx_Tx = PIN_2,
	.AFx = USART1_2,
	.UARTx = USART2,
	.baudrate = 112500,
	.interrupt_en = false,
};
void debug_uart_init()
{
	// enable GPIOA
	// RCC->AHB1ENR |= GPIOA_RCC_enr ;
	init_UARTx(&UART2_struct);
}

void debug_uart_write(char data)
{
	UARTx_write(&UART2_struct, data);
}
void debug_uart_read(int8_t *data)
{
	*data = UARTx_read(&UART2_struct);
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