#ifndef _USART_DEBUG8DRIVER_H_
#define _USART_DEBUG8DRIVER_H_
#include <stdint.h>
#include "USART.h"
#include <stdio.h>

void debug_uart_init();

void debug_uart_write(char data);

void  debug_uart_read(int8_t* data );
void print_console(char* data); 

#endif
