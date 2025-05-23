#ifndef UART_H_ 
#define UART_H_
#include <stdbool.h>
#include <stdint.h>
#include "stm32g070xx.h"
#include "GPIO.h"
typedef struct{
GPIO_TypeDef* GPIOx_Tx; 
PIN_Typedef PINx_Tx; 
GPIO_TypeDef* GPIOx_Rx;  
PIN_Typedef PINx_Rx;
//AF_types AFx; 
GPIO_function AFx;  
USART_TypeDef* UARTx;
uint32_t baudrate;
bool interrupt_en;

}UARTx_struct;  

#define UART_SR_TXE ( 1U<<7 )
#define UART_SR_RXNE ( 1U<<5)



void init_UARTx(UARTx_struct* UARTx_str); 

void UARTx_write(UARTx_struct* UARTx_str,int8_t data); 
int8_t UARTx_read(UARTx_struct* UARTx_str); 
#endif
