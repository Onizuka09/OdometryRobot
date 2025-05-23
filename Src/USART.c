/* USART
 * PA9 Tx
 * PA10 Rx
 */

#include "USART.h"
#include "GPIO.h"
#include "stm32g070xx.h"
#include <stdint.h>

#define SYS_FRQ 16000000 // 16Mhr
#define APB1_clk SYS_FRQ
#define SYS_FRQ 16000000 // 16Mhr
#define APB1_clk SYS_FRQ
#define USARTx_en (1U << 13)
#define CR1_TE (1U << 3)
#define CR1_RE (1U << 2)

static uint16_t calculate_baudrate(uint32_t bd, uint32_t periph_clk);
static void USARTx_clock_en(USART_TypeDef *UARTx);

void init_UARTx(UARTx_struct *UARTx_str)
{

	GPIO_UART_Config(UARTx_str->GPIOx_Tx, UARTx_str->PINx_Tx, UARTx_str->AFx);
	GPIO_UART_Config(UARTx_str->GPIOx_Rx, UARTx_str->PINx_Rx, UARTx_str->AFx);
	USARTx_clock_en(UARTx_str->UARTx);
	UARTx_str->UARTx->CR1 = 0; // clear the reg
	UARTx_str->UARTx->CR1 &= ~USARTx_en;
	// setup baudrate
	UARTx_str->UARTx->BRR = calculate_baudrate(UARTx_str->baudrate, APB1_clk);

	// set transfer direction:  fullduplex ( RX and TX )
	UARTx_str->UARTx->CR1 = CR1_RE | CR1_TE;
	// Enable USARTx NVIC
	// NVIC_EnableIRQ(USART2_IRQn);
	// enable USART2
	UARTx_str->UARTx->CR1 |= USARTx_en;
}

void UARTx_write(UARTx_struct *UARTx_str, int8_t data)
{
	while ((UARTx_str->UARTx->ISR & UART_SR_TXE) == 0)
	{
	}
	// write to the data register
	UARTx_str->UARTx->TDR = (data & 0xFF);
}
int8_t UARTx_read(UARTx_struct *UARTx_str)
{
	while ((UARTx_str->UARTx->ISR & UART_SR_RXNE) == 0)
	{
	}
	return UARTx_str->UARTx->RDR;
}

static void USARTx_clock_en(USART_TypeDef *UARTx)
{
	if (UARTx == USART2)
	{
		RCC->APBENR1 |= RCC_APBENR1_USART2EN;
	}
	else if (UARTx == USART3)
	{
		RCC->APBENR1 |= RCC_APBENR1_USART3EN;
	}
}
static uint16_t calculate_baudrate(uint32_t bd, uint32_t periph_clk)
{

	return ((periph_clk + (bd / 2U)) / bd);
}
