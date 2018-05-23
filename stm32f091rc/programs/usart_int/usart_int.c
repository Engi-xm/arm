#include <stm32f0xx.h>
#include "usart_int.h"

// global variables
volatile uint8_t usart2_rx_busy = 0; // status flag

// private variables
static volatile uint8_t usart2_tx_busy = 0; // status flag
static volatile uint8_t* p_tx_data; // pointer to tx buffer
// static volatile uint8_t* p_rx_data; // pointer to rx buffer
static volatile uint16_t usart2_tx_count; // number of bytes to send
static volatile uint16_t usart2_rx_count; // number of bytes to receive
static volatile uint16_t usart2_tx_incr; // increment variable

void USART2_IRQHandler(void) {
	if((USART2->ISR & USART_ISR_TC) == USART_ISR_TC) { // tc flag
		// TODO: stop
		usart2_tx_busy = 0; // set to available
		USART2->CR1 &= ~(USART_CR1_TE); // disable transmiter
		USART2->ICR |= USART_ICR_TCCF; // clear tc flag
	}
	if((USART2->ISR & USART_ISR_TXE) == USART_ISR_TXE) { // txe flag (TDR empty)
		// TODO: push next byte to TDR
		USART2->TDR = *(p_tx_data + usart2_tx_incr++);
	}
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) { // rxne flag (RDR not empty)
		// TODO: push byte from rdr to rx buffer
	}
}

void init_usart2(uint16_t baud) {
	// init pins
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // start clock
	GPIOA->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1); // set af mode
	GPIOA->AFR[0] |= 0x1100U; // set af1
	
	// init usart
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // start clock
	USART2->BRR = SystemCoreClock / baud; // set baud rate
	USART2->CR1 |= (USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE); // enable interrupts
	USART2->CR1 |= (USART_CR1_RE | USART_CR1_UE); // enable periph
}

void usart2_send(uint8_t* p_data, uint16_t count) {
	while(usart2_tx_busy); // check if available
	LED_TOGGLE(1);
	usart2_tx_busy = 1; // set to busy
	usart2_tx_incr = 0;
	usart2_tx_count = count; // set number of bytes to send
	p_tx_data = p_data; // set buffer pointer
	USART2->CR1 |= USART_CR1_TE; // enable transmiter
}

void usart2_read(uint8_t* p_data, uint16_t count) {
	while(usart2_rx_busy); // check if available
	usart2_rx_busy = 1; // set to busy
}
