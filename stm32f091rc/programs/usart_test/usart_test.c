#include <stm32f0xx.h>
#include <delay.h>

#define USART_DELIMITER '\0'

void init_leds(void);
void init_usart4(uint16_t baud);
void usart4_send_byte(uint8_t data);
// void usart2_stop_tx(void);
// uint8_t usart2_send(uint8_t* data_ptr);
// uint8_t peek(uint8_t* ptr);

// volatile uint8_t* usart2_tx_buf_ptr;
// volatile uint8_t tx_busy = 0;

// void USART2_IRQHandler(void) {
// 	if((USART2->ISR & USART_ISR_TXE) == USART_ISR_TXE) { // if TXE flag set
// 		if(*usart2_tx_buf_ptr == USART_DELIMITER) { // if reached end
// 			USART2->CR1 |= USART_CR1_TCIE; // enable TC interrupt
// 			USART2->CR1 &= ~USART_CR1_TXEIE; // disable txe interrupt
// 		} else {
// 			usart2_send_byte(*(usart2_tx_buf_ptr++)); // send byte and increment
// 		}
// 	}

// 	if((USART2->ISR & USART_ISR_TC) == USART_ISR_TC) { // if TC flag set
// 		usart2_stop_tx(); // stop transmission
// 	}
// }

int main(void) {
	// setup
	// NVIC_EnableIRQ(USART2_IRQn);
	// NVIC_SetPriority(USART2_IRQn, 2);
	init_leds();
	init_usart4(9600);
	init_delay();
	// uint8_t send_data[3] = {0x61, 0x62, '\0'};
	uint8_t data_byte = 0x63;

	// loop
	while(1) {
		_delay_ms(200);
		usart4_send_byte(data_byte);
	}

	return 0;
}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555; // set 8 pins to output
	GPIOB->OTYPER &= ~(0x00); // set to push-pull
	GPIOB->OSPEEDR &= ~(0x0000); // set low speed
}

void init_usart4(uint16_t baud) {
	// init pins
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // start clock
    GPIOA->MODER |= (GPIO_MODER_MODER1_1 | GPIO_MODER_MODER0_1); // set af mode
    GPIOA->AFR[0] |= 0x44U; // set af4
    GPIOA->PUPDR |= 0x5U; // set pull ups

    // init usart
    RCC->APB1ENR |= RCC_APB1ENR_USART4EN; // start clock
    USART4->BRR = SystemCoreClock / baud; // set baud rate
    USART4->CR1 |= (USART_CR1_TE | USART_CR1_UE); // enable periph
}

void usart4_send_byte(uint8_t data) {
	USART4->TDR = data; // load data to buffer
}

// void usart2_stop_tx(void) {
// 	USART2->CR1 &= ~USART_CR1_TCIE; // disable TC interrupt
// 	USART2->ICR |= USART_ICR_TCCF; // clear flag
// 	tx_busy = 0; // set to available
// }

// uint8_t usart2_send(uint8_t* data_ptr) {
// 	while(tx_busy); // check if available
// 	tx_busy = 1; // set to busy
// 	usart2_tx_buf_ptr = data_ptr; // set pointer
// 	if(*data_ptr == USART_DELIMITER) {
// 		return 1;
// 	}
//     USART2->CR1 |= USART_CR1_TXEIE; // enable tx interrupt
//     return 0;
// }

// uint8_t peek(uint8_t* ptr) {
// 	return *(++ptr); // return incremented pointer
// }
