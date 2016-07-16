#include <stm32f0xx.h>
#include <delay.h>
#include "usart.h"

void init_leds(void);

volatile uint8_t usart2_rx_busy;

int main(void) {
	// setup
	init_leds();
	init_usart2(9600);
	init_delay();
	uint8_t read_data[4];
	read_data[2] = '\0';

	// loop
	while(1) {
		// echo 2 chars
		usart2_read(read_data, 2);
		while(usart2_rx_busy);
		usart2_send(read_data);
		_delay_ms(10);
	}

	return 0;
}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555U; // set 8 pins to output
	GPIOB->OTYPER &= ~(0x11U); // set to push-pull
	GPIOB->OSPEEDR &= ~(0x1111U); // set low speed
}
