#include <stm32f0xx.h>
#include <delay.h>

#define LED_ON(led) GPIOB->BSRR |= (1 << led)
#define LED_OFF(led) GPIOB->BSRR |= ((1 << led) << 16)
#define LED_TOGGLE(led) GPIOB->ODR ^= (1 << led)

void init_leds(void);
void init_usart2(uint16_t baud);
void usart2_send(uint8_t* p_data, uint16_t count);
// void usart2_read(uint8_t* p_data, uint16_t count);

static volatile uint8_t usart2_tx_busy = 0; // status flag
static volatile uint8_t* p_tx_data; // pointer to tx buffer
// static volatile uint8_t* p_rx_data; // pointer to rx buffer
static volatile uint16_t usart2_tx_count; // number of bytes to send
static volatile uint16_t usart2_rx_count; // number of bytes to receive
static volatile uint16_t usart2_tx_incr; // increment variable

void USART2_IRQHandler(void) {
	LED_ON(7);
	if((USART2->ISR & USART_ISR_TC) == USART_ISR_TC) { // tc flag
		// TODO: stop
		usart2_tx_busy = 0; // set to available
		USART2->ICR |= USART_ICR_TCCF; // clear tc flag
	}
	if((USART2->ISR & USART_ISR_TXE) == USART_ISR_TXE) { // txe flag (TDR empty)
		// TODO: push next byte to TDR
		if(usart2_tx_incr + 1 >= usart2_tx_count) { // if last byte to transmit
			USART2->CR1 &= ~(USART_CR1_TXEIE); // disable interrupt
		}
		USART2->TDR = *(p_tx_data + usart2_tx_incr++); // push next byte
	}
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) { // rxne flag (RDR not empty)
		// TODO: push byte from rdr to rx buffer
	}
}

int main(void) {
	// setup
	init_leds();
	init_usart2(9600);
	init_delay();

	uint8_t data[3] = {'a', 'b', '\0'};


	// loop
	while(1) {
		_delay_ms(200);
		usart2_send(data, 2);
		LED_TOGGLE(0);
	}

	return 0;
}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555U; // set 8 pins to output
	GPIOB->OTYPER &= ~(0x11U); // set to push-pull
	GPIOB->OSPEEDR &= ~(0x1111U); // set low speed
}

void init_usart2(uint16_t baud) {
	// init pins
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // start clock
	GPIOA->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1); // set af mode
	GPIOA->AFR[0] |= 0x1100U; // set af1
	
	// init interrupts
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_SetPriority(USART2_IRQn, 2);
	
	// init usart
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // start clock
	USART2->BRR = SystemCoreClock / baud; // set baud rate
	USART2->CR1 |= (/*USART_CR1_TXEIE |*/ USART_CR1_TCIE); // enable interrupts
	USART2->CR1 |= (/*USART_CR1_TE |*/ USART_CR1_UE); // enable periph
}

void usart2_send(uint8_t* p_data, uint16_t count) {
	while(usart2_tx_busy); // check if available
	usart2_tx_busy = 1; // set to busy
	usart2_tx_incr = 0;
	usart2_tx_count = count; // set number of bytes to send
	p_tx_data = p_data; // set buffer pointer
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_TXEIE); // enable interrupt
}

// void usart2_read(uint8_t* p_data, uint16_t count) {
// 	while(usart2_rx_busy); // check if available
// 	usart2_rx_busy = 1; // set to busy
// }
