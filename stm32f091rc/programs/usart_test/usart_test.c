#include <stm32f0xx.h>
#include <delay.h>

#define USART_DELIMITER '\0'

void init_leds(void);
void init_usart2(uint16_t baud);
uint8_t usart2_read_byte(void);
void usart2_send(uint8_t* data_ptr);
// uint8_t peek(uint8_t* ptr);

void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void) {
	DMA2_Channel1->CCR &= ~(DMA_CCR_EN); // turn off periph
	DMA2->IFCR |= DMA_IFCR_CTCIF1; // clear interrupt flag
}

int main(void) {
	// setup
	NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
	NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 2);
	init_leds();
	init_usart2(9600);
	init_delay();
	uint8_t send_data[4] = {0x61, 0x62, 0x63, '\0'};
	// uint8_t data_byte = 0x63;

	// loop
	while(1) {
		// if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
		// 	GPIOB->ODR = usart2_read_byte();
		// }
		// _delay_us(500);
				
		GPIOB->ODR ^= 0x0f;
		usart2_send(send_data);
		_delay_ms(500);
	}

	return 0;
}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555; // set 8 pins to output
	GPIOB->OTYPER &= ~(0x11); // set to push-pull
	GPIOB->OSPEEDR &= ~(0x1111); // set low speed
}

void init_usart2(uint16_t baud) {
	// init pins
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // start clock
    GPIOA->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1); // set af mode
    GPIOA->AFR[0] |= 0x1100U; // set af1
	
	// init dma channel
    RCC->AHBENR |= RCC_AHBENR_DMA2EN; // start clock
	DMA2->CSELR |= 0x9; // set channel selection
	DMA2_Channel1->CCR |= DMA_CCR_PL_1; // set high priority
	DMA2_Channel1->CCR |= DMA_CCR_MINC; // set memory increment mode
	DMA2_Channel1->CCR |= DMA_CCR_DIR; // set read from memory
	DMA2_Channel1->CCR |= DMA_CCR_TCIE; // turn on tc interrupt
	DMA2_Channel1->CCR &= ~(DMA_CCR_MSIZE); // set 8bit memory size
	DMA2_Channel1->CCR &= ~(DMA_CCR_PSIZE); // set 8bit periph size
	DMA2_Channel1->CPAR = (uint32_t)&(USART2->TDR); // set periph address
	
    // init usart
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // start clock
    USART2->BRR = SystemCoreClock / baud; // set baud rate
    USART2->CR3 |= USART_CR3_DMAT; // enable dma on tx
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE); // enable periph
}

void usart2_send(uint8_t* data_ptr) {
	uint16_t count = 0;

	while(*(data_ptr + count) != USART_DELIMITER) {
		count++;
	}
	DMA2_Channel1->CNDTR = count; // set number of bytes to send
	DMA2_Channel1->CMAR = (uint32_t)data_ptr; // set memory address
	DMA2_Channel1->CCR |= DMA_CCR_EN; // turn on periph
}

// uint8_t usart2_read_byte(void) {
// 	return (uint8_t)USART2->RDR;
// }

// uint8_t peek(uint8_t* ptr) {
// 	return *(++ptr); // return incremented pointer
// }