#include "stm32f0xx.h"
#include "usart.h"

volatile uint8_t usart2_rx_busy = 0; // status flag

void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void) {
	if((DMA2->ISR & DMA_ISR_TCIF1) == DMA_ISR_TCIF1) { // ch1 tc flag
		DMA2_Channel1->CCR &= ~(DMA_CCR_EN); // turn off periph
		DMA2->IFCR |= DMA_IFCR_CTCIF1; // clear interrupt flag
	}
	if((DMA2->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2) { // ch2 tc flag
		usart2_rx_busy = 0; // set to available
		DMA2_Channel2->CCR &= ~(DMA_CCR_EN); // turn off periph
		DMA2->IFCR |= DMA_IFCR_CTCIF2; // clear interrupt flag
	}
}

void init_usart2(uint16_t baud) {
	// init dma interrupt
	NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
	NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 2);
	
	// init pins
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // start clock
    GPIOA->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER2_1); // set af mode
    GPIOA->AFR[0] |= 0x1100U; // set af1
	
	// init dma
    RCC->AHBENR |= RCC_AHBENR_DMA2EN; // start clock
	DMA2->CSELR |= (DMA2_CSELR_CH1_USART2_TX | DMA2_CSELR_CH2_USART2_RX); // set channel selection
	// init channel1
	DMA2_Channel1->CCR |= DMA_CCR_PL_1; // set high priority
	DMA2_Channel1->CCR |= DMA_CCR_MINC; // set memory increment mode
	DMA2_Channel1->CCR |= DMA_CCR_DIR; // set read from memory
	DMA2_Channel1->CCR |= DMA_CCR_TCIE; // turn on tc interrupt
	DMA2_Channel1->CCR &= ~(DMA_CCR_MSIZE); // set 8bit memory size
	DMA2_Channel1->CCR &= ~(DMA_CCR_PSIZE); // set 8bit periph size
	DMA2_Channel1->CPAR = (uint32_t)&(USART2->TDR); // set periph address
	// init channel2
	DMA2_Channel2->CCR |= DMA_CCR_PL_1; // set high priority
	DMA2_Channel2->CCR |= DMA_CCR_MINC; // set memory increment mode
	DMA2_Channel2->CCR |= DMA_CCR_TCIE; // turn on tc interrupt
	DMA2_Channel2->CCR &= ~(DMA_CCR_DIR); // set read from periph
	DMA2_Channel2->CCR &= ~(DMA_CCR_MSIZE); // set 8bit memory size
	DMA2_Channel2->CCR &= ~(DMA_CCR_PSIZE); // set 8bit periph size
	DMA2_Channel2->CPAR = (uint32_t)&(USART2->RDR); // set periph address
	
    // init usart
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // start clock
    USART2->BRR = SystemCoreClock / baud; // set baud rate
    USART2->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR); // enable dma on tx/rx
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

void usart2_read(uint8_t* data_ptr, uint16_t count) {
	while(usart2_rx_busy); // check if available
	usart2_rx_busy = 1; // set to busy
	DMA2_Channel2->CNDTR = count; // set number of bytes to read
	DMA2_Channel2->CMAR = (uint32_t)data_ptr; // set memory address
	DMA2_Channel2->CCR |= DMA_CCR_EN; // turn on periph
}
