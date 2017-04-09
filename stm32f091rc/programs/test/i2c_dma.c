#include "stm32f0xx.h"
#include "i2c_dma.h"

volatile uint8_t i2c1_rx_busy = 0; // status flag
volatile uint8_t i2c1_tx_busy = 0; // status flag

void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void) {
	if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2) { // ch2 tc flag
		i2c1_tx_busy = 0; // set to available
		DMA1_Channel2->CCR &= ~(DMA_CCR_EN); // turn off periph
		DMA1->IFCR |= DMA_IFCR_CTCIF2; // clear interrupt flag
	}
	if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3) { // ch3 tc flag
		i2c1_rx_busy = 0; // set to available
		DMA1_Channel3->CCR &= ~(DMA_CCR_EN); // turn off periph
		DMA1->IFCR |= DMA_IFCR_CTCIF3; // clear interrupt flag
	}
}

void init_i2c1(void) {
	// init dma interrupt
	NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
	NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 2);
	
	// init pins
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // start clock
	GPIOB->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER8_1); // set af mode
	GPIOB->AFR[1] |= 0x11U; // set af1
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_8); // set open drain output
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR8_0); // set pull ups
	
	// // init dma
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // start clock
	DMA1->CSELR |= (DMA1_CSELR_CH2_I2C1_TX | DMA1_CSELR_CH3_I2C1_RX); // set channel selection
	// init channel2
	DMA1_Channel2->CCR |= DMA_CCR_PL_1; // set high priority
	DMA1_Channel2->CCR |= DMA_CCR_MINC; // set memory increment mode
	DMA1_Channel2->CCR |= DMA_CCR_TCIE; // turn on tc interrupt
	DMA1_Channel2->CCR |= DMA_CCR_DIR; // set read from memory
	DMA1_Channel2->CCR &= ~(DMA_CCR_MSIZE); // set 8bit memory size
	DMA1_Channel2->CCR &= ~(DMA_CCR_PSIZE); // set 8bit periph size
	DMA1_Channel2->CPAR = (uint32_t)&(I2C1->TXDR); // set periph address
	// init channel3
	DMA1_Channel3->CCR |= DMA_CCR_PL_1; // set high priority
	DMA1_Channel3->CCR |= DMA_CCR_MINC; // set memory increment mode
	DMA1_Channel3->CCR |= DMA_CCR_TCIE; // turn on tc interrupt
	DMA1_Channel3->CCR &= ~(DMA_CCR_DIR); // set read from periph
	DMA1_Channel3->CCR &= ~(DMA_CCR_MSIZE); // set 8bit memory size
	DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE); // set 8bit periph size
	DMA1_Channel3->CPAR = (uint32_t)&(I2C1->RXDR); // set periph address
	
	// init i2c1
	RCC->CFGR3 |= RCC_CFGR3_I2C1SW_SYSCLK; // select SysClock as clock source
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // start clock
	I2C1->TIMINGR = (uint32_t)0x10905F86; // set timing register (Sm 100kHz)
	I2C1->CR1 |= I2C_CR1_PE; // enable periph
	I2C1->CR1 |= (I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN); // enable dma on tx/rx
}

void i2c1_send(uint8_t slave_addr, uint8_t addr, uint8_t* data_ptr, uint8_t count) {
	while(i2c1_tx_busy); // check if available
	i2c1_tx_busy = 1; // set to busy
	for(int8_t i = count; i >= 0; i--) { // shift data array
		*(data_ptr + i + 1) = *(data_ptr + i);
	}
	*(data_ptr) = addr; // set register address
	count++;
	
	I2C1->CR2 &= ~(I2C_CR2_RD_WRN); // set write direction
	I2C1->CR2 &= ~(I2C_CR2_SADD); // clear address
	I2C1->CR2 |= (slave_addr << 1); // set slave address
	I2C1->CR2 |= I2C_CR2_AUTOEND; // enable autoend
	I2C1->CR2 &= ~(I2C_CR2_NBYTES); // clear number of bytes
	I2C1->CR2 |= (count << 16); // set number of bytes to send

	DMA1_Channel2->CMAR = (uint32_t)data_ptr; // set memory address
	DMA1_Channel2->CNDTR = count; // set number of bytes to send
	DMA1_Channel2->CCR |= DMA_CCR_EN; // turn on perpiph

	I2C1->CR2 |= I2C_CR2_START; // start transmition
}

void i2c1_read(uint8_t slave_addr, uint8_t addr, uint8_t* data_ptr, uint8_t count) {
	uint8_t dummy[1] = {0}; // set dummy array
	
	while(i2c1_rx_busy); // check if available
	i2c1_rx_busy = 1; // set to busy
	i2c1_send(slave_addr, addr, dummy, 0); // set register to read from
	while(i2c1_tx_busy); // wait for delivery

	I2C1->CR2 |= I2C_CR2_RD_WRN; // set read direction
	I2C1->CR2 |= I2C_CR2_AUTOEND; // enable autoend
	I2C1->CR2 &= ~(I2C_CR2_SADD); // clear address
	I2C1->CR2 |= (slave_addr << 1); // set slave address
	I2C1->CR2 &= ~(I2C_CR2_NBYTES); // clear number of bytes
	I2C1->CR2 |= (count << 16); // set number of bytes to send
	
	DMA1_Channel3->CMAR = (uint32_t)data_ptr; // set memory address
	DMA1_Channel3->CNDTR = count; // set number of bytes to read
	DMA1_Channel3->CCR |= DMA_CCR_EN; // turn on periph

	I2C1->CR2 |= I2C_CR2_START; // start transmition
}
