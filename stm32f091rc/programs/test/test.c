#include <stm32f0xx.h>
#include <delay.h>
// #include <usart.h>

#define I2C_DELIMITER '\0'
#define LED_ON(led) GPIOB->BSRR |= (1 << led)
#define LED_OFF(led) GPIOB->BSRR |= ((1 << led) << 16)
#define LED_TOGGLE(led) GPIOB->ODR ^= (1 << led)
#define SLAVE_ADDR 0b1101000
#define REG_ADDR_1 0x0e
#define REG_ADDR_2 0x00

volatile uint8_t* i2c_data_ptr;

void init_leds(void);
void init_i2c1(void);
void i2c1_send(uint8_t slave_addr, uint8_t* data_ptr);
// void i2c1_read(uint8_t slave_addr, uint8_t addr, uint8_t i, uint8_t* data_ptr);
void init_adc(void);
void read_adc(uint16_t* adc_values);
void init_tim3(void);
void set_pwm(uint8_t i);
void itoa(uint8_t* str, uint8_t len, uint32_t val);
uint8_t zeroes(uint32_t	reg);

void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void) {
	LED_TOGGLE(7);
	if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2) { // ch2 tc flag
		LED_TOGGLE(5);
		DMA1_Channel2->CCR &= ~(DMA_CCR_EN); // turn off periph
		DMA1->IFCR |= DMA_IFCR_CTCIF2; // clear interrupt flag
	}
	if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3) { // ch3 tc flag
		DMA1_Channel3->CCR &= ~(DMA_CCR_EN); // turn off periph
		DMA1->IFCR |= DMA_IFCR_CTCIF3; // clear interrupt flag
	}
}

void I2C1_IRQHandler(void) {
	LED_TOGGLE(6);
	if((I2C1->ISR & I2C_ISR_NACKF) == I2C_ISR_NACKF) {
		LED_TOGGLE(5);
		I2C1->ICR |= I2C_ICR_NACKCF;
	}
	if((I2C1->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS) {
		LED_ON(4);
		I2C1->TXDR = *i2c_data_ptr++;
	}
}

int main() {
	init_leds();
	init_delay();
	init_i2c1();

	uint8_t rtc_setup[] = {REG_ADDR_1, 0b01001000};

	i2c1_send(SLAVE_ADDR, rtc_setup);

	while(1) {
		_delay_ms(200);
		LED_TOGGLE(0);
	}

	return 0;

}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555UL; // set 8 pins to output
}

void init_i2c1(void) {
	// init dma interrupt
	// NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
	// NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 2);
	NVIC_EnableIRQ(I2C1_IRQn);
	NVIC_SetPriority(I2C1_IRQn, 2);
	
	// init pins
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // start clock
	GPIOB->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER8_1); // set af mode
	GPIOB->AFR[1] |= 0x11U; // set af1
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR8_0); // set pull ups
	
	// // init dma
	// RCC->AHBENR |= RCC_AHBENR_DMA1EN; // start clock
	// DMA1->CSELR |= (DMA1_CSELR_CH2_I2C1_TX /*| DMA1_CSELR_CH3_I2C1_RX*/); // set channel selection
	// // init channel2
	// DMA1_Channel2->CCR |= DMA_CCR_PL_1; // set high priority
	// DMA1_Channel2->CCR |= DMA_CCR_MINC; // set memory increment mode
	// DMA1_Channel2->CCR |= DMA_CCR_DIR; // set read from memory
	// DMA1_Channel2->CCR |= DMA_CCR_TCIE; // turn on tc interrupt
	// DMA1_Channel2->CCR &= ~(DMA_CCR_MSIZE); // set 8bit memory size
	// DMA1_Channel2->CCR &= ~(DMA_CCR_PSIZE); // set 8bit periph size
	// DMA1_Channel2->CPAR = (uint32_t)&(I2C1->TXDR); // set periph address
	// // init channel3
	// DMA1_Channel3->CCR |= DMA_CCR_PL_1; // set high priority
	// DMA1_Channel3->CCR |= DMA_CCR_MINC; // set memory increment mode
	// DMA1_Channel3->CCR |= DMA_CCR_TCIE; // turn on tc interrupt
	// DMA1_Channel3->CCR &= ~(DMA_CCR_DIR); // set read from periph
	// DMA1_Channel3->CCR &= ~(DMA_CCR_MSIZE); // set 8bit memory size
	// DMA1_Channel3->CCR &= ~(DMA_CCR_PSIZE); // set 8bit periph size
	// DMA1_Channel3->CPAR = (uint32_t)&(I2C1->RXDR); // set periph address
	
	// init i2c1
	RCC->CFGR3 |= RCC_CFGR3_I2C1SW_SYSCLK; // select SysClock as clock source
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // start clock
	I2C1->TIMINGR = (uint32_t)0x10905F86; // set timing register (Sm 100kHz)
	I2C1->CR1 |= (I2C_CR1_NACKIE | I2C_CR1_TXIE); // enable tx and nack interrupt
	I2C1->CR1 |= I2C_CR1_PE; // enable periph
	// I2C1->CR1 |= (I2C_CR1_TXDMAEN /*| I2C_CR1_RXDMAEN*/); // enable dma on tx/rx
}

void i2c1_send(uint8_t slave_addr, uint8_t* data_ptr) {
	uint8_t count = 0;
	i2c_data_ptr = data_ptr;

	I2C1->CR2 &= ~(I2C_CR2_RD_WRN); // set write direction
	while(*(data_ptr + count) != I2C_DELIMITER) {
		count++;
	}
	I2C1->CR2 &= ~(I2C_CR2_SADD); // clear address
	I2C1->CR2 |= (slave_addr << 1); // set slave address
	I2C1->CR2 |= I2C_CR2_AUTOEND; // enable autoend
	I2C1->CR2 &= ~(I2C_CR2_NBYTES); // clear number of bytes
	I2C1->CR2 |= (count << 16); // set number of bytes to send

	// DMA1_Channel2->CMAR = (uint32_t)data_ptr; // set memory address
	// DMA1_Channel2->CCR |= DMA_CCR_EN; // turn on perpiph

	I2C1->CR2 |= I2C_CR2_START; // start transmition
	LED_TOGGLE(count);
}

void init_adc(void) {
	// init pin
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // turn clock on
	GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER5_0); // set analog mode
	GPIOA->MODER |= (GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0); // set analog mode

	// init dma
	RCC->AHBENR |= RCC_AHBENR_DMA2EN; // turn clock on
	DMA2->CSELR |= (DMA2_CSELR_CH5_ADC); // set channel selection
	// init channel5
	DMA2_Channel5->CCR |= DMA_CCR_PL_0; // set medium priority
	DMA2_Channel5->CCR |= DMA_CCR_MINC; // set memory increment mode
	DMA2_Channel5->CCR |= DMA_CCR_CIRC; // turn on circular mode
	DMA2_Channel5->CCR |= DMA_CCR_MSIZE_0; // set 16bit memory size
	DMA2_Channel5->CCR |= DMA_CCR_PSIZE_0; // set 16bit periph size
	DMA2_Channel5->CCR &= ~(DMA_CCR_DIR); // set read from periph
	DMA2_Channel5->CPAR = (uint32_t)&(ADC1->DR); // set periph address
	DMA2_Channel5->CNDTR = 0x2UL; // set to 2 channels

	// init adc
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // turn clock on
	RCC->CR2 |= RCC_CR2_HSI14ON; // turn hsi14 on
	while((RCC->CR2 & RCC_CR2_HSI14RDY) == 0); // wait for hsi14
	if((ADC1->CR & ADC_CR_ADEN) == ADC_CR_ADEN) { // if enabled
		ADC1->CR &= ~(ADC_CR_ADEN); // disable
	}
	ADC1->CR |= ADC_CR_ADCAL; // start calibration
	while((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL); // wait for calibration
	ADC1->CR |= ADC_CR_ADEN; // enable
	while((ADC1->ISR & ADC_ISR_ADRDY) == ADC_ISR_ADRDY); // wait for enable
	ADC1->CHSELR |= (ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL0); // select ch5 and ch0
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; // enable auto off
	ADC1->CFGR1 |= (ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG); // enable circular dma
	ADC1->SMPR |= ADC_SMPR_SMP_2; // set sampling time
}

void read_adc(uint16_t* adc_values) {
	while((ADC1->CR & ADC_CR_ADSTART) == ADC_CR_ADSTART); // check if available
	DMA2_Channel5->CMAR = (uint32_t)adc_values; // set memory address
	DMA2_Channel5->CCR |= DMA_CCR_EN; // enable periph
	ADC1->CR |= ADC_CR_ADSTART; // start conversion
}

void init_tim3(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // turn clock on
	TIM3->PSC = 48 - 1; // set prescaler
	TIM3->ARR = 1000; // set period (1ms)
	TIM3->CCR1 = 1; // set duty cycle (.5)
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // set pwm mode
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // enable preload
	TIM3->CCER |= TIM_CCER_CC1E; // enable output ch1
	TIM3->CR2 |= (TIM_CR1_CEN /*| TIM_CR1_URS*/); // turn on periph
}

void set_pwm(uint8_t i) {
	TIM3->CCR1 = i;
}

void itoa(uint8_t* str, uint8_t len, uint32_t val) {
	for(uint8_t i = 1; i <= len; i++) {
		str[len - i] = (uint8_t)((val % 10UL) + '0'); // set char
		val /= 10; // cycle
	}
	str[len] = '\0'; // close string
}

uint8_t zeroes(uint32_t reg) {
	uint8_t count = 0;
	
	while(reg != 0) {
		if(reg & 1) {
			break;
		} else {
			count++;
			reg = reg >> 1;
		}
	}
	return count;
}
