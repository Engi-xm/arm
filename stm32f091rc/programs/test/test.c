#include <stm32f0xx.h>
#include <delay.h>

#define LED_ON(led) GPIOB->BSRR |= (1 << led)
#define LED_OFF(led) GPIOB->BSRR |= ((1 << led) << 16)
#define LED_TOGGLE(led) GPIOB->ODR ^= (1 << led)
#define SLAVE_ADDR 0b1101000
#define REG_ADDR_1 0x0e
#define REG_ADDR_2 0x00

void init_leds(void);
void init_adc(void);
void read_adc(uint16_t* adc_values);
void init_tim3(void);
void set_pwm(uint8_t i);
void itoa(uint8_t* str, uint8_t len, uint32_t val);
uint8_t zeroes(uint32_t reg);

int main() {
	init_leds();
	init_delay();
	// init_usart2(9600);
	// init_i2c1();
	// uint8_t rtc_setup[] = {0b00010000};
	// uint8_t read_data[5] = {0};
	// uint8_t data[3] = {'a', 'b', '\0'};

	// i2c1_send(SLAVE_ADDR, REG_ADDR_1, rtc_setup, 1);

	while(1) {
		_delay_ms(500);
		// usart2_send(data, 2);
		LED_TOGGLE(2);

		// _delay_ms(200);
		// i2c1_read(SLAVE_ADDR, REG_ADDR_2, read_data, 1);
		// GPIOB->ODR = (read_data[0] >> 4) * 10 + (read_data[0] & 0x0f);
	}

	return 0;

}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555UL; // set 8 pins to output
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
