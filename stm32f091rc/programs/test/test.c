#include <stm32f0xx.h>
#include <delay.h>
#include <usart.h>
 
#define LED_PIN 6
#define LED_ON() GPIOB->BSRR |= (1 << LED_PIN)
#define LED_OFF() GPIOB->BSRR |= ((1 << LED_PIN) << 16)
#define LED_TOGGLE() GPIOB->ODR ^= (1 << LED_PIN)

void init_tim3(void); // pwm timer
void init_tim6(void); // interrupt timer
void init_leds(void);
void init_pwm(void);
void init_adc(void);
uint16_t read_adc(void);
void set_pwm(uint8_t i);
void itoa(uint8_t* str, uint8_t len, uint32_t val);

void TIM6_IRQHandler(void) {
	LED_TOGGLE();
	TIM6->SR &= ~TIM_SR_UIF; // clear flag
}

int main() {
	init_leds();
	init_delay();
	init_adc();
	init_usart2(9600);
	// init_tim6();
	// init_pwm();
	// init_tim3();
 	
 	uint16_t adc_value;
 	uint8_t adc_value_str[5];
 	uint8_t new_line[3] = {'\n', '\r','\0'};

 	while(1) {
 		adc_value = read_adc();
 		itoa(adc_value_str, 4, adc_value);
 		usart2_send(adc_value_str);
 		usart2_send(new_line);
 		GPIOB->ODR = (adc_value >> 4);
 		_delay_ms(200);
 	}

	return 0;
 
}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555; // set 8 pins to output
	GPIOB->OTYPER &= ~(0x11); // set to push-pull
	GPIOB->OSPEEDR &= ~(0x1111); // set low speed
}

void init_adc(void) {
	// init pin
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // turn clock on
	GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER5_0); // set analog mode

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
	ADC1->CHSELR |= ADC_CHSELR_CHSEL5; // select ch5
	ADC1->CFGR1 |= ADC_CFGR1_AUTOFF; // enable auto off
	ADC1->SMPR |= ADC_SMPR_SMP_2; // set sampling time
}

void init_pwm(void) {
	// init timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // start clock
    TIM3->CR1 |= TIM_CR1_ARPE; // enable preload
    TIM3->PSC = 480 - 1; // 100kHz
    TIM3->ARR = 100 - 1; // number of steps
    TIM3->CCR1 = 80; // duty cycle

    // init output
    TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); // set pwm mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // enable preload
    TIM3->CCER |= TIM_CCER_CC1E; // enable output

    // init pin
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // start clock
    GPIOB->MODER |= GPIO_MODER_MODER4_1; // set af mode
    GPIOB->AFR[0] |= 0x10000U; // set af1

    // final init
    TIM3->EGR |= TIM_EGR_UG; // generate update to load registers
    TIM3->CR1 |= TIM_CR1_CEN; // start timer
}

void init_tim6(void) {
	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn, 2);
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // turn clock on
	TIM6->DIER |= TIM_DIER_UIE; // turn interrupts on
	TIM6->PSC = 48000 - 1; // set prescaller 
	TIM6->ARR = 500 - 1; // set limit
	TIM6->CR1 |= (TIM_CR1_CEN | TIM_CR1_URS); // turn on periph
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

uint16_t read_adc(void) {
	while((ADC1->CR & ADC_CR_ADSTART) == ADC_CR_ADSTART); // check if available
	ADC1->CR |= ADC_CR_ADSTART; // start conversion
	while((ADC1->ISR & ADC_ISR_EOS) == 0); // wait for conversion
	return (uint16_t)ADC1->DR; // return result
}

void itoa(uint8_t* str, uint8_t len, uint32_t val) {
	for(uint8_t i = 1; i <= len; i++) {
		str[len - i] = (uint8_t)((val % 10UL) + '0');
		val /= 10;
	}
	str[len] = '\0';
}
