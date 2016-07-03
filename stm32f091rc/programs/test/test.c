#include <stm32f0xx.h>
#include <delay.h>
 
#define LED_PIN 7
#define LED_ON() GPIOB->BSRR |= (1 << LED_PIN)
#define LED_OFF() GPIOB->BSRR |= ((1 << LED_PIN) << 16)
#define LED_TOGGLE() GPIOB->ODR ^= (1 << LED_PIN)

void init_tim3(void);
void init_tim6(void);
void init_leds(void);
void init_pwm(void);
void set_pwm(uint8_t i);

void TIM6_IRQHandler(void) {
	LED_TOGGLE();
	TIM6->SR &= ~TIM_SR_UIF; // clear flag
}

int main() {
	NVIC_EnableIRQ(TIM6_IRQn);
	NVIC_SetPriority(TIM6_IRQn, 2);

	// init_leds();
	// init_tim6();
	init_delay();
	init_pwm();
	// init_tim3();
 	
	uint8_t i;

 	while(1) {
 		while(i < 100) {
 			set_pwm(i);
 			i++;
 			_delay_ms(10);
 		}
 		while(i > 0) {
 			set_pwm(i);
 			i--;
 			_delay_ms(10);
 		}
 	}

	return 0;
 
}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555; // set 8 pins to output
	GPIOB->OTYPER &= ~(0x00); // set to push-pull
	GPIOB->OSPEEDR &= ~(0x0000); // set low speed
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
