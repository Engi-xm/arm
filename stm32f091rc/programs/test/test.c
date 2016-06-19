#include <stm32f0xx.h>
#include <delay.h>
 
#define LED_PIN 5
#define LED_ON() GPIOA->BSRR |= (1 << LED_PIN)
#define LED_OFF() GPIOA->BSRR |= ((1 << LED_PIN) << 16)
#define LED_TOGGLE() GPIOA->ODR ^= (1 << LED_PIN)

void init_tim7(void);
void init_led(void);

void TIM7_IRQHandler(void) {
	LED_TOGGLE();
	TIM7->SR &= ~TIM_SR_UIF; // clear flag
}

int main() {
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 2);

	init_led();
	init_tim7();
	// init_delay();
 
 	while(1) {
 		// LED_ON();
 		// _delay_ms(500);
 		// LED_OFF();
 		// _delay_ms(500);
 	}

	return 0;
 
}

void init_led(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // turn clock on
	GPIOA->MODER |= (1 << (LED_PIN << 1)); // set to output
	GPIOA->OSPEEDR |= (3 << (LED_PIN << 1)); // set max speed
}

void init_tim7(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // turn clock on
	TIM7->DIER |= TIM_DIER_UIE; // turn interrupts on
	TIM7->PSC = 8000; // set prescaller 
	TIM7->ARR = 200; // set limit
	TIM7->CR1 |= (TIM_CR1_CEN | TIM_CR1_URS); // turn on periph
}
