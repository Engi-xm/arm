#include <stm32f0xx.h>
#include <delay.h>

#define LED_ON(led) GPIOB->BSRR |= (1 << led)
#define LED_OFF(led) GPIOB->BSRR |= ((1 << led) << 16)
#define LED_TOGGLE(led) GPIOB->ODR ^= (1 << led)

void init_leds(void);

int main() {
	init_leds();
	init_delay();

	while(1) {
		_delay_ms(2000);
		LED_TOGGLE(2);

	}

	return 0;

}

void init_leds(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // turn clock on
	GPIOB->MODER |= 0x5555UL; // set 8 pins to output
}
