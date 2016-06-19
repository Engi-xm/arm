#include "stm32f0xx.h"
#include "delay.h"

static void systick_stop() {
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
}

static void systick_start() {
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}


void init_delay() {
	SysTick->LOAD = 8;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
}

void _delay_us(uint32_t us) {
	systick_start();
	while (us != 0) {
		while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {}
		--us;
	}
	systick_stop();
}

inline void _delay_ms(uint32_t ms) {
	_delay_us(ms * 1000);
}