#ifndef __DELAY_H_
#define __DELAY_H_

/** Initialize delay core - configure SysTick timer */
void init_delay();

void _delay_us(uint32_t us);
void _delay_ms(uint32_t ms);

#endif
