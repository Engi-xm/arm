#ifndef __USART_INT_H_
#define __USART_INT_H_

void init_usart2(uint16_t baud);
void usart2_send(uint8_t* p_data, uint16_t count);
void usart2_read(uint8_t* p_data, uint16_t count);

extern volatile uint8_t usart2_rx_busy; // status flag

#endif
