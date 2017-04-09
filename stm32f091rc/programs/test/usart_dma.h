#ifndef __USART_DMA_H_
#define __USART_DMA_H_

void init_usart2(uint16_t baud);
void usart2_send(uint8_t* data_ptr, uint16_t count);
void usart2_read(uint8_t* data_ptr, uint16_t count);

extern volatile uint8_t usart2_rx_busy; // status flag

#endif
