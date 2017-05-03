#ifndef __I2C_DMA_H_
#define __I2C_DMA_H_

void init_i2c1(void); // initialize i2c @100kHz
void i2c1_send(uint8_t slave_addr, uint8_t addr, uint8_t* data_ptr, uint8_t count); // send message over i2c
void i2c1_read(uint8_t slave_addr, uint8_t addr, uint8_t* data_ptr, uint8_t count); // read message over i2c

extern volatile uint8_t i2c1_rx_busy; // status flag

#endif
