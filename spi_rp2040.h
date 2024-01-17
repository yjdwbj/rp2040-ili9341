#ifndef SPI_RP2040_INC_H
#define SPI_RP2040_INC_H
#include <stdio.h>

uint8_t transfer(uint8_t data);
uint16_t transfer16(uint16_t data);
void transfer_buf(uint8_t *buf, size_t count);
#endif // SPI_RP2040_INC_H