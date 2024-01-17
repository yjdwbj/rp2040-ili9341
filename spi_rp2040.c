#include "user_config.h"
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/spi.h>
#include <hardware/structs/iobank0.h>

uint8_t transfer(uint8_t data) {
    uint8_t ret;
    hw_write_masked(&spi_get_hw(SPI_IDX)->cr0, (8 - 1) << SPI_SSPCR0_DSS_LSB, SPI_SSPCR0_DSS_BITS); // Fast set to 8-bits
    spi_write_read_blocking(SPI_IDX, &data, &ret, 1);
    return ret;
}

uint16_t transfer16(uint16_t data) {
    uint16_t ret;
    hw_write_masked(&spi_get_hw(SPI_IDX)->cr0, (16 - 1) << SPI_SSPCR0_DSS_LSB, SPI_SSPCR0_DSS_BITS); // Fast set to 16-bits
    spi_write16_read16_blocking(SPI_IDX, &data, &ret, 1);
    return ret;
}

void transfer_buf(uint8_t *buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
        *buf = transfer(*buf);
        buf++;
    }
}
