/*
 * @Author: Liu Chun Yang
 * @Date: 2023-12-14 00:11:15
 * @Last Modified by: Liu Chun Yang
 * @Last Modified time: 2023-12-14 00:39:13
 */
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "hardware/uart.h"

#include "pico/stdlib.h"

#include "raspberry_256x256_rgb565.h"

#if USE_BIT_BANGING == 0
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "ili9341_lcd.pio.h"
#else
#define LSBFIRST            0
#define MSBFIRST            1
#endif

// ili9341  common registers
#define CASET               0x2A
#define PASET               0x2B
#define RAMWR               0x2C

#define SCREEN_WIDTH        240
#define SCREEN_HEIGHT       320


#define IMAGE_SIZE          256
#define LOG_IMAGE_SIZE      8

#define PIN_DIN             0     // For 74hc595n connect to DS,      For SPI TFT LCD connect to SDI(MOSI)
#define PIN_CLK             1     // For 74hc595n connect to SHCP,    For SPI TFT LCD connect to SCK
#define PIN_RS              2     // For Parallel LCD connect to RS,  For SPI TFT LCD connect to DC
#define PIN_WR              3     // For Parallel LCD connect to WR
#define PIN_RESET           4     // For Parallel LCD connect to RES, For SPI TFT LCD connect to RES
#define PIN_LATCH           5     // For 74hc595n connect to STCP

#define SERIAL_CLK_DIV      1.f

#ifndef M_PI
#define M_PI                3.14159265358979323846
#endif

#define PLL_SYS_KHZ         (133 * 1000)

// Format: cmd length (including cmd byte), post delay in units of 5 ms, then cmd payload
// Note the delays have been shortened a little

static const uint8_t ili9341_init_seq[] = {
    1, 20, 0x01,                                                      // Software reset
    1, 10, 0x11,                                                      // Exit sleep mode
    4, 0, 0xCF, 0x00, 0x83, 0x30,                                     // Power control B
    4, 0, 0xE8, 0x85, 0x01, 0x79,                                     // Driver timing control A
    2, 2, 0xF7, 0x20,                                                 // Pump ratio control, DDVDH=2xVCl
    3, 0, 0xEA, 0x00, 0x00,                                           // Driver timing control B
    2, 2, 0x3A, 0x55,                                                 // Set colour mode to 16 bit
    2, 0, 0x36, 0x08,                                                 // Set MADCTL: row then column, refresh is bottom to top ????
    5, 0, 0x2A, 0x00, 0x00, SCREEN_WIDTH >> 8, SCREEN_WIDTH & 0xFF,   // CASET: column addresses
    5, 0, 0x2B, 0x00, 0x00, SCREEN_HEIGHT >> 8, SCREEN_HEIGHT & 0xFF, // RASET: row addresses
    3, 2, 0xB1, 0x00, 0x10,                                           // Frame Rate control 119Hz
    2, 2, 0xF2, 0x08,
    2, 0, 0x26, 0x01,
    16, 0, 0xE0, 0x1F, 0x36, 0x36, 0x3A, 0x0C, 0x05, 0x4F, 0X87, 0x3C, 0x08, 0x11, 0x35, 0x19, 0x13, 0x00,
    16, 0, 0xE1, 0x00, 0x09, 0x09, 0x05, 0x13, 0x0A, 0x30, 0x78, 0x43, 0x07, 0x0E, 0x0A, 0x26, 0x2C, 0x1F,
    1, 0, 0x13,                                                        // Normal display on, then 10 ms delay
    1, 0, 0x29,                                                        // Main screen turn on, then wait 500 ms
    0                                                                  // Terminate list
};

#define UART_ID uart1
#define PICO_DEFAULT_UART_BAUD_RATE     115200
// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#undef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN        8

#undef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN        9

#if USE_BIT_BANGING == 0
PIO tft_pio;
int8_t pio_sm = 0; // pioinit will claim a free one
pio_sm_config sm_conf;

// Updated later with the loading offset of the PIO program.
uint32_t program_offset = 0;

void pioinit(uint32_t clock_freq) {

    // Find enough free space on one of the PIO's
    tft_pio = pio0;
    // Load the PIO program
    program_offset = pio_add_program(tft_pio, &ili9341_lcd_program);
    // Configure the state machine
    sm_conf = ili9341_lcd_program_get_default_config(program_offset);
    ili9341_lcd_program_init(tft_pio, pio_sm, program_offset, PIN_DIN, PIN_CLK, clock_freq);
}
#endif

static inline void shiftout( uint16_t val,uint8_t bits) {

#if USE_BIT_BANGING == 0
    gpio_put(PIN_LATCH, 0);
    if(bits == 8)
    {
        ili9341_lcd_wait_idle(tft_pio,pio_sm);
        ili9341_lcd_put(tft_pio,pio_sm,val);
    } else {
        // NOTE: The 3.2-inch 16-bit parallel LCD cannot write rgb565 pixels to LCD memory twice using 8 bits.

        // A little trick is used here, that is, the shift-out of the state machine configuration must be 8,
        // if it is 16, then sending the LCD control command will fail,
        // So it has to be written twice, so that the 16-bit data fills both 74hc595 registers,
        // and then it is written to the LCD memory at once.
        ili9341_lcd_put(tft_pio,pio_sm,val >>8);

        ili9341_lcd_wait_idle(tft_pio, pio_sm);
        ili9341_lcd_put(tft_pio, pio_sm, val);
    }
    ili9341_lcd_wait_idle(tft_pio, pio_sm);
    gpio_put(PIN_WR, 0);
    /* When the latch pin is enabled, the contents of the shift register are copied to the storage/latch register. */
    gpio_put(PIN_LATCH, 1);
    /* Just for delay, __asm volatile ("nop\n") not work, but use sleep_us(1) it too long. */
    gpio_put(PIN_LATCH, 1);
    // The host processor provides information during the write cycle
    // when the display module captures the information from host processor on the rising edge of WR.
    gpio_put(PIN_WR, 1);
#else
    uint8_t bitOrder = MSBFIRST;
    uint8_t i;
    uint8_t max_len = bits - 1;
    gpio_put(PIN_LATCH, 0);
    for (i = 0; i < bits; i++) {
        if (bitOrder == MSBFIRST)
        {
            gpio_put(PIN_DIN, (val & (1 << (max_len - i))) >> (max_len - i));
        } else {
            gpio_put(PIN_DIN, (val & (1 << i)) >> i);
        }
        gpio_put(PIN_CLK, 1);
        gpio_put(PIN_CLK, 0);
    }
    gpio_put(PIN_WR, 0);
    gpio_put(PIN_LATCH, 1);
    /* Just for delay, __asm volatile ("nop\n") not work, if use sleep_us(1) it too long. */
    gpio_put(PIN_LATCH, 1);
    gpio_put(PIN_LATCH, 1);
    gpio_put(PIN_WR, 1);
#endif
}

static inline void lcd_send_cmd(const uint8_t cmd) {
    gpio_put(PIN_RS, 0);
    shiftout(cmd, 8);
    gpio_put(PIN_RS, 1);
}

static inline void lcd_send_data(const uint8_t data) {
    gpio_put(PIN_RS, 1);
    shiftout(data, 8);
}

static inline void lcd_write_cmd(const uint8_t *cmd, size_t count) {
    lcd_send_cmd(*cmd++);
    if (count >= 2) {
        for (size_t i = 0; i < count - 1; ++i)
        {
            lcd_send_data(*cmd++);
        }
    }
}

static inline void lcd_init(const uint8_t *init_seq) {
    const uint8_t *cmd = init_seq;
    while (*cmd) {
        lcd_write_cmd( cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

static void ili9341_openwindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    lcd_send_cmd(CASET);
    lcd_send_data(x1 >> 8);
    lcd_send_data(x1 & 0xFF);
    lcd_send_data((x1 + x2 - 1) >> 8);
    lcd_send_data((x1 + x2 - 1) & 0xFF);

    lcd_send_cmd(PASET);
    lcd_send_data(y1 >> 8);
    lcd_send_data(y1 & 0xFF);
    lcd_send_data((y1 + y2 - 1) >> 8);
    lcd_send_data((y1 + y2 - 1) & 0xFF);
    sleep_ms(1);
    lcd_send_cmd(RAMWR);
}

static void ili9341_show_rgb565_data(uint16_t *data, int len) {
    ili9341_openwindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    for (int i = 0; i < len; i++) {
        shiftout(*data++, 16);
    }
}

static void init_uart()
{
    uart_init(UART_ID, PICO_DEFAULT_UART_BAUD_RATE);
    if (PICO_DEFAULT_UART_TX_PIN >= 0)
        gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    if (PICO_DEFAULT_UART_RX_PIN >= 0)
        gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);
    // bi_decl_if_func_used(bi_2pins_with_func(PICO_DEFAULT_UART_RX_PIN, PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART));
}


int main() {
    // set_sys_clock_khz(PLL_SYS_KHZ, true);
    stdio_init_all();

    init_uart();
#if USE_BIT_BANGING == 0
    pioinit(72000000);
    uart_puts(UART_ID ,"Use PIO driven TFT LCD\n\r");
#else
    uart_puts(UART_ID, "Use GPIO bit-banging driven TFT LCD\n\r");
    gpio_init(PIN_DIN);
    gpio_init(PIN_CLK);
    gpio_set_dir(PIN_DIN, GPIO_OUT);
    gpio_set_dir(PIN_CLK, GPIO_OUT);
#endif
    gpio_init(PIN_LATCH);
    gpio_set_dir(PIN_LATCH, GPIO_OUT);

    gpio_init(PIN_WR);
    gpio_set_dir(PIN_WR, GPIO_OUT);

    gpio_init(PIN_RS);
    gpio_init(PIN_RESET);

    gpio_set_dir(PIN_RS, GPIO_OUT);
    gpio_set_dir(PIN_RESET, GPIO_OUT);

    gpio_put(PIN_RS, 1);
    // gpio_put(PIN_WR, 1);
    gpio_put(PIN_RESET, 1);

    lcd_init(ili9341_init_seq);

    // Other SDKs: static image on screen, lame, boring
    // Raspberry Pi Pico SDK: spinning image on screen, bold, exciting

    // Lane 0 will be u coords (bits 8:1 of addr offset), lane 1 will be v
    // coords (bits 16:9 of addr offset), and we'll represent coords with
    // 16.16 fixed point. ACCUM0,1 will contain current coord, BASE0/1 will
    // contain increment vector, and BASE2 will contain image base pointer

#if 0
#include "ffmpeg_bgr565_240x320x16.h"
    /*  show static bgr565 image  */
    ili9341_show_rgb565_data(ffmpeg_bgr565_240x320x16, SCREEN_WIDTH * SCREEN_HEIGHT);

#else
#define UNIT_LSB 16
    interp_config lane0_cfg = interp_default_config();
    interp_config_set_shift(&lane0_cfg, UNIT_LSB - 1); // -1 because 2 bytes per pixel
    interp_config_set_mask(&lane0_cfg, 1, 1 + (LOG_IMAGE_SIZE - 1));
    interp_config_set_add_raw(&lane0_cfg, true); // Add full accumulator to base with each POP
    interp_config lane1_cfg = interp_default_config();
    interp_config_set_shift(&lane1_cfg, UNIT_LSB - (1 + LOG_IMAGE_SIZE));
    interp_config_set_mask(&lane1_cfg, 1 + LOG_IMAGE_SIZE, 1 + (2 * LOG_IMAGE_SIZE - 1));
    interp_config_set_add_raw(&lane1_cfg, true);

    interp_set_config(interp0, 0, &lane0_cfg);
    interp_set_config(interp0, 1, &lane1_cfg);
    interp0->base[2] = (uint32_t) raspberry_256x256;

    float theta = 0.f;
    float theta_max = 2.f * (float) M_PI;
    ili9341_openwindow(0, 0, SCREEN_WIDTH , SCREEN_HEIGHT);
    while (1) {
        theta += 0.02f;
        if (theta > theta_max)
            theta -= theta_max;
        int32_t rotate[4] = {
                (int32_t) (cosf(theta) * (1 << UNIT_LSB)), (int32_t) (-sinf(theta) * (1 << UNIT_LSB)),
                (int32_t) (sinf(theta) * (1 << UNIT_LSB)), (int32_t) (cosf(theta) * (1 << UNIT_LSB))
        };
        interp0->base[0] = rotate[0];
        interp0->base[1] = rotate[2];
        for (int y = 0; y < SCREEN_HEIGHT; ++y) {
            interp0->accum[0] = rotate[1] * y;
            interp0->accum[1] = rotate[3] * y;
            for (int x = 0; x < SCREEN_WIDTH; ++x) {
                uint16_t colour = *(uint16_t *) (interp0->pop[2]);
                // shiftout(colour >> 8,16);
                shiftout(colour, 16);
            }
        }
    }
#endif
}
