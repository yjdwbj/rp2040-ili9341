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

#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define PLL_SYS_KHZ (133 * 1000)
#define UART_ID uart0
#define PICO_DEFAULT_UART_BAUD_RATE 115200
// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#undef PICO_DEFAULT_UART_TX_PIN
#define PICO_DEFAULT_UART_TX_PIN 16

#undef PICO_DEFAULT_UART_RX_PIN
#define PICO_DEFAULT_UART_RX_PIN 17

#include "ili9341.h"

static void init_uart()
{
    uart_init(UART_ID, PICO_DEFAULT_UART_BAUD_RATE);
    if (PICO_DEFAULT_UART_TX_PIN >= 0)
        gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
    if (PICO_DEFAULT_UART_RX_PIN >= 0)
        gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);
    bi_decl_if_func_used(bi_2pins_with_func(PICO_DEFAULT_UART_RX_PIN, PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART));
}

int main() {
    // set_sys_clock_khz(PLL_SYS_KHZ, true);
    stdio_init_all();
    init_uart();
    sleep_ms(1);
    printf("Starting\n");
#if USE_BIT_BANGING == 1
    uart_puts(UART_ID, "Runing on GPIO Bit-banging mode\n");
#elif USE_PIO == 1
    uart_puts(UART_ID, "Runing on PIO mode\n");
#else
    uart_puts(UART_ID, "Runing on SPI mode\n");
#endif
    ili9341_init();

#ifdef USE_LVGL
    init_lvgl();
    while (true) {
        lv_tick_inc(1);
        lv_task_handler();
        // lv_timer_handler(); /* let the GUI do its work */
        sleep_ms(10);
    }
#else

#if 0
#include "ffmpeg_bgr565_240x320x16.h"
        /*  show static bgr565 image  */
    ili9341_set_rotation(NORMAL);
    ili9341_draw_rgb565_data(ffmpeg_bgr565_240x320x16, sizeof(ffmpeg_bgr565_240x320x16) / sizeof(char) / 2);
#else
    ili9341_draw_test();
#endif
#endif

}
