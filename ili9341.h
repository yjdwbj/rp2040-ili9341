#ifndef _IL9341_LCD_H
#define _IL9341_LCD_H
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include <math.h>
#include <stdio.h>

#define UART_ID uart0

#ifdef USE_LVGL
#include "lv_conf.h"
#include "lvgl/lvgl.h"
#include "xpt2046.h"
#endif

#if USE_PIO == 1
#include "hardware/pio.h"
#include "hardware/pio_instructions.h"
#include "ili9341_lcd.pio.h"
#endif

#if USE_BIT_BANGING == 1
#define LSBFIRST 0
#define MSBFIRST 1

#endif

// ili9341  common registers
#define CASET   0x2A
#define PASET   0x2B
#define RAMWR   0x2C

#define IMAGE_SIZE      256
#define LOG_IMAGE_SIZE  8

typedef enum {
    NORMAL = 0,
    Y_MIRROR = (1 << 7),
    X_MIRROR = (1 << 6),
    XY_CHANGE = (1 << 5),
    XY_MIRROR = (Y_MIRROR | X_MIRROR),
    XY_CH_YM = (XY_CHANGE | Y_MIRROR),
    XY_CH_XM = (XY_CHANGE | X_MIRROR),
    XY_CH_XYM = (XY_CH_YM | XY_CH_XM)
} display_direct;

void ili9341_init(void);
void ili9341_draw_rgb565_data(uint16_t *data, int len);
void shiftout(uint16_t val, uint8_t bits);
void ili9341_openwindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void ili9341_set_rotation(display_direct value);
void ili9341_draw_test();
#ifdef USE_LVGL
void init_lvgl();
#endif
#endif // _IL9341_LCD_H
