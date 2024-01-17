#include "ili9341.h"
#include "hardware/interp.h"
#include "hardware/spi.h"
#include "lvgl/examples/widgets/lv_example_widgets.h"
#include "pico/binary_info.h"
#include "raspberry_256x256_rgb565.h"

#include "user_config.h"
#if USE_BIT_BANGING == 1
uint8_t bitOrder = MSBFIRST;
#endif

// for 74hc165d
// #define USE_74HC165
#if defined(USE_74HC165)
#define TFT_DIN     12 // read from 74hc165 -> LCD
#define TFT_RD      13
#define TFT_CP      14 // clock
#define TFT_PL      15 // active low load

#define READ_ID1 0xDA
#define READ_ID2 0xDB
#define READ_ID3 0xDC
#define READ_ID4 0xD3
#endif

#define SERIAL_CLK_DIV 1.f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if !defined(TFT_CS) || TFT_CS == -1
#define CS_L // No macro allocated so it generates no code
#define CS_H // No macro allocated so it generates no code
#else
#define CS_L gpio_put(TFT_CS, 0)
#define CS_H gpio_put(TFT_CS, 1)
#endif

#ifdef USE_LVGL
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_WIDTH * SCREEN_HEIGHT / 10]; /*Declare a buffer for 1/10 screen size*/
static lv_disp_drv_t disp_drv;                             /*Descriptor of a display driver*/
static lv_indev_drv_t indev_drv;                           /*Descriptor of a input device driver*/
#endif
static inline void lcd_send_cmd(const uint8_t cmd);
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
    2, 0, 0x36, 0x08 | Y_MIRROR,                                      // Set MADCTL: row then column, refresh is bottom to top ????
    5, 0, 0x2A, 0x00, 0x00, SCREEN_WIDTH >> 8, SCREEN_WIDTH & 0xFF,   // CASET: column addresses
    5, 0, 0x2B, 0x00, 0x00, SCREEN_HEIGHT >> 8, SCREEN_HEIGHT & 0xFF, // RASET: row addresses
    3, 2, 0xB1, 0x00, 0x10,                                           // Frame Rate control 119Hz
    2, 2, 0xF2, 0x08,
    2, 0, 0x26, 0x01,
    16, 0, 0xE0, 0x1F, 0x36, 0x36, 0x3A, 0x0C, 0x05, 0x4F, 0X87, 0x3C, 0x08, 0x11, 0x35, 0x19, 0x13, 0x00,
    16, 0, 0xE1, 0x00, 0x09, 0x09, 0x05, 0x13, 0x0A, 0x30, 0x78, 0x43, 0x07, 0x0E, 0x0A, 0x26, 0x2C, 0x1F,
    1, 0, 0x13, // Normal display on, then 10 ms delay
    1, 0, 0x29, // Main screen turn on, then wait 500 ms
    0           // Terminate list
};

#if USE_PIO == 1
PIO tft_pio;
int8_t pio_sm = 0; // pioinit will claim a free one
pio_sm_config sm_conf;

// Updated later with the loading offset of the PIO program.
uint32_t program_offset = 0;

static void pioinit(uint32_t clock_freq) {

    // Find enough free space on one of the PIO's
    tft_pio = pio0;
    // Load the PIO program
    program_offset = pio_add_program(tft_pio, &ili9341_lcd_program);
    // Configure the state machine
    sm_conf = ili9341_lcd_program_get_default_config(program_offset);
    ili9341_lcd_program_init(tft_pio, pio_sm, program_offset, TFT_DOUT, TFT_CLK, clock_freq);
}
#endif

#if USE_SPI == 1
static inline void init_spi() {
    // https://docs.arduino.cc/learn/communication/spi/
    spi_init(spi0, SPI_ILI9341_FREQ);
    gpio_set_function(TFT_CLK, GPIO_FUNC_SPI);
    gpio_set_function(TFT_DOUT, GPIO_FUNC_SPI);
    gpio_set_function(TFT_DIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    // bi_decl(bi_3pins_with_func(TFT_DOUT, TFT_DIN, TFT_CLK, GPIO_FUNC_SPI));

#if defined(TFT_CS) && TFT_CS > 0
    gpio_init(TFT_CS);
    gpio_put(TFT_CS, 1);
    gpio_set_dir(TFT_CS, GPIO_OUT);
#endif
    // bi_decl(bi_1pin_with_name(TFT_CS, "SPI CS0"));
}
#endif


void shiftout(uint16_t val, uint8_t bits) {
#ifdef USE_SPI
    transfer16(val);
#endif

#if USE_BIT_BANGING == 1
    uint8_t i;
    uint8_t max_len = bits - 1;
    gpio_put(TFT_LATCH, 0);
    for (i = 0; i < bits; i++) {
        if (bitOrder == MSBFIRST) {
            gpio_put(TFT_DOUT, (val & (1 << (max_len - i))) >> (max_len - i));
        } else {
            gpio_put(TFT_DOUT, (val & (1 << i)) >> i);
        }
        gpio_put(TFT_CLK, 1);
        gpio_put(TFT_CLK, 0);
    }
    gpio_put(TFT_LATCH, 1);
#if defined(USE_74HC165)
    gpio_put(TFT_LATCH, 1); // Failure to execute this command to delay will cause the screen to go black.
#endif
    gpio_put(TFT_WR, 0);
    gpio_put(TFT_WR, 1);
#endif // endif USE_BIT_BANGING

#if USE_PIO == 1
    gpio_put(TFT_LATCH, 0);
    if (bits == 8) {
        ili9341_lcd_wait_idle(tft_pio, pio_sm);
        ili9341_lcd_put(tft_pio, pio_sm, val);
    } else {
        // NOTE: The 3.2-inch 16-bit parallel LCD cannot write rgb565 pixels to LCD memory twice using 8 bits.

        // A little trick is used here, that is, the shift-out of the state machine configuration must be 8,
        // if it is 16, then sending the LCD control command will fail,
        // So it has to be written twice, so that the 16-bit data fills both 74hc595 registers,
        // and then it is written to the LCD memory at once.
        ili9341_lcd_wait_idle(tft_pio, pio_sm);
        ili9341_lcd_put(tft_pio, pio_sm, val >> 8);

        ili9341_lcd_wait_idle(tft_pio, pio_sm);
        ili9341_lcd_put(tft_pio, pio_sm, val);
    }
    ili9341_lcd_wait_idle(tft_pio, pio_sm);

    /* When the latch pin is enabled, the contents of the shift register are copied to the storage/latch register. */
    gpio_put(TFT_LATCH, 1);
    // The host processor provides information during the write cycle
    // when the display module captures the information from host processor on the rising edge of WR.
    gpio_put(TFT_WR, 0);
    gpio_put(TFT_WR, 1);
#endif
}

#ifdef USE_LVGL

static void my_disp_flush(lv_disp_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    int32_t x, y;
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    ili9341_openwindow(area->x1, area->y1, w, h);
    uint16_t color;
    CS_L;
    for (y = area->y1; y <= area->y2; y++) {
        for (x = area->x1; x <= area->x2; x++) {
            shiftout(lv_color_to16(*color_p++), 16);
        }
    }
    CS_H;
    lv_disp_flush_ready(disp); /* Indicate you are ready with the flushing*/
}

void init_lvgl(void) {
    lv_indev_t *mouse_indev;
    lv_obj_t *cursor_obj;

    XPT2046_init();
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL,
                          SCREEN_WIDTH * SCREEN_HEIGHT / 10); /*Initialize the display buffer.*/
    lv_disp_drv_init(&disp_drv);                              /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush;                        /*Set your driver function*/
    disp_drv.draw_buf = &draw_buf;                            /*Assign the buffer to the display*/
    disp_drv.hor_res = SCREEN_WIDTH;                          /*Set the horizontal resolution of the display*/
    disp_drv.ver_res = SCREEN_HEIGHT;                         /*Set the vertical resolution of the display*/
    lv_disp_drv_register(&disp_drv);                          /*Finally register the driver*/

    // init input device
    lv_indev_drv_init(&indev_drv);          /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER; /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = XPT2046_read;       /*Set your driver function*/
    lv_indev_drv_register(&indev_drv);

    mouse_indev = lv_indev_drv_register(&indev_drv);
#if LV_BUILD_EXAMPLES == 1
    // lv_example_calendar_1();
    // lv_example_slider_1();
    // lv_example_slider_2();
    // lv_example_colorwheel_1();
    // lv_example_keyboard_1();
    // lv_example_img_2();
    lv_example_meter_2();
#else
    cursor_obj = lv_img_create(lv_scr_act());     /*Create an image object for the cursor */
    lv_img_set_src(cursor_obj, LV_SYMBOL_GPS);    /*Set the image source*/
    lv_indev_set_cursor(mouse_indev, cursor_obj); /*Connect the image  object to the driver*/
#endif
}
#endif

static inline void lcd_send_cmd(const uint8_t cmd) {
    gpio_put(TFT_RS, 0);
#if USE_SPI == 1
    transfer(cmd);
#else
    shiftout(cmd, 8);
#endif
    gpio_put(TFT_RS, 1);
}

static inline void lcd_send_data(const uint8_t data) {
    gpio_put(TFT_RS, 1);
#if USE_SPI == 1
    transfer(data);
#else
    shiftout(data, 8);
#endif
}

#if USE_BIT_BANGING == 1 && defined(USE_74HC165)

static inline uint8_t lcd_74hcxxx_test(uint8_t data) {
    uint8_t value = 0;
    uint8_t i;

    gpio_put(TFT_LATCH, 0);
    for (i = 0; i < 8; ++i) {
        if (bitOrder == MSBFIRST) {
            gpio_put(TFT_DOUT, (data & (1 << (7 - i))) >> (7 - i));
        } else {
            gpio_put(TFT_DOUT, (data & (1 << i)) >> i);
        }
        gpio_put(TFT_CLK, 1);
        gpio_put(TFT_CLK, 0);
    }
    gpio_put(TFT_LATCH, 1);
    gpio_put(TFT_PL, 0);
    sleep_us(1); // MCU freq too fast need delay.
    gpio_put(TFT_PL, 1);
    sleep_us(1);
    for (i = 0; i < 8; ++i) {
        gpio_put(TFT_CP, 0);
        if (bitOrder == LSBFIRST)
            value |= gpio_get(TFT_DIN) << i;
        else
            value |= gpio_get(TFT_DIN) << (7 - i);
        gpio_put(TFT_CP, 1);
        sleep_us(1);
    }
    static char text[64];
    sprintf(text, "TEST Read 74hcxxx , 0x%04x\r\n", value);
    uart_puts(UART_ID, text);
    return value;
}

static inline uint8_t shiftin() {
    uint8_t value = 0;
    uint8_t i;
    gpio_put(TFT_RD, 0);
    gpio_put(TFT_PL, 0);
    sleep_us(1);
    gpio_put(TFT_PL, 1);
    gpio_put(TFT_RD, 1);
    for (i = 0; i < 8; ++i) {
        gpio_put(TFT_CP, 0);
        if (bitOrder == LSBFIRST)
            value |= gpio_get(TFT_DIN) << i;
        else
            value |= gpio_get(TFT_DIN) << (7 - i);
        gpio_put(TFT_CP, 1);
        sleep_us(1);
    }
    return value;
}

static inline uint32_t lcd_read_device_id() {
    uint32_t id = 0;
    lcd_send_cmd(READ_ID4);
    gpio_put(TFT_WR, 1);
    shiftin();      // dummy data
    shiftin();      // 0x00
    id = shiftin(); // 0x93
    id <<= 8;
    id |= shiftin(); // 0x41
    return id;
}
#endif

static inline void lcd_write_cmd(const uint8_t *cmd, size_t count) {
    CS_L;
    lcd_send_cmd(*cmd++);
    if (count >= 2) {
        for (size_t i = 0; i < count - 1; ++i) {
            lcd_send_data(*cmd++);
        }
    }
    CS_H;
}


static void driver_init() {
#if USE_BIT_BANGING == 1
    gpio_init(TFT_DOUT);
    gpio_init(TFT_CLK);
    gpio_set_dir(TFT_DOUT, GPIO_OUT);
    gpio_set_dir(TFT_CLK, GPIO_OUT);
#elif USE_PIO == 1
    pioinit(72000000);
#elif USE_SPI == 1
    init_spi();
#endif

#if defined(USE_74HC165)
    gpio_init(TFT_RD);
    gpio_set_dir(TFT_RD, GPIO_OUT);
    gpio_put(TFT_RD, 1);
#endif

#if defined(USE_LVGL)
    gpio_init(TFT_CS);
    gpio_set_dir(TFT_CS, GPIO_OUT);
    gpio_put(TFT_CS, 0);
#endif

    gpio_init(TFT_LATCH);
    gpio_set_dir(TFT_LATCH, GPIO_OUT);

    gpio_init(TFT_WR);
    gpio_set_dir(TFT_WR, GPIO_OUT);

    gpio_init(TFT_RS);
    gpio_init(TFT_RESET);

    gpio_set_dir(TFT_RS, GPIO_OUT);
    gpio_set_dir(TFT_RESET, GPIO_OUT);

    gpio_put(TFT_RS, 1);
    gpio_put(TFT_RESET, 1);

#if USE_BIT_BANGING == 1 && defined(USE_74HC165)
    gpio_init(TFT_DIN);
    gpio_init(TFT_CP);
    gpio_init(TFT_PL);

    gpio_set_dir(TFT_CP, GPIO_OUT);

    gpio_set_dir(TFT_PL, GPIO_OUT);
    gpio_set_dir(TFT_DIN, GPIO_IN);

    gpio_put(TFT_PL, 1);

#if 0
    uint8_t inval[] = {0x43, 0x10, 0xa1, 0x49};
    for (int i = 0; i < sizeof(inval) / sizeof(uint8_t); i++)
        lcd_74hcxxx_test(inval[i]);
#else
    lcd_send_cmd(0x1);
    sleep_ms(10);
    uint32_t device_id = lcd_read_device_id();
    static char text[64];
    sprintf(text, "Read TFT LCD device id: 0x%08x\r\n", device_id);
    uart_puts(UART_ID, text);
#endif
#endif
}

void ili9341_init() {
    driver_init();
    uart_puts(UART_ID, "after ili9341 device initial\r\n");
    const uint8_t *cmd = &ili9341_init_seq;
    while (*cmd) {
        lcd_write_cmd(cmd + 2, *cmd);
        sleep_ms(*(cmd + 1) * 5);
        cmd += *cmd + 2;
    }
}

void ili9341_set_rotation(display_direct value) {
    CS_L;
    lcd_send_cmd(0x36);
    lcd_send_data(value | 0x8);
    CS_H;
}

void ili9341_openwindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    CS_L;
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
    CS_H;
}

void ili9341_draw_rgb565_data(uint16_t *data, int len) {
    ili9341_openwindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
    CS_L;
    for (int i = 0; i < len; i++) {
        shiftout(*data++, 16);
    }
    CS_H;
}

void ili9341_draw_test() {
    // Other SDKs: static image on screen, lame, boring
    // Raspberry Pi Pico SDK: spinning image on screen, bold, exciting

    // Lane 0 will be u coords (bits 8:1 of addr offset), lane 1 will be v
    // coords (bits 16:9 of addr offset), and we'll represent coords with
    // 16.16 fixed point. ACCUM0,1 will contain current coord, BASE0/1 will
    // contain increment vector, and BASE2 will contain image base pointer
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
    interp0->base[2] = (uint32_t)raspberry_256x256;

    float theta = 0.f;
    float theta_max = 2.f * (float)M_PI;
    ili9341_openwindow(0, 0, SCREEN_HEIGHT, SCREEN_WIDTH);
    CS_L;
    while (1) {
        theta += 0.02f;
        if (theta > theta_max)
            theta -= theta_max;
        int32_t rotate[4] = {
            (int32_t)(cosf(theta) * (1 << UNIT_LSB)), (int32_t)(-sinf(theta) * (1 << UNIT_LSB)),
            (int32_t)(sinf(theta) * (1 << UNIT_LSB)), (int32_t)(cosf(theta) * (1 << UNIT_LSB))};
        interp0->base[0] = rotate[0];
        interp0->base[1] = rotate[2];
        for (int y = 0; y < SCREEN_WIDTH; ++y) {
            interp0->accum[0] = rotate[1] * y;
            interp0->accum[1] = rotate[3] * y;
            for (int x = 0; x < SCREEN_HEIGHT; ++x) {
                uint16_t color = *(uint16_t *)(interp0->pop[2]);
                shiftout(color, 16);
            }
        }
    }
}