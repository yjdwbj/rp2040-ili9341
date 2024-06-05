#include "xpt2046.h"
#include "hardware/gpio.h"
#include "hardware/interp.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "user_config.h"

static const uint16_t screenWidth = SCREEN_WIDTH;
static const uint16_t screenHeight = SCREEN_HEIGHT;

static const uint16_t XPT2046_ADC_LIMIT = 4095;

// xpt2046 registers
//                                  START  ADDR  SER/  INT   VREF    ADC
//                                               DFR   ENA   INT/EXT ENA
#define Z_VALUE_1       0xB0      // 1      011   0     0     0       1
#define Z_VALUE_2       0xC0      // 1      100   0     0     0       1
#define Y_POSITION      0x90      // 1      001   0     0     0       1
#define X_POSITION      0xD0      // 1      101   0     0     0       1


#define Z_THRESHOLD     350


uint32_t _pressTime;              // Press and hold time-out
uint16_t _pressX, _pressY;        // For future use (last sampled calibrated coordinates)

// The following values came from the TFT_eSPI::calibrateTouch of Keypad_240x320.ino project tested results.
uint16_t touchCalibration_x0 = 289;
uint16_t touchCalibration_x1 = 3415;
uint16_t touchCalibration_y0 = 371;
uint16_t touchCalibration_y1 = 3490;
uint8_t touchCalibration_rotate = 0;
uint8_t touchCalibration_invert_x = 1;
uint8_t touchCalibration_invert_y = 0;

// The following touch screen support code from the TFT_eSPI https://github.com/Bodmer/TFT_eSPI.

static void convertRawXY(uint16_t *x, uint16_t *y);

/***************************************************************************************
** Function name:           begin_touch_read_write - was spi_begin_touch
** Description:             Start transaction and select touch controller
***************************************************************************************/
// The touch controller has a low SPI clock rate
inline void begin_touch_read_write(void) {
    gpio_put(TFT_CS, 1); // Just in case it has been left low
    spi_set_baudrate(SPI_IDX,SPI_TOUCH_FREQUENCY);
    gpio_put(XPT_CS, 0);
}

/***************************************************************************************
** Function name:           end_touch_read_write - was spi_end_touch
** Description:             End transaction and deselect touch controller
***************************************************************************************/
inline void end_touch_read_write(void) {
    gpio_put(XPT_CS, 1);
    spi_set_baudrate(SPI_IDX, SPI_ILI9341_FREQ);
}
/***************************************************************************************
** Function name:           getTouchRaw
** Description:             read raw touch position.  Always returns true.
***************************************************************************************/
static uint8_t getTouchRaw(uint16_t *x, uint16_t *y) {
    uint16_t tmp;

    begin_touch_read_write();

    // Start YP sample request for x position, read 4 times and keep last sample
    transfer(X_POSITION); // Start new YP conversion
    transfer(0);    // Read first 8 bits
    transfer(X_POSITION); // Read last 8 bits and start new YP conversion
    transfer(0);    // Read first 8 bits
    transfer(X_POSITION); // Read last 8 bits and start new YP conversion
    transfer(0);    // Read first 8 bits
    transfer(X_POSITION); // Read last 8 bits and start new YP conversion

    tmp = transfer(0); // Read first 8 bits
    tmp = tmp << 5;
    tmp |= 0x1f & (transfer(Y_POSITION) >> 3); // Read last 8 bits and start new XP conversion

    *x = tmp;

    // Start XP sample request for y position, read 4 times and keep last sample
    transfer(0);    // Read first 8 bits
    transfer(Y_POSITION); // Read last 8 bits and start new XP conversion
    transfer(0);    // Read first 8 bits
    transfer(Y_POSITION); // Read last 8 bits and start new XP conversion
    transfer(0);    // Read first 8 bits
    transfer(Y_POSITION); // Read last 8 bits and start new XP conversion

    tmp = transfer(0); // Read first 8 bits
    tmp = tmp << 5;
    tmp |= 0x1f & (transfer(0) >> 3); // Read last 8 bits

    *y = tmp;

    end_touch_read_write();

    return true;
}

/***************************************************************************************
** Function name:           getTouchRawZ
** Description:             read raw pressure on touchpad and return Z value.
***************************************************************************************/
static uint16_t getTouchRawZ(void) {

    begin_touch_read_write();

    // Z sample request
    int16_t tz = 0xFFF;
    transfer(Z_VALUE_1);              // Start new Z1 conversion
    tz += transfer16(Z_VALUE_2) >> 3; // Read Z1 and start Z2 conversion
    tz -= transfer16(0x00) >> 3; // Read Z2

    end_touch_read_write();

    if (tz == XPT2046_ADC_LIMIT)
        tz = 0;

    return (uint16_t)tz;
}

/***************************************************************************************
** Function name:           validTouch
** Description:             read validated position. Return false if not pressed.
***************************************************************************************/
#define _RAWERR 20 // Deadband error allowed in successive position samples
static uint8_t validTouch(uint16_t *x, uint16_t *y, uint16_t threshold) {
    uint16_t x_tmp, y_tmp, x_tmp2, y_tmp2;

    // Wait until pressure stops increasing to debounce pressure
    uint16_t z1 = 1;
    uint16_t z2 = 0;
    while (z1 > z2) {
        z2 = z1;
        z1 = getTouchRawZ();
        sleep_ms(1);
    }

    if (z1 <= threshold)
        return false;

    getTouchRaw(&x_tmp, &y_tmp);

    sleep_ms(1); // Small delay to the next sample
    if (getTouchRawZ() <= threshold)
        return false;

    sleep_ms(2); // Small delay to the next sample
    getTouchRaw(&x_tmp2, &y_tmp2);

    if (abs(x_tmp - x_tmp2) > _RAWERR)
        return false;
    if (abs(y_tmp - y_tmp2) > _RAWERR)
        return false;

    *x = x_tmp;
    *y = y_tmp;

    return true;
}

/***************************************************************************************
** Function name:           getTouch
** Description:             read callibrated position. Return false if not pressed.
***************************************************************************************/
static uint8_t getTouch(uint16_t *x, uint16_t *y) {
    uint16_t threshold = 350;
    uint16_t x_tmp,
        y_tmp;

    if (threshold < 20)
        threshold = 20;
    if (_pressTime > to_ms_since_boot(get_absolute_time()))
        threshold = 20;

    uint8_t n = 5;
    uint8_t valid = 0;
    while (n--) {
        if (validTouch(&x_tmp, &y_tmp, threshold))
            valid++;
    }

    if (valid < 1) {
        _pressTime = 0;
        return false;
    }

    _pressTime = to_ms_since_boot(get_absolute_time()) + 50;

    convertRawXY(&x_tmp, &y_tmp);

    if (x_tmp >= screenWidth || y_tmp >= screenHeight)
        return false;

    _pressX = x_tmp;
    _pressY = y_tmp;
    *x = _pressX;
    *y = _pressY;
    return valid;
}

/***************************************************************************************
** Function name:           convertRawXY
** Description:             convert raw touch x,y values to screen coordinates
***************************************************************************************/
static void convertRawXY(uint16_t *x, uint16_t *y) {
    uint16_t x_tmp = *x, y_tmp = *y, xx, yy;

    if (!touchCalibration_rotate) {
        xx = (x_tmp - touchCalibration_x0) * screenWidth / touchCalibration_x1;
        yy = (y_tmp - touchCalibration_y0) * screenHeight / touchCalibration_y1;
        if (touchCalibration_invert_x)
            xx = screenWidth - xx;
        if (touchCalibration_invert_y)
            yy = screenHeight - yy;
    } else {
        xx = (y_tmp - touchCalibration_x0) * screenWidth / touchCalibration_x1;
        yy = (x_tmp - touchCalibration_y0) * screenHeight / touchCalibration_y1;
        if (touchCalibration_invert_x)
            xx = screenWidth - xx;
        if (touchCalibration_invert_y)
            yy = screenHeight - yy;
    }
    *x = xx;
    *y = yy;
}

#if 0
/***************************************************************************************
** Function name:           calibrateTouch
** Description:             generates calibration parameters for touchscreen.
***************************************************************************************/
void calibrateTouch(uint16_t *parameters, uint32_t color_fg, uint32_t color_bg, uint8_t size) {
    int16_t values[] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t x_tmp, y_tmp;

    for (uint8_t i = 0; i < 4; i++) {
        fillRect(0, 0, size + 1, size + 1, color_bg);
        fillRect(0, screenHeight - size - 1, size + 1, size + 1, color_bg);
        fillRect(screenWidth - size - 1, 0, size + 1, size + 1, color_bg);
        fillRect(screenWidth - size - 1, screenHeight - size - 1, size + 1, size + 1, color_bg);

        if (i == 5)
            break; // used to clear the arrows

        switch (i) {
        case 0: // up left
            drawLine(0, 0, 0, size, color_fg);
            drawLine(0, 0, size, 0, color_fg);
            drawLine(0, 0, size, size, color_fg);
            break;
        case 1: // bot left
            drawLine(0, screenHeight - size - 1, 0, screenHeight - 1, color_fg);
            drawLine(0, screenHeight - 1, size, screenHeight - 1, color_fg);
            drawLine(size, screenHeight - size - 1, 0, screenHeight - 1, color_fg);
            break;
        case 2: // up right
            drawLine(screenWidth - size - 1, 0, screenWidth - 1, 0, color_fg);
            drawLine(screenWidth - size - 1, size, screenWidth - 1, 0, color_fg);
            drawLine(screenWidth - 1, size, screenWidth - 1, 0, color_fg);
            break;
        case 3: // bot right
            drawLine(screenWidth - size - 1, screenHeight - size - 1, screenWidth - 1, screenHeight - 1, color_fg);
            drawLine(screenWidth - 1, screenHeight - 1 - size, screenWidth - 1, screenHeight - 1, color_fg);
            drawLine(screenWidth - 1 - size, screenHeight - 1, screenWidth - 1, screenHeight - 1, color_fg);
            break;
        }

        // user has to get the chance to release
        if (i > 0)
            sleep_ms(1000);

        for (uint8_t j = 0; j < 8; j++) {
            // Use a lower detect threshold as corners tend to be less sensitive
            while (!validTouch(&x_tmp, &y_tmp, Z_THRESHOLD / 2))
                ;
            values[i * 2] += x_tmp;
            values[i * 2 + 1] += y_tmp;
        }
        values[i * 2] /= 8;
        values[i * 2 + 1] /= 8;
    }

    // from case 0 to case 1, the y value changed.
    // If the measured delta of the touch x axis is bigger than the delta of the y axis, the touch and TFT axes are switched.
    touchCalibration_rotate = false;
    if (abs(values[0] - values[2]) > abs(values[1] - values[3])) {
        touchCalibration_rotate = true;
        touchCalibration_x0 = (values[1] + values[3]) / 2; // calc min x
        touchCalibration_x1 = (values[5] + values[7]) / 2; // calc max x
        touchCalibration_y0 = (values[0] + values[4]) / 2; // calc min y
        touchCalibration_y1 = (values[2] + values[6]) / 2; // calc max y
    } else {
        touchCalibration_x0 = (values[0] + values[2]) / 2; // calc min x
        touchCalibration_x1 = (values[4] + values[6]) / 2; // calc max x
        touchCalibration_y0 = (values[1] + values[5]) / 2; // calc min y
        touchCalibration_y1 = (values[3] + values[7]) / 2; // calc max y
    }

    // in addition, the touch screen axis could be in the opposite direction of the TFT axis
    touchCalibration_invert_x = false;
    if (touchCalibration_x0 > touchCalibration_x1) {
        values[0] = touchCalibration_x0;
        touchCalibration_x0 = touchCalibration_x1;
        touchCalibration_x1 = values[0];
        touchCalibration_invert_x = true;
    }
    touchCalibration_invert_y = false;
    if (touchCalibration_y0 > touchCalibration_y1) {
        values[0] = touchCalibration_y0;
        touchCalibration_y0 = touchCalibration_y1;
        touchCalibration_y1 = values[0];
        touchCalibration_invert_y = true;
    }

    // pre calculate
    touchCalibration_x1 -= touchCalibration_x0;
    touchCalibration_y1 -= touchCalibration_y0;

    if (touchCalibration_x0 == 0)
        touchCalibration_x0 = 1;
    if (touchCalibration_x1 == 0)
        touchCalibration_x1 = 1;
    if (touchCalibration_y0 == 0)
        touchCalibration_y0 = 1;
    if (touchCalibration_y1 == 0)
        touchCalibration_y1 = 1;

    // export parameters, if pointer valid
    if (parameters != NULL) {
        parameters[0] = touchCalibration_x0;
        parameters[1] = touchCalibration_x1;
        parameters[2] = touchCalibration_y0;
        parameters[3] = touchCalibration_y1;
        parameters[4] = touchCalibration_rotate | (touchCalibration_invert_x << 1) | (touchCalibration_invert_y << 2);
    }
}
#endif

static void driver_init(void) {
    gpio_init(XPT_CS);
    gpio_put(XPT_CS, 1);
    gpio_set_dir(XPT_CS, GPIO_OUT);
}

void XPT2046_init(){
    driver_init();
}

/**
 * Get the current position and state of the touchpad
 * @param data store the read data here
 * @return false: because no ore data to be read
 */
#if LVGL_VERSION_MAJOR >= 9
void XPT2046_read(lv_indev_data_t *drv, lv_indev_data_t *data) {
#else
void XPT2046_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
#endif
    if (getTouch(&data->point.x, &data->point.y)) {
        data->state = LV_INDEV_STATE_PR;
        uart_puts(UART_ID, "press ed \r\n");
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}