# RP2040-LCD example of Pico-examples  revisited.

## Taken from the Pico-examples, and expanded

Expanded with:
* Two shift register 74HC595 daisy chained together to save many pins, which is main purpose of this project.
* Simultaneously drive two LCDs with different interfaces.
* Also supports GPIO shift of bit-banging method output to drive LCD.

## Supported and tested TFT screens

![ili9341_2.8inch_240x320_spi_lcd.jpg](images/ili9341_2.8inch_240x320_spi_lcd.jpg)
![ili9341_3.2inch_240x320_16b_lcd.jpg](images/ili9341_3.2inch_240x320_16b_lcd.jpg)
![rp2040-74hc595n-lcd.jpg](images/rp2040-74hc595n-lcd.jpg)
![rp2040-with-two-lcd.gif](images/rp2040-with-two-lcd.gif)

## Logic probe timing diagram

* LCD control command
![images/down_speed_lcd_control_cmd.png](images/down_speed_lcd_control_cmd.png)

* Pixels data of BGR565  after `0x2C`.
![images/down_speed_rgb55.png](images/down_speed_rgb55.png)

## Circuit Diagram Table


|    RP2040  | 74HC565N(1) | 74HC565N(2) |3.2 inch 16bit|2.8 inch SPI |
|:----------:|:-----------:|:-----------:|:------------:|:-----------:|
|    GPIO0   |       DS    |             |              |   SDI(MOSI) |
|    GPIO1   |     SHCP    |     SHCP    |              |     SCK     |
|    GPIO2   |             |             |      RS      |     DC      |
|    GPIO3   |             |             |      WR      |             |
|    GPIO4   |             |             |      RESET   |     RESET   |
|    GPIO5   |     STCP    |     STCP    |              |             |
|     VCC    |      VCC    |     VCC     |              |             |
|     VCC    |       MR    |     MR      |              |             |
|     GND    |      GND    |     GND     |      GND     |     GND     |
|     GND    |       QE    |     QE      |              |             |
|            |      Q7S    |     DS      |              |             |
|            |       Q0    |             |      D00     |             |
|            |       Q1    |             |      D01     |             |
|            |       Q2    |             |      D02     |             |
|            |       Q3    |             |      D03     |             |
|            |       Q4    |             |      D04     |             |
|            |       Q5    |             |      D05     |             |
|            |       Q6    |             |      D06     |             |
|            |       Q7    |             |      D07     |             |
|            |             |     Q0      |      D08     |             |
|            |             |     Q1      |      D09     |             |
|            |             |     Q2      |      D10     |             |
|            |             |     Q3      |      D11     |             |
|            |             |     Q4      |      D12     |             |
|            |             |     Q5      |      D13     |             |
|            |             |     Q6      |      D14     |             |
|            |             |     Q7      |      D15     |             |
|     VCC    |      VCC    |     VCC     |      VCC     |     VCC     |
|     VCC    |             |             |              |BL(High Act) |
|     VCC    |             |             |      RD      |             |
|     GND    |             |             |      CS      |     CS      |
|     GND    |             |             |BL(Low Active)|             |