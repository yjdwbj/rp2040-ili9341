
#define SCREEN_WIDTH    240
#define SCREEN_HEIGHT   320


#define TFT_DOUT    0    // For 74hc595n connect to DS,      For SPI TFT LCD connect to SDO(MISO)
#define TFT_DIN     6    // For 74hc165n connect to DS,      For SPI TFT LCD connect to SDI(MOSI)
#define TFT_CLK     4    // For 74hc595n connect to SHCP,    For SPI TFT LCD connect to SCK
#define TFT_CS      -1    // The bit-bang can be connected to GND, but for SPI it should be set to pin.
#define TFT_RS      1    // For Parallel LCD connect to RS,  For SPI TFT LCD connect to DC
#define TFT_RESET   2    // For Parallel LCD connect to RES, For SPI TFT LCD connect to RES
#define TFT_LATCH   3    // For 74hc595n connect to STCP
#define TFT_WR      5    // For Parallel LCD connect to WR, SPI LCD will no have this pin.


#define SPI_ILI9341_FREQ        70000000 // 48 * 1000 * 1000

// The XPT2046 requires a lower SPI clock rate of 2.5MHz so we define that here:
#define SPI_TOUCH_FREQUENCY     2500000

#define XPT_IRQ         11
#define XPT_CS          8

#define UART_ID         uart0
#define SPI_IDX         spi0

