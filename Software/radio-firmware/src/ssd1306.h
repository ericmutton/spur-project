#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

typedef struct {
  TwoWire *i2c;
  uint8_t sensor_address = SCREEN_ADDRESS; // known sensor address
} ssd1306_config_t;

int ssd1306_init(ssd1306_config_t &configuration);
void ssd1306_drawScreen(char *radioInfo);
