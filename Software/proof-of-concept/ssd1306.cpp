#include "ssd1306.h"
#include "HardwareSerial.h"
#include "Wire.h"
extern ssd1306_config_t ssd1306;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int ssd1306_init(ssd1306_config_t &configuration) {
  ssd1306 = configuration;
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    return -1;
  }
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  return 0;
}

void ssd1306_drawScreen(char *radioInfo) {
  display.clearDisplay();
  display.setCursor(0, 0);     // Start at top-left corner
  display.print(radioInfo);
  display.display();
}