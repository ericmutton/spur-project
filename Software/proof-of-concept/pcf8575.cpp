#include "pcf8575.h"

// TODO: Phase out adafruit BusIO dependency
#include <Adafruit_PCF8575.h>
Adafruit_PCF8575 pcf;
const pcf8575_address_map_t rows[4] = {PORT17, PORT16, PORT15, PORT14};
const pcf8575_address_map_t cols[4] = {PORT13, PORT12, PORT11, PORT10};

//#define DEBUG_PCF8575 Serial

extern pcf8575_config_t pcf8575;
// char scan_buffer[sizeof(uint32_t)];

bool pcf8575_portMode(uint8_t port, uint8_t mode) {
  return pcf.pinMode(port, mode);
}

int pcf8575_readPort(uint8_t port) {
  return pcf.digitalRead((uint8_t)port);
  // uint8_t bytes_transferred = 0;
  // TwoWire *i2c = pcf8575.i2c;
  // i2c->beginTransmission(pcf8575.sensor_read_address); // transmit sensor address
  // i2c->write(port);                                    // transmit port address
  // if (!(i2c->endTransmission(true))) {
  //   bytes_transferred = i2c->requestFrom(pcf8575.sensor_read_address, sizeof(uint16_t), true);
  //   if (bytes_transferred > 0 && bytes_transferred == sizeof(uint16_t)) {
  //     i2c->readBytes(scan_buffer, bytes_transferred);
  //   }
  // }
  // return bytes_transferred;
}

// int pcf8575_pinMode(uint8_t access_buffer, pcf8575_address_map_t port, uint8_t value) {
//   int bytes_transferred = 0;
//   strcpy(access_buffer, "");
//   TwoWire *i2c = pcf8575.i2c;
//   i2c->beginTransmission(pcf8575.sensor_read_address); // transmit sensor address
//   if ((value == INPUT) || (value == INPUT_PULLUP)) {
//     access_buffer |= (uint8_t *)(1UL << port);
//   } else {
//     access_buffer &= (uint8_t *)(~(1UL << port));
//   }
//   bytes_transferred += i2c->write((uint8_t)&access_buffer); // transmit register address
//   if (!(i2c->endTransmission(true))) {
//     bytes_transferred = sizeof(uint16_t);
//   }
//   return bytes_transferred;
// }

int pcf8575_writePort(uint8_t port, uint8_t value) {
  return pcf.digitalWrite(port, value);
  // int bytes_transferred = 0;
  // TwoWire *i2c = pcf8575.i2c;
  // i2c->beginTransmission(pcf8575.sensor_read_address); // transmit sensor address
  // bytes_transferred += i2c->write(port);               // transmit register address
  // bytes_transferred += i2c->write(value);              // transmit write value
  // if (!(i2c->endTransmission(true))) {
  //   bytes_transferred = sizeof(uint8_t);
  // }
  // return bytes_transferred;
}

int pcf8575_init(pcf8575_config_t &configuration) {
  pcf8575 = configuration;
  pcf.begin(pcf8575.sensor_address, pcf8575.i2c);
  return 0;
}

int pcf8575_scanKeys() {
  int val = -1;
  int row = 0;
  int col = 0;
  for (row = 0; row < 4; row++) {
    pcf8575_writePort(rows[0], HIGH);
    pcf8575_writePort(rows[1], HIGH);
    pcf8575_writePort(rows[2], HIGH);
    pcf8575_writePort(rows[3], HIGH);
    for (col = 0; col < 4; col++) {
      pcf8575_writePort(rows[row], LOW); // Pull-down to scan PORT
      if (pcf8575_readPort(cols[col]) == LOW) {
        #ifdef DEBUG_PCF8575
        DEBUG_PCF8575.printf("Read in pushbutton (%d,%d) via (PORT%.2d,PORT%.2d)\n", row, col, rows[row]+2, cols[col]+2);
        #endif
        val = (row * 4) + col;
      }
    }
    pcf8575_writePort(rows[row], HIGH); // Pull-up PORTs
  }
  return val;
}