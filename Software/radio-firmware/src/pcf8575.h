
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "HardwareSerial.h"
#include "Wire.h"


typedef struct {
  TwoWire *i2c;
  uint8_t sensor_address = 0x20; // known sensor address
  uint8_t pin_interrupt;
  // Select one of eight PCF8575 sensor address pairs
  // uint8_t sensor_read_address = 0x41;  // 0x41 -> 0x4F
  // uint8_t sensor_write_address = 0x40; // 0x40 -> 0x4E
} pcf8575_config_t;

typedef enum {
  // LOW,   HIGH,    LOW,    LOW,  ADDR2,  ADDR1,  ADDR0,   R/NW
  PORT00, PORT01, PORT02, PORT03, PORT04, PORT05, PORT06, PORT07, // P0x I/O data bus
  PORT10, PORT11, PORT12, PORT13, PORT14, PORT15, PORT16, PORT17  // P1x I/O data bus
} pcf8575_address_map_t;


bool pcf8575_portMode(uint8_t port, uint8_t mode);

int pcf8575_readPort(uint8_t port);

int pcf8575_writePort(uint8_t port, uint8_t value);

int pcf8575_init(pcf8575_config_t &configuration);

int pcf8575_scanKeys();

char pcf8575_readKeypad(int &key_ghost_count);