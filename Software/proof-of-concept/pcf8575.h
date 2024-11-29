
#include <stdint.h>
#include <stdio.h>
#include "Wire.h"

typedef struct {
  TwoWire *i2c;
  uint8_t sensor_address = 0x41;
} pcf8575_config_t;

int pcf8575_init(pcf8575_config_t &configuration);