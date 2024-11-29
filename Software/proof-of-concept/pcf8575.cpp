
#include "pcf8575.h"

extern pcf8575_config_t pcf8575;

int pcf8575_init(pcf8575_config_t &configuration) {
   pcf8575 = configuration;
}