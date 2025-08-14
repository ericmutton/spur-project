#include "pam8302a.h"
#include "pcf8575.h"

extern pam8302a_config_t pam8302a;

int pam8302a_init(pam8302a_config_t &configuration) {
  pam8302a = configuration;

  pcf8575_writePort(pam8302a.pin_shutdown, HIGH); // active low

  return 0;
}