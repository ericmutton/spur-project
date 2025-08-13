#include <stdint.h>

typedef struct {
  //Adafuit PAM8302A Module
  uint8_t pin_shutdown;
} pam8302a_config_t;

int pam8302a_init(pam8302a_config_t &configuration);