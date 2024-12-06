#include "max98357a.h"

extern max98357a_config_t max98357a;

int max98357a_init(max98357a_config_t &configuration) {
  max98357a = configuration;
  return 0;
}