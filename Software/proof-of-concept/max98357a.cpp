#include "max98357a.h"
#include "pcf8575.h"

extern max98357a_config_t max98357a;

int max98357a_init(max98357a_config_t &configuration) {
  max98357a = configuration;
  if (i2s_driver_install(max98357a.i2s_port_num, &max98357a.i2s_config, 0, NULL)){
    //DBG("Install and start I2S driver failed !");
    return -1;
  }
  if (i2s_set_pin(max98357a.i2s_port_num, &max98357a.pin_config)){
    //DBG("Set I2S pin number failed !");
    return -1;
  }

  //max98357a->i2s.setVolume(5);
  pcf8575_writePort(max98357a.pin_shutdown, HIGH); // active high

  return 0;
}

size_t max98357a_audio_data_in(int16_t data) {
  // Convert to 16-bit signed format for I2S
  int16_t sample = (int16_t)(data); // Center around 0
  size_t i2s_bytes_written = 0;
  i2s_write(max98357a.i2s_port_num, &sample, 4, &i2s_bytes_written, 20);
  return i2s_bytes_written;
}
