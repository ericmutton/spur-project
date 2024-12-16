#include "hal/i2s_types.h"
#include <driver/i2s.h>
// #include <DFRobot_MAX98357A.h>

static i2s_config_t i2s_config_default = {
    .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),  // The main controller can transmit data but not receive.
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,       // 16 bits per sample
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,       // 2-channels
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // I2S communication I2S Philips standard, data launch at second BCK
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // Interrupt level 1
    .dma_buf_count = 4,                                 // number of buffers, 128 max.
    .dma_buf_len = 400,                                 // size of each buffer, AVRC communication may be affected if the value is too high.
    .use_apll = false,                                  // For the application of a high precision clock, select the APLL_CLK clock source in the frequency range of 16 to 128 MHz. It's not the case here, so select false.
    .tx_desc_auto_clear = true
};

typedef struct {
  //DFRobot_MAX98357A *i2s;
  i2s_port_t i2s_port_num = I2S_NUM_0;
  i2s_config_t i2s_config = i2s_config_default;
  i2s_pin_config_t pin_config;
  uint8_t pin_shutdown;
} max98357a_config_t;

int max98357a_init(max98357a_config_t &configuration);

size_t max98357a_audio_data_in(int16_t data);