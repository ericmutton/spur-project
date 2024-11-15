/*
 * precondition: Configure and expose two UARTs on the ESP32C3.
 * - UART0 is used as the USB CDC and Default Zephyr Shell peripheral.
 * - UART1 is used as the Hayes AT and SA868 Transmission peripheral.
 * postcondition: Implement SA868 Terminal-Module communication protocol
 * https://github.com/espressif/arduino-esp32/blob/09a6770320b75c219053aa19d630afe1a7c61147/tests/validation/uart/uart.ino#L563
 */
#include "sa868.h"

#define DEBUG

#define TASK_STACK_SIZE (3072)

#define RX_PIN (6)
#define TX_PIN (7)
#define PTT_PIN (8)
#define PD_PIN (9)

sa868_config_t sa868;

void setup() {
  Serial.begin(115200);
  /**
   * UART Interface Format:
   * Baud rate = 9600 Baud
   * Data bit = 8 bit
   * Parity = None
   * Stop bit = 1 bit
   * Terminal Rx,Tx pins = GPIO6,GPIO7 @ 3.3V
   */
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  //config.MAX_CONNECTION_ATTEMPTS = 3;
  sa868.bandwidth_wide_fm = 1; // BW = 25 kHz
  sa868.squelch = 3;
  sa868.volume_level = 3;

  int repeater_offset = 5; // +5 MHz
  sa868.rx_subaudio = "0015"; // CTCSS = 110.9 Hz
  sa868.rx_freq_mhz = 443; 
  sa868.rx_freq_khz = 3750;
  sa868.tx_freq_mhz = sa868.rx_freq_mhz + repeater_offset; // 448 
  sa868.tx_freq_khz = 3750; 

  xTaskCreate(task, "uart_task", TASK_STACK_SIZE, NULL, 10, NULL);
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

static void task(void *arg) {
  int val = sa868_init(sa868);
  if (val == 0) {
    #ifdef DEBUG
    Serial.printf("Connection successful, handshake return value: %d\r\n", val);
    #endif
    while (1) {
      val = sa868_communication_handler(sa868, RSSI);
      if (val >= 0) {
        Serial.printf("RSSI on %.3d.%4.0d MHz : %.3d dB\n", sa868.rx_freq_mhz, sa868.rx_freq_khz, val);
      }
      vTaskDelay(1000);
    }
  } else {
    #ifdef DEBUG
    Serial.println("Module should be restarted.\n");
    #endif
  }
}