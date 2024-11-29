
#include "bq24190.h"
#include "pcf8575.h"
#include "sa868.h"

// #define DEBUG
#define DEBUG_BQ24190


#define TASK_STACK_SIZE (3072)

/* BQ24190 definitions */
#define SDA_PIN (1)
#define SCL_PIN (3)
#define BQ24190_INT_PIN (18)
#define BQ24190_OTG_PIN (19) // TODO: put on PCF8575
#define BQ24190_NCE_PIN (4)  // TODO: put on PCF8575
bq24190_config_t bq24190;

#define PCF8575_INT_PIN (-1) // TODO: Free up GPIO
pcf8575_config_t pcf8575;

/* SA868 definitions */
#define BAUD_RATE (9600)
#define RX_PIN (6)
#define TX_PIN (7)
#define SA868_PTT_PIN (10) // TODO: put on PCF8575
#define SA868_PD_PIN (9)   // TODO: put on PCF8575
sa868_config_t sa868;

void setup() {
  Serial.begin(115200);

  pinMode(BQ24190_INT_PIN, INPUT);
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  bq24190.i2c = &Wire;

  pinMode(BQ24190_OTG_PIN, OUTPUT); // active high
  digitalWrite(BQ24190_OTG_PIN, LOW);
  pinMode(BQ24190_NCE_PIN, OUTPUT); // active low
  digitalWrite(BQ24190_NCE_PIN, HIGH);
  bq24190.pin_interrupt = BQ24190_INT_PIN;

  pcf8575.i2c = &Wire;

  pinMode(SA868_PTT_PIN, OUTPUT); // active low
  digitalWrite(SA868_PTT_PIN, HIGH);
  pinMode(SA868_PD_PIN, OUTPUT); // active low
  digitalWrite(SA868_PD_PIN, HIGH);
  /**
   * UART Interface Format:
   * Baud rate = 9600 Baud
   * Data bit = 8 bit
   * Parity = None
   * Stop bit = 1 bit
   * Terminal Rx,Tx pins = GPIO6,GPIO7 @ 3.3V
   */
  Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  sa868.UART = &Serial1; // using UART1 for SA868
  //sa868.MAX_CONNECTION_ATTEMPTS = 3;
  sa868.bandwidth_wide_fm = 1; // BW = 25 kHz
  sa868.squelch = 2;
  sa868.volume_level = 3;

  // int repeater_offset = 5; // +5 MHz
  // sa868.rx_subaudio = "0015"; // CTCSS = 110.9 Hz
  // sa868.rx_freq_mhz = 443; 
  // sa868.rx_freq_khz = 3750;
  // sa868.tx_freq_mhz = sa868.rx_freq_mhz + repeater_offset; // 448 
  // sa868.tx_freq_khz = 3750; 

  // national calling frequency 446.0000 MHz
  sa868.rx_freq_mhz = 446;
  sa868.rx_freq_khz = 0;
  sa868.tx_freq_mhz = 446;
  sa868.tx_freq_khz = 0;

  xTaskCreate(task, "uart_task", TASK_STACK_SIZE, NULL, 10, NULL);
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

static void task(void *arg) {
  int val;
  // power subsystem
  val = bq24190_init(bq24190);
  digitalWrite(BQ24190_NCE_PIN, LOW);
  // enable charging in host mode
  bq24190_enableCharging(false);

  val = sa868_init(sa868);
  if (val == 0) {
    #ifdef DEBUG
    Serial.printf("Connection successful, handshake return value: %d\r\n", val);
    #endif
    while (1) {
      bq24190_maintainHostMode();
      val = sa868_communication_handler(RSSI);
      if (val >= 0) {
        Serial.printf("RSSI on %.3d.%4.4d MHz : %.3d dB\n", sa868.rx_freq_mhz, sa868.rx_freq_khz, val);
      }
      vTaskDelay(1000);
    }
  } else {
    #ifdef DEBUG
    Serial.println("Module should be restarted.\n");
    #endif
  }
}