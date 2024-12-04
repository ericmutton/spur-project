
#include "bq24190.h"
#include "pcf8575.h"
#include "sa868.h"

#define DEBUG Serial
// #define DEBUG_KEYPAD Serial

#define TASK_STACK_SIZE (3072)

/* BQ24190 definitions */
#define SDA_PIN (1)
#define SCL_PIN (3)
#define BQ24190_INT_PIN (18)
#define BQ24190_OTG_PIN (19) // TODO: put on PCF8575
#define BQ24190_NCE_PIN (4)  // TODO: put on PCF8575
bq24190_config_t bq24190;

#define PCF8575_INT_PIN (5)
pcf8575_config_t pcf8575;

/* SA868 definitions */
#define BAUD_RATE (9600)
#define RX_PIN (6)
#define TX_PIN (7)
#define SA868_PTT_PIN (7)
#define SA868_PD_PIN (6)
sa868_config_t sa868;

char keypad_layout[16] = {
  'A','B','C','D',
  '1','2','3','*',
  '4','5','6','0',
  '7','8','9','#'
};

void setup() {
  Serial.begin(115200);
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  // Initialize GPIO expander
  pinMode(PCF8575_INT_PIN, INPUT);
  pcf8575.i2c = &Wire;
  int val = pcf8575_init(pcf8575);
  // Initialize Keypad GPIO pins
  pcf8575_portMode(PORT10, INPUT_PULLUP);
  pcf8575_portMode(PORT11, INPUT_PULLUP);
  pcf8575_portMode(PORT12, INPUT_PULLUP);
  pcf8575_portMode(PORT13, INPUT_PULLUP);
  pcf8575_portMode(PORT14, OUTPUT);
  pcf8575_portMode(PORT15, OUTPUT);
  pcf8575_portMode(PORT16, OUTPUT);
  pcf8575_portMode(PORT17, OUTPUT);
  // Initialize power subsystem
  pinMode(BQ24190_INT_PIN, INPUT);
  pinMode(BQ24190_OTG_PIN, OUTPUT); // active high
  digitalWrite(BQ24190_OTG_PIN, bq24190.OTG);
  pinMode(BQ24190_NCE_PIN, OUTPUT); // active low
  bq24190.pin_interrupt = BQ24190_INT_PIN;
  bq24190.i2c = &Wire;
  digitalWrite(BQ24190_NCE_PIN, LOW);
  val = bq24190_init(bq24190);
  // enable charging in host mode
  bq24190_enableCharging(false);

  pcf8575_portMode(SA868_PTT_PIN, OUTPUT); // active low
  pcf8575_writePort(SA868_PTT_PIN, HIGH);
  pcf8575_portMode(SA868_PD_PIN, OUTPUT); // active low
  pcf8575_writePort(SA868_PD_PIN, HIGH);
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
  // sa868.MAX_CONNECTION_ATTEMPTS = 3;
  sa868.PTT_TIMEOUT = 180; // 180 second timeout
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
  val = sa868_init(sa868);

  xTaskCreate(task, "main_task", TASK_STACK_SIZE, NULL, 10, NULL);
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

static void task(void *arg) {
  int val;
  int ptt_timer = 0; // in seconds
  // keypad entry
  char entry[10] = "";
  bool rx_entry_mode = false, tx_entry_mode = false;
  int keys_entered, key_index, key_scanned, previous_key_scanned, key_ghost_count;
  while (1) {
    bool ptt = (pcf8575_readPort(SA868_PTT_PIN) == LOW);
    if (ptt) {
      if (ptt_timer < sa868.PTT_TIMEOUT) {
        Serial.printf("Currently PTT... (%d sec)\n", ptt_timer);
        vTaskDelay(1000);
        ptt_timer++;
      } else {
        pcf8575_writePort(SA868_PTT_PIN, HIGH);
        ptt_timer = 0;
      }
    } else {
      key_index = pcf8575_scanKeys();
      if (key_index != -1) {
        key_scanned = keypad_layout[key_index];
        if (key_scanned != previous_key_scanned) {
          key_ghost_count = 0;
          previous_key_scanned = key_scanned;
          #ifdef DEBUG_KEYPAD
          DEBUG_KEYPAD.printf("New Key Press: %c\n", key_scanned);
          #endif
        } else {
          #ifdef DEBUG_KEYPAD
          DEBUG_KEYPAD.printf("Same Key Pressed (%d) times: %c\n", key_ghost_count, key_scanned);
          #endif
          key_ghost_count++;
        }
        switch(key_scanned) {
          case('*'):
            if (!rx_entry_mode) {
              rx_entry_mode = true;
              Serial.printf("RX Entry Mode enabled...\n");
              vTaskDelay(1000);
            }
            break;
          case('#'):
            if (!tx_entry_mode) {
              tx_entry_mode = true;
              Serial.printf("TX Entry Mode enabled...\n");
              vTaskDelay(1000);
            }
            break;
          default:
            if (rx_entry_mode || tx_entry_mode) {
              char key_to_enter[2];
              key_to_enter[0] = key_scanned;
              key_to_enter[1] = '\0';
              if (strlen(entry) < 7) {
                strcat(entry, key_to_enter);
                Serial.printf("Current Keypad Entry (%d): %s\n", strlen(entry), entry);
              }
            }
            vTaskDelay(500);
            break;
        }
        if (strlen(entry) >= 7) {
          Serial.printf("Leaving entry mode...\n");
          val = updateFrequency(tx_entry_mode, entry);
          keys_entered = 0;
          strcpy(entry, "");
          rx_entry_mode = false;
          tx_entry_mode = false;
        }
      }
      if (!rx_entry_mode && !tx_entry_mode) {
        bq24190_maintainHostMode();
        val = sa868_communication_handler(RSSI);
        if (val >= 0) {
          Serial.printf("RSSI on %.3d.%4.4d MHz : %.3d dB\n", sa868.rx_freq_mhz, sa868.rx_freq_khz, val);
        }
        vTaskDelay(1000);
      }
    }
  }
}