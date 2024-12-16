
#include "bq24190.h"
#include "pcf8575.h"
#include "max98357a.h"
#include "sa868.h"

#define DEBUG Serial
// #define DEBUG_KEYPAD Serial

#define TASK_STACK_SIZE (3072)

/* BQ24190 definitions */
#define SDA_PIN (1)
#define SCL_PIN (3)
#define BQ24190_INT_PIN (4)
#define BQ24190_OTG_PIN (PORT04)
#define BQ24190_NCE_PIN (PORT03)
bq24190_config_t bq24190;

#define PCF8575_INT_PIN (5)
pcf8575_config_t pcf8575;

int key_ghost_count;
int keys_entered;
int short_press_delay_milliseconds = 250;
int long_press_delay_milliseconds = 500;
TimerHandle_t keypadTimer;

/* MAX98357A definitions */
#define MAX98357A_BCLK (8)
#define MAX98357A_LRCLK (9)
#define MAX98357A_DIN (2)
#define MAX98357A_SD (PORT02)
max98357a_config_t max98357a;
hw_timer_t *audioTimer = NULL;

/* SA868 definitions */
#define BAUD_RATE (9600)
#define RX_PIN (6)
#define TX_PIN (7)
#define SA868_PTT_PIN (PORT07)
#define SA868_PTT_BUTTON (PORT06) // Digital Input 
#define SA868_PD_PIN (PORT05)
#define SA868_AF_PIN (0) // must be Analog Input
sa868_config_t sa868;
TimerHandle_t pttTimer;
TimerHandle_t rssiTimer;

volatile bool inputFlag = false;
void inputISR() {
  inputFlag = true;
}

void setup() {
  Serial.begin(115200);
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  // Initialize GPIO expander
  pcf8575.pin_interrupt = PCF8575_INT_PIN;
  attachInterrupt(digitalPinToInterrupt(PCF8575_INT_PIN), inputISR, CHANGE);
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
  bq24190.pin_interrupt = BQ24190_INT_PIN;
  pinMode(bq24190.pin_interrupt, INPUT_PULLUP);
  pcf8575_portMode(BQ24190_OTG_PIN, OUTPUT); // active high
  pcf8575_writePort(BQ24190_OTG_PIN, bq24190.OTG);
  pcf8575_portMode(BQ24190_NCE_PIN, OUTPUT); // active low
  pcf8575_writePort(BQ24190_NCE_PIN, LOW);
  bq24190.i2c = &Wire;
  val = bq24190_init(bq24190);
  // enable charging in host mode
  bq24190_enableCharging(false);

  // Initialize audio subsystem
  i2s_pin_config_t pin_config = {
    .bck_io_num = MAX98357A_BCLK,   // Serial clock (SCK), aka bit clock (BCK)
    .ws_io_num = MAX98357A_LRCLK,   // Word select (WS), i.e. command (channel) select, used to switch between left and right channel data
    .data_out_num = MAX98357A_DIN,   // Serial data signal (SD), used to transmit audio data in two's complement format
    .data_in_num = I2S_PIN_NO_CHANGE  // Not used
  };
  max98357a.pin_config = pin_config;
  max98357a.pin_shutdown = MAX98357A_SD;
  pcf8575_portMode(max98357a.pin_shutdown, OUTPUT);
  pcf8575_writePort(max98357a.pin_shutdown, LOW); // active high
  val = max98357a_init(max98357a);

  audioTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz)
  timerAttachInterrupt(audioTimer, &audioCallback, true);
  //timerAlarmWrite(audioTimer, 12500, true); // 1 second
  //timerAlarmEnable(audioTimer); // Start the timer

  // Initialize communications subsystem
  pcf8575_portMode(SA868_PTT_PIN, OUTPUT); // active low
  pcf8575_writePort(SA868_PTT_PIN, HIGH);
  pcf8575_portMode(SA868_PD_PIN, OUTPUT); // active low
  pcf8575_writePort(SA868_PD_PIN, HIGH);
  pcf8575_portMode(SA868_PTT_BUTTON, INPUT_PULLUP); // active low
  pinMode(SA868_AF_PIN, INPUT);
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
  sa868.PTT_TIMEOUT_SECONDS = 180; // 180 second timeout
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

  //attachInterrupt(digitalPinToInterrupt(SA868_PTT_BUTTON), pttPressed, FALLING);
  pttTimer = xTimerCreate("PTTTimer", pdMS_TO_TICKS(1000), pdFALSE, NULL, pttTimeoutCallback);
  keypadTimer = xTimerCreate("KeypadTimer", pdMS_TO_TICKS(short_press_delay_milliseconds), pdFALSE, NULL, keypadCallback);
  rssiTimer = xTimerCreate("RSSITimer", pdMS_TO_TICKS(1000), pdFALSE, NULL, rssiCallback);

  xTimerStart(keypadTimer, 0);
  Serial.println("Keypad enabled...");

  xTaskCreate(task, "main_task", TASK_STACK_SIZE, NULL, 10, NULL);
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

int pttElapsedSeconds = 0;

void pttTimeoutCallback(TimerHandle_t xTimer) {
    pttElapsedSeconds++;
    if (pttElapsedSeconds >= sa868.PTT_TIMEOUT_SECONDS) {
        pcf8575_writePort(SA868_PTT_PIN, HIGH);  // Bypass PTT
        Serial.println("PTT Timeout reached, bypassing PTT...");
        xTimerStop(pttTimer, 0);
        pttElapsedSeconds = 0;
    } else {
        Serial.printf("Currently PTT... (%d sec)\n", pttElapsedSeconds);
    }
}
// keypad entry
char entry[8] = "";
bool rx_entry_mode = false, tx_entry_mode = false;

void keypadCallback(TimerHandle_t xTimer) {
  char key = pcf8575_readKeypad(key_ghost_count);
  if (key != '\0') {
    char key_to_enter[2];
    key_to_enter[0] = key;
    key_to_enter[1] = '\0';
    #ifdef DEBUG_KEYPAD
    DEBUG_KEYPAD.printf("Keypress to use: %s x%d\n", key_to_enter, key_ghost_count+1);
    #endif
    switch(key) {
      case('*'): // Use as RX Entry (7 chars)
        if (!rx_entry_mode && key_ghost_count == 2) {
          rx_entry_mode = true;
          strcpy(entry, "");
          Serial.printf("RX Entry Mode enabled...\n");
        }
        //vTaskDelay(long_press_delay_milliseconds);
        break;
      case('#'): // Use as TX Entry (7 chars)
        if (!tx_entry_mode && key_ghost_count == 2) {
          tx_entry_mode = true;
          strcpy(entry, "");
          Serial.printf("TX Entry Mode enabled...\n");
        }
        //vTaskDelay(long_press_delay_milliseconds);
        break;
      default:
        if ((rx_entry_mode || tx_entry_mode) && strlen(entry) < 7) {
          strcat(entry, key_to_enter);
          Serial.printf("Current Keypad Entry (%d): %s\n", strlen(entry), entry);
        }
        //vTaskDelay(short_press_delay_milliseconds);
        break;
    }
  }
  if (strlen(entry) >= 7) {
    Serial.printf("Leaving entry mode...\n");
    int val = updateFrequency(tx_entry_mode, entry);
    strcpy(entry, "");
    rx_entry_mode = false;
    tx_entry_mode = false;
  }
}

void rssiCallback(TimerHandle_t xTimer) {
  bq24190_maintainHostMode();
  int val = sa868_communication_handler(RSSI);
  if (val >= 0) {
    Serial.printf("RSSI on %.3d.%4.4d MHz : %.3d dB\n", sa868.rx_freq_mhz, sa868.rx_freq_khz, val);
  }
}

const int frequency = 440; // frequency of square wave in Hz

const int amplitude = INT16_MAX; // amplitude of square wave

const int sampleRate = 8000; // sample rate in Hz

const int halfWavelength = (sampleRate / frequency) / 2; // half wavelength of square wave

int16_t sampled_audio = amplitude; // current sample value
int count = 0;
bool edge = false;

void audioCallback() {
  //uint16_t sampled_audio = analogRead(SA868_AF_PIN);
  if (count % halfWavelength == 0) {
      // invert the sample every half wavelength count multiple to generate square wave
      sampled_audio = edge ? INT16_MAX : INT16_MIN;
  }
  count++;
  //Serial.printf("SA868_AF_PIN: %d (centered %d)", sampled_audio, sampled_audio - 512);
  max98357a_audio_data_in(sampled_audio);
}


static void task(void *arg) {
  int val;
  while (1) {
    bool ptt = (pcf8575_readPort(SA868_PTT_BUTTON) == LOW);
    if (ptt) {
      if (!xTimerIsTimerActive(pttTimer)) {
        xTimerStart(pttTimer, 0);
        Serial.println("PTT activated...");
      }
      if (xTimerIsTimerActive(keypadTimer)) {
        xTimerStop(keypadTimer, 0); // keypad input is disabled during PTT.
        Serial.println("Keypad deactivated...");
      }
      if (xTimerIsTimerActive(rssiTimer)) {
        xTimerStop(rssiTimer, 0);
        Serial.println("RSSI deactivated...");
      }
    } else {
      if (xTimerIsTimerActive(pttTimer)) {
        xTimerStop(pttTimer, 0);
        Serial.println("PTT released...");
      }
      if (!xTimerIsTimerActive(keypadTimer)) {
        xTimerStart(keypadTimer, 0);
      }
      if (!rx_entry_mode && !tx_entry_mode) {
        if (!xTimerIsTimerActive(rssiTimer)) {
          xTimerStart(rssiTimer, 0);
        }
      }
      vTaskDelay(50); // Short delay to PTT debounce
    }
  }
  vTaskDelete(NULL);
}