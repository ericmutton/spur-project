
#include "bq24190.h"
#include "pcf8575.h"
#include "pam8302a.h"
#include "sa868.h"
#include "ssd1306.h"

#define DEBUG_KEYPAD Serial
#define DEBUG_RSSI Serial

#define TASK_STACK_SIZE (4096)

#define SDA_PIN (1)
#define SCL_PIN (3)

#define PCF8575_INT_PIN (5)
pcf8575_config_t pcf8575;
#define ENCODER_CLK_PIN (PORT00)
#define ENCODER_DT_PIN (PORT01)
#define ENCODER_SW_PIN (PORT07)

#define PTT_BUTTON_PIN (PORT06) 


#define MAX_DISPLAY_COUNT 128
char display_buffer[MAX_DISPLAY_COUNT];
char *display_format_buffer;
ssd1306_config_t ssd1306;
TimerHandle_t displayTimer;

/* BQ24190 definitions */
#define BQ24190_INT_PIN (0)
// #define BQ24190_OTG_PIN
#define BQ24190_NCE_PIN (PORT03)
bq24190_config_t bq24190;
TimerHandle_t powerTimer;

// Table 16. REG09 Fault Register Description
const char* charger_faults_tag[4] {
  "Normal, ",
  "Input Fault (VBUS OVP or VBAT < VBUS < 3.8 V), ",
  "Thermal shutdown, ",
  "Charge Safety Timer Expiration, "
};
const char* ntc_faults_tag[5] {
  "Normal (T2-T3), ",
  "Warm (T3-T5), ",
  "Cool (T1-T2), ",
  "Cold (< T1), ",
  "Hot (> T5), "
};

int key_ghost_count;
int keys_entered;
int short_press_delay_milliseconds = 250;
int long_press_delay_milliseconds = 500;
TimerHandle_t inputTimer;

/* MAX98357A definitions */
// #define MAX98357A_BCLK (8)
// #define MAX98357A_LRCLK (9)
// #define MAX98357A_DIN (2)
// #define MAX98357A_SD (PORT04)
// max98357a_config_t max98357a;

#define PAM8302A_SD (PORT04)
pam8302a_config_t pam8302a;

/* SA868 definitions */
#define BAUD_RATE (9600)
#define RX_PIN (20)
#define TX_PIN (21)
#define SA868_PD_PIN (PORT05)
sa868_config_t sa868;
TimerHandle_t pttTimer;

volatile bool inputFlag = false;
void inputISR() {
  inputFlag = true;
}
// Rotary Encoder Increment Handling
volatile int rotary_encoder_clock = 0;
volatile int rotary_encoder_clocked = 0;
int rotary_encoder_counter = 0;
bool rotary_encoder_direction = false;
bool rotary_encoder_pressed = false;
bool rotary_encoder_ccw = false;
// Quadrature Encoding
void readEncoderDirection() {
  rotary_encoder_clock = pcf8575_readPort(ENCODER_CLK_PIN);
  if (rotary_encoder_clock != rotary_encoder_clocked
    && rotary_encoder_clock == 1) {
      if (pcf8575_readPort(ENCODER_DT_PIN) != rotary_encoder_clock) {
        rotary_encoder_counter--;
        rotary_encoder_ccw = true;
      } else {
        rotary_encoder_counter++;
        rotary_encoder_ccw = false;
      }
      Serial.printf("Rotary Encoder Incremented to (%d) %s\n",
        rotary_encoder_counter,
        rotary_encoder_ccw ? "CW" : "CCW");
  }
  rotary_encoder_clocked = rotary_encoder_clock;
}

// Push-To-Talk Button Handling
volatile bool ptt = false;
int pttElapsedSeconds = 0; // Elapsed PTT time in seconds
int pttTimeoutSeconds = 0; // Previous PTT time in seconds
// Software timer callback for counting PTT time in seconds 
void pttTimeoutCallback(TimerHandle_t xTimer) {
  pttElapsedSeconds++;
}
void activePTT() {
  pttTimeoutSeconds = pttElapsedSeconds;
  // Push-to-Talk has an optional timeout.
  if (sa868.PTT_TIMEOUT_SECONDS != -1 && pttTimeoutSeconds >= sa868.PTT_TIMEOUT_SECONDS) {
    //pcf8575_writePort(SA868_PTT_PIN, HIGH);  // Bypass PTT
    xTimerStop(pttTimer, 0);
    ptt = false;
    Serial.println("PTT Timeout reached, bypassing PTT...");
  }
}

// Potentiometer wiper is on ADC input pin.
#define VOLUME_WIPER_PIN 4

// const int frequency = 440; // frequency of square wave in Hz
// const int amplitude = 500; // amplitude of square wave
// const int sampleRate = 8000; // sample rate in Hz
// const int bps = 16;

// const int halfWavelength = (sampleRate / frequency); // half wavelength of square wave

// short sample = amplitude; // current sample value
// int count = 0;

//i2s_mode_t mode = I2S_PHILIPS_MODE; // I2S decoder is needed


void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

// Received Signal Strength Indicator Handling
void updateRSSI() {
  int val = DMOERROR;
  // val = sa868_communication_handler(RSSI);
  if (val != DMOERROR) {
    // Datasheet suggests the module reports RSSI in dB.
    // What is the fixed reference value for the SA868?
    val += sa868.rssi_offset; // calibration offset for dBm
    //val *= -1; // RSSI is expected to be sub-milliwatt
    // -73 dBm is S9, -174 dBm S0
    sa868.rssi = -1*(255 - val);
    if (sa868.rssi != val) {
      sa868.rssi = val;
    }
  }
}

// Keypad Handling
char entry[8] = "";
bool rx_entry_mode = false, tx_entry_mode = false;

// TODO: make an alarm that indicates short or long keypresses.
void readKeypadEntry(char key) {
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

bool drawScreen = false;
void displayCallback(TimerHandle_t xTimer) {
  drawScreen = true;
}

const char* display_template[5] = {
  "%s RX: %.3d.%4.4d MHz\n",
  "%s TX: %.3d.%4.4d MHz\n",
  "%s ASQ: %d/8 VOL: %d/8\n",
  "%s CXCSS RX=%s\n%s CXCSS TX=%s\n"
};

char* display_format[5];

bool audio_squelch_entry_mode = false;

int8_t selection = 0;
uint16_t rotary_encoder_presses = 0;

void display_DMO_status() {
  display_format_buffer = (char *)pvPortMalloc(MAX_DISPLAY_COUNT);
  memset(display_format_buffer, 0, MAX_DISPLAY_COUNT);
  int offset = 0;
  display_format[0] = (char *)display_template[0];
  display_format[1] = (char *)display_template[1];
  display_format[2] = (char *)display_template[2];
  display_format[3] = (char *)display_template[3];
  selection = rotary_encoder_counter;
  if (rotary_encoder_pressed) {
    rotary_encoder_presses++;
    switch (selection) {
      // Change frequency
      case 0: {
        display_format[0] = (char *)"%s RX: XXX.XXXX MHz\n";
        rx_entry_mode = true;
        tx_entry_mode = false;
      };
      case 1: {
        display_format[1] = (char *)"%s TX: XXX.XXXX MHz\n";
        rx_entry_mode = false;
        tx_entry_mode = true;
      };
      // Change Audio Squelch Level
      case 2: {
        if (audio_squelch_entry_mode) { 
          selection = 2;
          sa868.squelch = rotary_encoder_counter;
          // if (rotary_encoder_presses >= 1 && sa868_communication_handler(SETGROUP) == 0) {
          //   audio_squelch_entry_mode = false;
          //   rotary_encoder_presses = 0;
          //   display_format[2] = (char *)"%s ASQ:  =%d VOL: %d/8\n";
          // }
        } else {
          display_format[2] = (char *)"%s ASQ: X/8 VOL: %d/8\n";
        }
        audio_squelch_entry_mode = true;
      };
      // Select Squelch System
      case 3: {
        display_format[3] = (char *)"%s CXCSS RX=XXXX\n%s CXCSS TX=%s\n";
      };
      case 4: {
        display_format[3] = (char *)"%s CXCSS RX=%s\n%s CXCSS TX=XXXX\n";
      };
    }
    rotary_encoder_pressed = false;
  }
  // carousel selection operation
  if (selection <= -1 ) {
    rotary_encoder_counter = 4;
  }
  if (selection > 4) {
    rotary_encoder_counter = 0;
  }
  offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset, display_format[0],
    (selection == 0) ? "*" : " ", sa868.rx_freq_mhz, sa868.rx_freq_khz
  );
  offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset, display_format[1],
    (selection == 1) ? "*" : " ", sa868.tx_freq_mhz, sa868.tx_freq_khz
  );
  if (ptt) {
      offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset,
        "  PTT: %d/180 sec \n", pttElapsedSeconds
      );
  } else {
      offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset,
        "  RSSI: %.3d dBm (%s)\n",
        sa868.rssi, sa868_s_meter()
      );
  }
  
  offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset,
    display_format[2], (selection == 2) ? "*" : " ",
    sa868.squelch, sa868.volume_level
  );

  offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset,
    display_format[3],
    (selection == 3) ? "*" : " ",
    sa868.rx_subaudio == "0000" ? "NONE" : sa868_ctcss_subtone(sa868.rx_subaudio),
    (selection == 4) ? "*" : " ",
    sa868.tx_subaudio == "0000" ? "NONE" : sa868_ctcss_subtone(sa868.tx_subaudio)
  );

  // Check if the final string fits in the display buffer
  if (offset >= MAX_DISPLAY_COUNT) {
      // Handle error: contents too long
      vPortFree(display_format_buffer);
      return;
  }
  snprintf(display_buffer, MAX_DISPLAY_COUNT, "%s", display_format_buffer);
  vPortFree(display_format_buffer);
}


void display_BMS_status() {
  display_format_buffer = (char *)pvPortMalloc(MAX_DISPLAY_COUNT);
  memset(display_format_buffer, 0, MAX_DISPLAY_COUNT);
  int offset = 0;

  if (bq24190.charger_fault == 0 /*Exclude Charger Fault: Normal*/) {
      offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset,
        "%s\n%s\n%s%s%s%s\n",
            bq24190.vbus_status,
            bq24190.charge_status,
            ((bq24190.system_status & DPM_STAT)   >> 3) ? "VINDPM or IINDPM, " : "Not DPM, ", //0
            ((bq24190.system_status & PG_STAT)    >> 2) ? "Power Good, " : "Not Power Good, ", //0
            ((bq24190.system_status & THERM_STAT) >> 1) ? "In Thermal Regulation, " : "Normal, ", //0
            ((bq24190.system_status & VSYS_STAT)      ) ? "In VSYSMIN Regulation (BAT < VSYSMIN)" : "Not in VSYSMIN Regulation (BAT > VSYSMIN)"
      );
  } else {
      offset += snprintf(display_format_buffer + offset, MAX_DISPLAY_COUNT - offset,
        "Power Faults: %s%s%s%s%s%s", // Enumerate each fault as presented
            ((bq24190.fault_status & WATCHDOG_FAULT) >> 7) ? "WATCHDOG, " : "",
            ((bq24190.fault_status & BOOST_FAULT)    >> 6) ? "BOOST, " : "",
            (bq24190.charger_fault) >= 0                        ? charger_faults_tag[bq24190.charger_fault] : "",
            ((bq24190.fault_status & BAT_FAULT)      >> 3) ? "BATTERY OVP, " : "",
            (bq24190.ntc_fault) >= 0                            ? ntc_faults_tag[bq24190.ntc_fault] : "",
            ((bq24190.fault_status)                 ) == 0 ? "NONE; " : ""
      );
  }
  // Check if the final string fits in the display buffer
  if (offset >= MAX_DISPLAY_COUNT) {
      // Handle error: contents too long
      vPortFree(display_format_buffer);
      return;
  }
  snprintf(display_buffer, MAX_DISPLAY_COUNT, "%s", display_format_buffer);
  vPortFree(display_format_buffer);
}

// #include <I2S.h>
// const int frequency = 440; // frequency of square wave in Hz
// const int amplitude = 500; // amplitude of square wave
// const int sampleRate = 8000; // sample rate in Hz
// const int bps = 16;

// const int halfWavelength = (sampleRate / frequency); // half wavelength of square wave

// short sample = amplitude; // current sample value
// int count = 0;

// i2s_mode_t mode = I2S_PHILIPS_MODE; // I2S decoder is needed

// Volume Level Control Handling
bool updateVolume() {
  int response;
  uint16_t wiper = analogRead(VOLUME_WIPER_PIN);
  uint8_t volume = map(wiper, 0, 4095, 1, 8);
  // analogReadResolution(12);
  if (sa868.volume_level != volume) {
    sa868.volume_level = volume;
    // int response = sa868_communication_handler(SETVOLUME);
  }
  return response;
}
void audioCallback(TimerHandle_t xTimer) {
  // //uint16_t sampled_audio = analogRead(SA868_AF_PIN);
  // if (count % halfWavelength == 0) {
  //     // invert the sample every half wavelength count multiple to generate square wave
  //     sampled_audio = edge ? INT16_MAX : INT16_MIN;
  // }
  // count++;
  // //Serial.printf("SA868_AF_PIN: %d (centered %d)", sampled_audio, sampled_audio - 512);
  // max98357a_audio_data_in(sampled_audio);
}

volatile bool booted = false;
volatile bool debug_mode = false; 
static void task(void *arg) {
  int val;
  while (1) {
    // Flag for change on Port Expander 
    if (inputFlag) {
      // Poll PTT button from port expander.
      ptt = (pcf8575_readPort(PTT_BUTTON_PIN) == LOW);
      // Poll Rotary Encoder from port expander.
      readEncoderDirection();
      rotary_encoder_pressed = (pcf8575_readPort(ENCODER_SW_PIN) == LOW);
      if (!booted && rotary_encoder_pressed) {
        debug_mode = true;
      }
      // Poll keypad from port expander.
      char key = pcf8575_readKeypad(key_ghost_count);
      readKeypadEntry(key);
    }
    booted = true;
    if (ptt) {
      if (!xTimerIsTimerActive(pttTimer)) {
        xTimerStart(pttTimer, 0);
      }
      activePTT();
    } else {
      // once no longer PTT, stop timeout timer and start audio.
      if (xTimerIsTimerActive(pttTimer)) {
        xTimerStop(pttTimer, 0);
        pttTimeoutSeconds = 0;
        pttElapsedSeconds = 0;
      }
    }
    if (!xTimerIsTimerActive(displayTimer)) {
      xTimerStart(displayTimer, 0);
    }
    if (drawScreen) {
      bq24190_maintainHostMode();
      if (debug_mode) {
        bq24190_enableCharging(false);
        display_BMS_status();
      } else {
        updateVolume();
        // updateRSSI();
        display_DMO_status();
      }
      ssd1306_drawScreen(display_buffer);
      drawScreen = false;
    }
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  // Serial.println("I2S simple tone");
  //I2S.setAllPins(MAX98357A_BCLK, MAX98357A_LRCLK, MAX98357A_DIN, 26); // (CLK, WS, IN, OUT)
  // start I2S at the sample rate with 16-bits per sample
  // if (!I2S.begin(mode, sampleRate, bps)) {
  //   Serial.println("Failed to initialize I2S!");
  //   while (1); // do nothing
  // }

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();

  pinMode(VOLUME_WIPER_PIN, INPUT);
  
  // Initialize GPIO expander
  pcf8575.pin_interrupt = PCF8575_INT_PIN;
  attachInterrupt(digitalPinToInterrupt(pcf8575.pin_interrupt), inputISR, CHANGE);
  pcf8575.i2c = &Wire;
  int val = pcf8575_init(pcf8575);
  pcf8575_portMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pcf8575_portMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pcf8575_portMode(ENCODER_SW_PIN, INPUT_PULLUP);
  rotary_encoder_clocked = pcf8575_readPort(ENCODER_CLK_PIN); // initial quadrature
  // Initialize Keypad GPIO pins
  pcf8575_portMode(PORT10, INPUT_PULLUP);
  pcf8575_portMode(PORT11, INPUT_PULLUP);
  pcf8575_portMode(PORT12, INPUT_PULLUP);
  pcf8575_portMode(PORT13, INPUT_PULLUP);
  pcf8575_portMode(PORT14, OUTPUT);
  pcf8575_portMode(PORT15, OUTPUT);
  pcf8575_portMode(PORT16, OUTPUT);
  pcf8575_portMode(PORT17, OUTPUT);

  // Initialize OLED display
  ssd1306.i2c = &Wire;
  snprintf(display_buffer, MAX_DISPLAY_COUNT, "%s", "UHF Handheld Radio\nBased on SA8X8 Module\nde Eric Mutton KJ5DIF\nThanks to UTD ECE\n& Dr. Bill Swartz");
  do {
    val = ssd1306_init(ssd1306);
    ssd1306_drawScreen(display_buffer);
    vTaskDelay(1000/portTICK_PERIOD_MS);
  } while (val == -1);
  // Initialize power subsystem
  bq24190.OTG = true;
  bq24190.pin_interrupt = BQ24190_INT_PIN;
  pinMode(bq24190.pin_interrupt, INPUT_PULLUP);
  // pcf8575_portMode(BQ24190_OTG_PIN, OUTPUT); // active high
  // pcf8575_writePort(BQ24190_OTG_PIN, bq24190.OTG);
  pcf8575_portMode(BQ24190_NCE_PIN, OUTPUT); // active low
  pcf8575_writePort(BQ24190_NCE_PIN, LOW);
  bq24190.i2c = &Wire;
  val = bq24190_init(bq24190);
  // enable charging in host mode
  bq24190_enableCharging(false);

  // Initialize audio subsystem
  // i2s_pin_config_t pin_config = {
  //   .bck_io_num = MAX98357A_BCLK,   // Serial clock (SCK), aka bit clock (BCK)
  //   .ws_io_num = MAX98357A_LRCLK,   // Word select (WS), i.e. command (channel) select, used to switch between left and right channel data
  //   .data_out_num = MAX98357A_DIN,   // Serial data signal (SD), used to transmit audio data in two's complement format
  //   .data_in_num = I2S_PIN_NO_CHANGE  // Not used
  // };
  // max98357a.pin_config = pin_config;
  // max98357a.pin_shutdown = MAX98357A_SD;
  // pcf8575_portMode(max98357a.pin_shutdown, OUTPUT);
  // pcf8575_writePort(max98357a.pin_shutdown, LOW); // active high
  // val = max98357a_init(max98357a);
  pam8302a.pin_shutdown = PAM8302A_SD;
  pcf8575_portMode(pam8302a.pin_shutdown, OUTPUT);
  pcf8575_writePort(pam8302a.pin_shutdown, LOW); // active high
  val = pam8302a_init(pam8302a);
  // audioTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz)
  // timerAttachInterrupt(audioTimer, &audioCallback, true);
  //timerAlarmWrite(audioTimer, 12500, true); // 1 second
  //timerAlarmEnable(audioTimer); // Start the timer

  // Initialize communications subsystem 
  sa868.pin_shutdown = SA868_PD_PIN;
  pcf8575_portMode(sa868.pin_shutdown, OUTPUT); 
  pcf8575_writePort(sa868.pin_shutdown, LOW); // active low
  pcf8575_portMode(PTT_BUTTON_PIN, INPUT_PULLUP); // active low
  /**
   * UART Interface Format:
   * Baud rate = 9600 Baud
   * Data bit = 8 bit
   * Parity = None
   * Stop bit = 1 bit
   * Terminal Rx,Tx pins = GPIO20,GPIO21 @ 3.3V
   */
  Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  sa868.UART = &Serial0; // using UART0 for SA868
  // sa868.MAX_CONNECTION_ATTEMPTS = 3;
  sa868.PTT_TIMEOUT_SECONDS = 180; // 180 second timeout
  sa868.bandwidth_wide_fm = 1; // BW = 25 kHz
  sa868.squelch = 3;
  sa868.volume_level = 3;

  // int repeater_offset = 5; // +5 MHz
  // sa868.rx_subaudio = "0015"; // CTCSS = 110.9 Hz
  // sa868.rx_freq_mhz = 443; 
  // sa868.rx_freq_khz = 3750;
  // sa868.tx_freq_mhz = sa868.rx_freq_mhz + repeater_offset; // 448 
  // sa868.tx_freq_khz = 3750; 

  // national calling frequency 446.0000 MHz
  sa868.rx_freq_mhz = 446;
  sa868.rx_freq_khz = 0000;
  sa868.tx_freq_mhz = 446;
  sa868.tx_freq_khz = 0000;
  // dBm = dBW - (30 dBm - 16 dBm)
  //sa868.rssi_offset = 99;
  
  // val == DMOERROR;
  // do {
  //   val = sa868_init(sa868);
  //   vTaskDelay(1000/portTICK_PERIOD_MS);
  // } while (val == DMOERROR);

  // Display Refresh Timer
  displayTimer = xTimerCreate("DisplayTimer", pdMS_TO_TICKS(1000), pdFALSE, NULL, displayCallback);
  // PTT Timeout Timer
  pttTimer = xTimerCreate("PTTTimer", pdMS_TO_TICKS(1000), pdFALSE, NULL, pttTimeoutCallback);

  xTaskCreate(task, "main_task", TASK_STACK_SIZE, NULL, 10, NULL);
}