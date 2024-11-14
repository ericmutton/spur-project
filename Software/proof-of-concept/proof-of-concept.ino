/*
 * precondition: Configure and expose two UARTs on the ESP32C3.
 * - UART0 is used as the USB CDC and Default Zephyr Shell peripheral.
 * - UART1 is used as the Hayes AT and SA868 Transmission peripheral.
 * postcondition: Implement SA868 Terminal-Module communication protocol
 * https://github.com/espressif/arduino-esp32/blob/09a6770320b75c219053aa19d630afe1a7c61147/tests/validation/uart/uart.ino#L563
 */

// From FreeRTOS
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
// From Arduino Core for ESP32
#include "HardwareSerial.h"

#define DEBUG

#define MAX_BUFFER_SIZE (50)
#define TASK_STACK_SIZE (3072)


typedef struct {
    uint8_t MAX_CONNECTION_ATTEMPTS = 3;
    uint16_t sweep_freq_mhz; uint16_t sweep_freq_khz;

    bool bandwidth_wide_fm;
    uint16_t tx_freq_mhz; uint16_t tx_freq_khz;
    uint16_t rx_freq_mhz; uint16_t rx_freq_khz;
    char *tx_subaudio = "0000";
    uint8_t squelch;
    char *rx_subaudio = "0000";

    uint8_t volume_level;

    bool emphasis_bypass;
    bool voice_highpass_bypass;
    bool voice_lowpass_bypass;  

} sa868_config_t;

/** 
 * SA868 Instruction Set (<CR><LF>)
 * CONNECT: Detect the module is in normal working status.
 * SWEEP: Set the frequency to scan
 * SETGROUP: Set the working parameters of the module
 * SETVOLUME: Set the volume level of the module
 * RSSI: Read Received Signal Strength Indicator value of the module
 * SETFILTER: Set the filter of the module
 */
enum sa868_instruction_set {
  CONNECT = 0,  // AT+DMOCONNECT
  SWEEP,        // S+SWEEP=?
  SETGROUP,     // AT+DMOSETGROUP=?
  SETVOLUME,    // AT+DMOSETVOLUME=?
  RSSI,         // AT+RSSI?
  SETFILTER,    // AT+SETFILTER=?
  UNKNOWN = -1  // Unknown instruction
};
char *command_formats[] = {
  "AT+DMOCONNECT\r\n",
  /**
   * RX_freq
   * = [134:174].[mod 1250]: valid VHF frequency when BW=0
   * = [400:480].[mod 2500]: valid UHF frequency when BW=1
   */
  "S+%.3d.%.4d\r\n",
  /**
   * BW=[0:1]: 0 = 12.5kHz or 1 = 25kHz
   * SQ=[0:8]: 0 = listen mode or [1:8] squelch
   * Tx_freq, Rx_freq
   * = [134:174].[mod 1250]: valid VHF frequency when BW=0
   * = [400:480].[mod 2500]: valid UHF frequency when BW=1
   * Tx_subaudio, Rx_subaudio
   * = 0000: subaudio disabled 
   * = [0001:0038]: CTCSS
   * = [023?:754?]: CDCSS code (?=N indicates complement or ?=I)
   */
  "AT+DMOSETGROUP=%i,%.3d.%.4d,%.3d.%.4d,%.4s,%i,%.4s\r\n",
  // X=[1:8]: set configured volume level to X
  "AT+DMOSETVOLUME=%i\r\n",
  /**
   * X=1: emphasis,voice_highpass,voice_lowpass bypass
   * X=0: emphasis,voice_highpass,voice_lowpass normal
   */
  "AT+RSSI?\r\n",
  "AT+SETFILTER=%i,%i,%i\r\n"
};
char *command_instructions[] = {
  "AT+DMOCONNECT",    // CONNECT
  "S+",               // SWEEP
  "AT+DMOSETGROUP",   // SETGROUP
  "AT+DMOSETVOLUME",  // SETVOLUME
  "AT+RSSI?",         // RSSI
  "AT+SETFILTER"      // SETFILTER
};
char *response_formats[] = {
  // +DMOCONNECT:0 -> Normal working status
  "+DMOCONNECT:%i\r\n",
  // S=0 -> The frequency to be scanned has a signal
  // S=1 -> There is no signal to sweep the frequency
  "S=%i\r\n",
  // +DMOSETGROUP:0 -> Successfully set working parameters
  // +DMOSETGROUP:1 -> Data setting is out of range
  "+DMOSETGROUP:%i\r\n",
  // +DMOSETVOLUME:0 -> Successfully set volume level
  // +DMOSETVOLUME:1 -> Failed volume level setup
  "+DMOSETVOLUME:%i\r\n",
  // RSSI:XXX -> Current signal strength value from 0 to 255, unit 1dB
  "RSSI=%d\r\n",
  // +DMOSETFILTER:0 -> Successfully set module filter
  // +DMOSETFILTER:1 -> Failed filter setup
  "+DMOSETFILTER:%i\r\n",
  // UNKNOWN ERROR
  "+DMOERROR:%i\r\n"
};
const int DMOERROR = -2;

// const int INSTRUCTION_COUNT = sizeof(command_instructions) / sizeof(command_instructions[0]);
/**
 * @brief this getter matches a substring of a command with an instruction.  
 * @param command cstring to associate with an instruction
 * @returns the associated enum sa868_instruction_set value
 */
// enum sa868_instruction_set getInstructionFromCommand(char *command) {
//   char *command_instruction;
//   int diff;
//   for (int instruction = CONNECT; instruction < INSTRUCTION_COUNT; instruction++) {
//     command_instruction = command_instructions[instruction];
//     diff = strncmp(command, command_instruction, strlen(command_instruction));
//     if (diff == 0) {
//       // Instruction was found in command
//       return (enum sa868_instruction_set)instruction;  // Return the corresponding enum value
//     }
//   }
//   return (enum sa868_instruction_set)(UNKNOWN);  // Return unknown instruction if not found
// }

sa868_config_t config;

char command_buffer[MAX_BUFFER_SIZE];
char *getCommandFromConfiguration(enum sa868_instruction_set instruction){
  int size = 0;
  switch(instruction) {
    case (CONNECT):
    case (RSSI):
      return command_formats[instruction];
    case (SWEEP):
      size = snprintf(NULL, 0, command_formats[instruction],
          config.sweep_freq_mhz, config.sweep_freq_khz);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        return NULL;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          config.sweep_freq_mhz, config.sweep_freq_khz);
      break;
    case (SETGROUP):
      size = snprintf(NULL, 0, command_formats[instruction],
          config.bandwidth_wide_fm,
          config.tx_freq_mhz, config.tx_freq_khz,
          config.rx_freq_mhz, config.rx_freq_khz,
          config.tx_subaudio,
          config.squelch,
          config.rx_subaudio);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        return NULL;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          config.bandwidth_wide_fm,
          config.tx_freq_mhz, config.tx_freq_khz,
          config.rx_freq_mhz, config.rx_freq_khz,
          config.tx_subaudio,
          config.squelch,
          config.rx_subaudio);
      break;
    case (SETVOLUME):
      size = snprintf(NULL, 0, command_formats[instruction],
          config.volume_level);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        return NULL;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          config.volume_level);
      break;
    case (SETFILTER):
      size = snprintf(NULL, 0, command_formats[instruction],
          config.emphasis_bypass,
          config.voice_highpass_bypass,
          config.voice_lowpass_bypass);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        return NULL;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          config.emphasis_bypass,
          config.voice_highpass_bypass,
          config.voice_lowpass_bypass);
      break;
  }
  #ifdef DEBUG
  Serial.printf("Generated command: %s \n", command_buffer);
  #endif
  return command_buffer;
}
/**
 * SA868 Communication Handler
 * @param response char buffer for module to terminal transmission
 * @param command char buffer for terminal to module transmission 
 * @returns the module response value to buffered command
 */
int sa868_communication_handler(enum sa868_instruction_set instruction) {
  // instruction and response value begins invalid
  int val = UNKNOWN;
  char *command = getCommandFromConfiguration(instruction);
  char *response = (char *)malloc(MAX_BUFFER_SIZE);
  memset(response, 0, MAX_BUFFER_SIZE);
  // using Serial1 (UART1) for SA868 communication
  int rxFIFOcount, txFIFOcount;
  txFIFOcount = Serial1.write(command, strlen(command));
  if (txFIFOcount > 0) {
    #ifdef DEBUG
    Serial.printf("Sent command over UART of size (%d) bytes: %s\n", txFIFOcount, command);
    #endif
    vTaskDelay(100);
    int availableBytes = Serial1.available();
    if (availableBytes > 0) {
      rxFIFOcount = Serial1.readBytesUntil('\n', response, MAX_BUFFER_SIZE - 1);
      #ifdef DEBUG
      if (rxFIFOcount > 0) {
        Serial.printf("Received response over UART of size (%d) bytes: %s\n", rxFIFOcount, response);
      }
      #endif
    }
  }
  // extract instruction return value from response
  int returnCount = sscanf(response, response_formats[instruction], &val);
  free(response);
  return val;
}

int sa868_init(sa868_config_t &config) {
  // Configure a temporary buffer for the response
  int val = -1;
  for (int attempt = 0; attempt < config.MAX_CONNECTION_ATTEMPTS; attempt++) {
    val = sa868_communication_handler(CONNECT);
    // continue until return value is zero
    if (val == 0) {
      break;
    }
    // if the handshake command does not receive the module response within N attempts,
    if (attempt >= (config.MAX_CONNECTION_ATTEMPTS - 1)) {
      // the terminal should restart the module.
      #ifdef DEBUG
      Serial.printf("Handshake failed after %d attempts with return value: %d\r\n", attempt + 1, val);
      #endif
    }
  }
  val = DMOERROR;
  int setterQueue[3] = {SETGROUP, SETVOLUME, SETFILTER};
  for (int i = 0; i < 3; i++) {
    enum sa868_instruction_set instruction = (enum sa868_instruction_set)setterQueue[i];
    val = sa868_communication_handler(instruction);
    if (val == DMOERROR) {
      #ifdef DEBUG
      Serial.printf("Could not handle terminal to module command: %s\n", getCommandFromConfiguration(instruction));
      #endif
      continue;
    }
  }
  return val;
}

void setup() {
  Serial.begin(115200);
  /**
   * UART Interface Format:
   * Baud rate = 9600 Baud
   * Data bit = 8 bit
   * Parity = None
   * Stop bit = 1 bit
   * Terminal Rx,Tx pins = GPIO9,GPIO8 @ 3.3V
   */
  Serial1.begin(9600, SERIAL_8N1, 9, 8);

  //config.MAX_CONNECTION_ATTEMPTS = 3;
  config.bandwidth_wide_fm = 1; // BW = 25 kHz
  config.squelch = 3;
  config.volume_level = 3;

  int repeater_offset = 5; // +5 MHz
  config.rx_subaudio = "0015"; // CTCSS = 110.9 Hz
  config.rx_freq_mhz = 443; 
  config.rx_freq_khz = 3750;
  config.tx_freq_mhz = config.rx_freq_mhz + repeater_offset; // 448 
  config.tx_freq_khz = 3750; 

  xTaskCreate(task, "uart_task", TASK_STACK_SIZE, NULL, 10, NULL);
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

static void task(void *arg) {
  int val = sa868_init(config);
  if (val == 0) {
    #ifdef DEBUG
    Serial.printf("Connection successful, handshake return value: %d\r\n", val);
    #endif
    while (1) {
      val = sa868_communication_handler(RSSI);
      if (val >= 0) {
        Serial.printf("RSSI on %.3d.%4.0d MHz : %.3d dB\n", config.rx_freq_mhz, config.rx_freq_khz, val);
      }
      vTaskDelay(1000);
    }
  } else {
    #ifdef DEBUG
    Serial.println("Module should be restarted.\n");
    #endif
    // TODO: find out how to 'restart' the SA868 module.
    vTaskDelay(10000);
  }
}