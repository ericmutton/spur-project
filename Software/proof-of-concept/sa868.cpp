#include "sa868.h"

#define MAX_BUFFER_SIZE (50) // ESP32 UART FIFO size is max 128 bytes

/** 
 * SA868 Instruction Set (<CR><LF>)
 */
char *command_formats[] = {
  "AT+DMOCONNECT\r\n",
  /**
   * Sweep_freq
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
  "AT+RSSI?\r\n",
  /**
   * X=1: emphasis,voice_highpass,voice_lowpass bypass
   * X=0: emphasis,voice_highpass,voice_lowpass normal
   */
  "AT+SETFILTER=%i,%i,%i\r\n"
};
char *command_instructions[] = {
  "AT+DMOCONNECT",
  "S+",
  "AT+DMOSETGROUP",
  "AT+DMOSETVOLUME",
  "AT+RSSI?",
  "AT+SETFILTER"
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

char command_buffer[MAX_BUFFER_SIZE];
/**
 * @brief this getter reads relevant configuration data into a command buffer.
 * @param config relevant module configuration data
 * @param instruction format to use for generating the terminal-to-module command 
 * @returns the generated command buffer
 */
char *sa868_generateCommand(sa868_config_t &config, enum sa868_instruction_set instruction){
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
int sa868_communication_handler(sa868_config_t &config, enum sa868_instruction_set instruction) {
  // instruction and response value begins invalid
  int val = UNKNOWN;
  char *command = sa868_generateCommand(config, instruction);
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
  int val = UNKNOWN;
  for (int attempt = 0; attempt < config.MAX_CONNECTION_ATTEMPTS; attempt++) {
    val = sa868_communication_handler(config, CONNECT);
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
    val = sa868_communication_handler(config, instruction);
    if (val == DMOERROR) {
      #ifdef DEBUG
      Serial.printf("Could not handle terminal to module command: %s\n", sa868_generateCommand(config, instruction));
      #endif
      continue;
    }
  }
  return val;
}