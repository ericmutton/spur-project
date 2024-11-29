#include "sa868.h"

#define MAX_BUFFER_SIZE (50) // ESP32 UART FIFO size is max 128 bytes

// #define DEBUG_SA868
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
  "+DMOERROR\r\n"
};

/**
 * @brief this getter matches a substring of a command with an instruction.  
 * @param command cstring to associate with an instruction
 * @returns the associated enum sa868_instruction_set_t value
 */
// enum sa868_instruction_set_t getInstructionFromCommand(char *command) {
//   char *command_instruction;
//   int diff;
//   for (int instruction = CONNECT; instruction < INSTRUCTION_COUNT; instruction++) {
//     command_instruction = command_instructions[instruction];
//     diff = strncmp(command, command_instruction, strlen(command_instruction));
//     if (diff == 0) {
//       // Instruction was found in command
//       return (sa868_instruction_set_t)instruction;  // Return the corresponding enum value
//     }
//   }
//   return (sa868_instruction_set_t)(UNKNOWN);  // Return unknown instruction if not found
// }

char sa868_ctdss_code[100][5] = {
	"023I", "025I", "026I", "031I", "032I", "043I", "047I", "051I", "054I", "065I", 
	"071I", "072I", "073I", "074I", "114I", "115I", "116I", "125I", "131I", "132I", 
	"134I", "143I", "152I", "155I", "156I", "162I", "165I", "172I", "174I", "205I",
	"223I", "226I", "243I", "244I", "245I", "251I", "261I", "263I", "265I", "271I",
	"306I", "311I", "315I", "331I", "343I", "346I", "351I", "364I", "365I", "371I", 
	"411I", "412I", "413I", "423I", "431I", "432I", "445I", "464I", "465I", "466I",
	"503I", "506I", "516I", "532I", "546I", "565I", "606I", "612I", "624I", "627I", 
	"631I", "632I", "654I", "662I", "664I", "703I", "712I", "723I", "731I", "732I", 
	"734I", "743I", "754I"
};

uint32_t sa868_ctdss_pattern[100] = {
	0x640E37, 0x540F6B, 0x340DD3, 0x4C0FC5, 0x2C0D7D, 0x620B6D, 0x720DF8, 0x4A0A9F,
	0x1A097B, 0x560C5D, 0x4E0CF3, 0x2E0E4B, 0x6E0B3A, 0x1E0F17, 0x190BD6, 0x590EA7, 
	0x390C1F, 0x550EF0, 0x4D0E5E, 0x2D0CE6, 0x1D0DBA, 0x630AF6, 0x2B09BC, 0x5B0D91, 
	0x3B0F29, 0x2709EB, 0x570DC6, 0x2F0FD0, 0x1F0E8C, 0x508CBB, 0x648B8B, 0x34886F, 
	0x628ED1, 0x128AFC, 0x528F8D, 0x4A8F23, 0x468F74, 0x6688BD, 0x5689E1, 0x4E894F, 
	0x318F98, 0x498D8E, 0x598B1B, 0x4D8BE2, 0x638F4A, 0x338CAE, 0x4B8EB8, 0x178D0B, 
	0x57887A, 0x4F88D4, 0x484B77, 0x2849CF, 0x684CBE, 0x644CE9, 0x4C4D1B, 0x2C4FA3, 
	0x5248EF, 0x164BF2, 0x564E83, 0x364C3B, 0x614B1E, 0x3148FA, 0x394EC1, 0x2D4E38, 
	0x334BCC, 0x574F18, 0x30CCDD, 0x28CC73, 0x14CD78, 0x74CFC0, 0x4CC8A7, 0x2CCA1F, 
	0x1ACE19, 0x26CF12, 0x16CE4E, 0x61CEA2, 0x29CDE8, 0x65C8CE, 0x4DC93C, 0x2DCB84, 
	0x1DCAD8, 0x63CD94, 0x1BCF82
};

extern sa868_config_t sa868;

char command_buffer[MAX_BUFFER_SIZE];
/**
 * @brief this getter reads relevant configuration data into a command buffer.
 * @param instruction format to use for generating the terminal-to-module command 
 * @returns the generated command buffer
 */
char *sa868_generateCommand(sa868_instruction_set_t instruction){
  int size = 0;
  switch(instruction) {
    case (CONNECT):
    case (RSSI):
      strcpy(command_buffer, command_formats[instruction]);
      break;
    case (SWEEP):
      size = snprintf(NULL, 0, command_formats[instruction],
          sa868.sweep_freq_mhz, sa868.sweep_freq_khz);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        break;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          sa868.sweep_freq_mhz, sa868.sweep_freq_khz);
      break;
    case (SETGROUP):
      size = snprintf(NULL, 0, command_formats[instruction],
          sa868.bandwidth_wide_fm,
          sa868.tx_freq_mhz, sa868.tx_freq_khz,
          sa868.rx_freq_mhz, sa868.rx_freq_khz,
          sa868.tx_subaudio,
          sa868.squelch,
          sa868.rx_subaudio);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        break;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          sa868.bandwidth_wide_fm,
          sa868.tx_freq_mhz, sa868.tx_freq_khz,
          sa868.rx_freq_mhz, sa868.rx_freq_khz,
          sa868.tx_subaudio,
          sa868.squelch,
          sa868.rx_subaudio);
      break;
    case (SETVOLUME):
      size = snprintf(NULL, 0, command_formats[instruction],
          sa868.volume_level);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        break;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          sa868.volume_level);
      break;
    case (SETFILTER):
      size = snprintf(NULL, 0, command_formats[instruction],
          sa868.emphasis_bypass,
          sa868.voice_highpass_bypass,
          sa868.voice_lowpass_bypass);
      if (size >= MAX_BUFFER_SIZE) {
        // Handle error: command too long
        break;
      }
      snprintf(command_buffer, size + 1, command_formats[instruction],
          sa868.emphasis_bypass,
          sa868.voice_highpass_bypass,
          sa868.voice_lowpass_bypass);
      break;
  }
  #ifdef DEBUG_SA868
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
int sa868_communication_handler(sa868_instruction_set_t instruction) {
  // instruction and response value begins invalid
  int val;
  char *command = sa868_generateCommand(instruction);
  char *response = (char *)malloc(MAX_BUFFER_SIZE);
  memset(response, 0, MAX_BUFFER_SIZE);
  HardwareSerial *uart = sa868.UART;
  int rxFIFOcount, txFIFOcount;
  txFIFOcount = uart->write(command, strlen(command));
  if (txFIFOcount > 0) {
    #ifdef DEBUG_SA868
    Serial.printf("Sent command over UART of size (%d) bytes: %s\n", txFIFOcount, command);
    #endif
    //uart.flush();
    vTaskDelay(800);
    int availableBytes = uart->available();
    if (availableBytes > 0) {
      rxFIFOcount = uart->readBytesUntil('\n', response, MAX_BUFFER_SIZE - 1);
      #ifdef DEBUG_SA868
      if (rxFIFOcount > 0) {
        Serial.printf("Received response over UART of size (%d) bytes: %s\n", rxFIFOcount, response);
      }
      #endif
    }
  }
  if (response == response_formats[DMOERROR]) {
    val = DMOERROR;
  } else {
    // extract instruction return value from response
    int returnCount = sscanf(response, response_formats[instruction], &val);
  }
  free(response);
  return val;
}

int sa868_init(sa868_config_t &configuration) {
  sa868 = configuration;
  // Configure a temporary buffer for the response
  int val = DMOERROR;
  for (int attempt = 0; attempt < sa868.MAX_CONNECTION_ATTEMPTS; attempt++) {
    val = sa868_communication_handler(CONNECT);
    // continue until return value is zero
    if (val == 0) {
      break;
    }
    // if the handshake command does not receive the module response within N attempts,
    if (attempt >= (sa868.MAX_CONNECTION_ATTEMPTS - 1)) {
      // the terminal should restart the module.
      #ifdef DEBUG_SA868
      Serial.printf("Handshake failed after %d attempts with return value: %d\r\n", attempt + 1, val);
      #endif
    }
  }
  val = DMOERROR;
  sa868_instruction_set_t setterQueue[3] = {SETGROUP, SETVOLUME, SETFILTER};
  sa868_instruction_set_t instruction;
  for (int i = 0; i < 3; i++) {
    instruction = setterQueue[i];
    val = sa868_communication_handler(instruction);
    if (val == DMOERROR) {
      #ifdef DEBUG_SA868
      Serial.printf("Could not handle terminal to module command: %s\n", sa868_generateCommand(instruction));
      #endif
      continue;
    }
  }
  return val;
}