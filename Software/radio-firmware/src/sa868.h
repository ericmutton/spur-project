#include <stdint.h>
#include <stdio.h>
// From Arduino Core for ESP32
#include "HardwareSerial.h"

typedef struct {
    HardwareSerial *UART;

    uint8_t MAX_CONNECTION_ATTEMPTS = 3;
    uint16_t PTT_TIMEOUT_SECONDS = -1; // default is no timeout
    uint16_t sweep_freq_mhz; uint16_t sweep_freq_khz;

    bool bandwidth_wide_fm = true;
    uint16_t tx_freq_mhz; uint16_t tx_freq_khz;
    uint16_t rx_freq_mhz; uint16_t rx_freq_khz;
    char *tx_subaudio = (char *) "0000";
    uint8_t squelch = 3;
    char *rx_subaudio = (char *) "0000";

    uint8_t volume_level = 3;

    bool emphasis_bypass = false;
    bool voice_highpass_bypass = false;
    bool voice_lowpass_bypass = false;  

    int16_t rssi = 0; // ranges from 0 to 255
    int8_t rssi_offset = -30; // Module reports RSSI value in dB

} sa868_config_t;

typedef enum {
  CONNECT = 0,  // Detect the module is in normal working status
  SWEEP,        // Set the frequency to scan
  SETGROUP,     // Set the working parameters of the module
  SETVOLUME,    // Set the volume level of the module
  RSSI,         // Read Received Signal Strength Indicator value of the module
  SETFILTER,    // Set the filter of the module
  DMOERROR = -1  // Unknown instruction
} sa868_instruction_set_t;

char *sa868_ctcss_subtone(char *ctcss_frequency);

/**
 * @brief this getter reads relevant configuration data into a command buffer.
 * @param instruction format to use for generating the terminal-to-module command 
 * @returns the generated command buffer
 */
char *sa868_generateCommand(sa868_instruction_set_t instruction);

// const int INSTRUCTION_COUNT = sizeof(command_instructions) / sizeof(command_instructions[0]);
/**
 * @brief this getter matches a substring of a command with an instruction.  
 * @param command cstring to associate with an instruction
 * @returns the associated enum sa868_instruction_set_t value
 */
// sa868_instruction_set_t getInstructionFromCommand(char *command);

/**
 * SA868 Communication Handler
 * @param instruction to handle in terminal to module transmission 
 * @returns the module response value to buffered command
 */
int sa868_communication_handler(sa868_instruction_set_t instruction);

/**
 * SA868 Initialization Handler
 * @param config initial module configuration
 * @returns the last staged initialization operation return value
 */
int sa868_init(sa868_config_t &config);

bool updateFrequency(bool tx_freq, char *entry);

char *sa868_s_meter();
