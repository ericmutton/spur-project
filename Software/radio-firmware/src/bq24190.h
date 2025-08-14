#ifndef BQ24190_H
#define BQ24190_H

#include <stdint.h>
#include <stdio.h>
// From Arduino Core for ESP32
#include "HardwareSerial.h"
#include "Wire.h"

/** Number of bits in a long int. */
#define BITS_PER_LONG	(__CHAR_BIT__ * __SIZEOF_LONG__)
/**
 * @brief Create a contiguous bitmask starting at bit position @p l
 *        and ending at position @p h.
 */
#define GENMASK(h, l) \
	(((~0UL) - (1UL << (l)) + 1) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

typedef enum {
  REG00 = 0,  // Input Source Control Register
    #define EN_HIZ BIT(7)        // Input Enable
    #define VINDPM GENMASK(6, 3) // Input Voltage Limit
    #define IINLIM GENMASK(2, 0) // Input Current Limit
  REG01,      // Power-On Configuration Register
    #define REGISTER_RESET BIT(7)
    #define WATCHDOG_RESET BIT(6)
    #define CHG_CONFIG GENMASK(5, 4) // Charger Configuration
    #define SYS_MIN GENMASK(3, 1)    // Minimum System Voltage Limit
    #define BOOST_LIM BIT(0)         // Boost Mode Current Limit
  REG02,      // Charge Current Control Register
    #define ICHG GENMASK(7, 2) // Fast Charge Current Limit
    #define FORCE_20PCT BIT(0) // 100% or 20% Current Limit
  REG03,      // Pre-Charge/Termination Current Control Register
    #define IPRECHG GENMASK(7, 4) // Pre-Charge Current Limit
    #define ITERM GENMASK(3, 0)   // Termination Current Limit
  REG04,      // Charge Voltage Control Register
    #define VREG GENMASK(7, 2) // Charge Voltage Limit
    #define BATLOWV BIT(1)     // Battery Precharge to Fast Charge Threshold
    #define VRECHG BIT(0)      // Battery Recharge Threshold (below battery regulation voltage)
  REG05,      // Charge Termination/Timer Control Register
    #define EN_TERM BIT(7)          // Charging Termination Enable
    #define TERM_STAT BIT(6)        // Termination Indicator Threshold
    #define WATCHDOG GENMASK(5, 4)  // I2C Watchdog Timer Setting
    #define EN_TIMER BIT(3)         // Charging Safety Timer Enable
    #define CHG_TIMER GENMASK(2, 1) // Fast Charge Timer Setting
    #define JEITA_ISET BIT(0)       // JEITA Low Temperature Current Setting
  REG06,      // IR Compensation / Thermal Regulation Control Register
    #define BAT_COMP GENMASK(7, 5) // IR Compensation Resistor Setting
    #define VCLAMP GENMASK(4, 2)   // IR Compensation Voltage Clamp (above regulation voltage)
    #define TREG GENMASK(1, 0)     // Thermal Regulation Threshold
  REG07,      // Misc Operation Control Register
    #define DPDM_EN BIT(7)             // Set default input current limit from PSEL/OTG pins
    #define TMR2X_EN BIT(6)            // Safety Timer Setting during Input DPM and Thermal Regulation
    #define BATFET_DISABLE BIT(5)      // Force BATFET Off
    #define JEITA_VSET BIT(4)          // VREG or VREG_200mV
    #define INT_MASK_CHRG_FAULT BIT(1) // Interrupt on Charge Fault
    #define INT_MASK_BAT_FAULT BIT(0)  // Interrupt on Battery Fault
  REG08,      // System Status Register
    #define VBUS_STAT GENMASK(7, 6) // Unknown, USB host, Adapter port, OTG
    #define CHRG_STAT GENMASK(5, 4) // Not charging, Pre-charge, Fast-charge, Charge-termination done
    #define DPM_STAT BIT(3)         // VINDPM or IINDPM
    #define PG_STAT BIT(2)          // Power Good
    #define THERM_STAT BIT(1)       // In Thermal Regulation
    #define VSYS_STAT BIT(0)        // In VSYSMIN Regulation
  REG09,      // Fault Register
    #define WATCHDOG_FAULT BIT(7)    //
    #define BOOST_FAULT BIT(6)       //
    #define CHRG_FAULT GENMASK(5, 4) //
    #define BAT_FAULT BIT(3)         //
    #define NTC_FAULT GENMASK(2, 0)  // 
  REG0A       // Vender / Part / Revision Status Register
    #define PN GENMASK(5, 3)      // 101 – Part Number
    #define TS_PROFILE BIT(2)     // 1 – JEITA profile
    #define DEV_REG GENMASK(1, 0) // 11 – Device Configuration?
} bq24190_register_map_t;

typedef enum {
  DISABLE = 0,
  TIMER_40s,
  TIMER_80s,
  TIMER_160s
} timer_setting_t;

typedef struct {
  TwoWire *i2c;
  uint8_t sensor_address = 0x6B;
  uint8_t pin_interrupt;

  timer_setting_t i2c_watchdog_timer = DISABLE;
  // Input Source Control
  uint16_t input_source_voltage_max_millivolt = 4360;
  uint16_t input_source_current_max_milliamp = 100; // Actual input current limit is the lower of I2C and ILIM
  // Power-On Configuration

  // Charge Current Control 
  uint16_t fast_charge_current_max_milliamp = 2048; // Fast Charge Current Limit (mA)
  bool charge_current_force_20_percent = false;
  // Pre-Charge/Termination Current Control
  uint16_t pre_charge_current_max_milliamp = 256;  // Pre-Charge Current Limit (mA)
  uint16_t termination_current_max_milliamp = 256; // Termination Current Limit (mA)
  uint16_t charge_voltage_max_millivolt = 4208;    // Charge Voltage Limit (mV)
  
  bool jeita_warm_temperature_voltage_200_millivolt_offset = false;
  bool jeita_low_temperature_current_20_percent = false;

  uint8_t fault_sources = INT_MASK_CHRG_FAULT | INT_MASK_BAT_FAULT;
  uint8_t fault_status;
  // 
  uint8_t charger_fault;
  uint8_t ntc_fault;
  uint8_t system_status;
  // status strings
  char *vbus_status;
  char *charge_status;
  //OTG
  bool OTG = false;
} bq24190_config_t;
/**
 * Once the input source passes all the conditions above,
 * the status register REG08[2] goes high and the PG pin
 * goes low. An INT is asserted to the host.
 */
int bq24190_init(bq24190_config_t &config);

int bq24190_readRegister(char *access_buffer, bq24190_register_map_t reg);

int bq24190_readFaultRegister(char *access_buffer);

int bq24190_writeRegister(char *access_buffer, bq24190_register_map_t reg, uint8_t value);

int bq24190_modifyRegister(bq24190_register_map_t reg, uint8_t field, uint8_t value);

/**
 * Autonomous Charging Cycle
 * 
 * With battery charging enabled at POR (REG01[5:4] = 01),
 * bq24195 can complete a charging cycle outside host mode.
 * 
 * Table 3. Charging Parameter Default Settings
 * Charging voltage    4.208 V
 * Charging current    2.048 A
 * Pre-charge current  256 mA
 * Termination current 256 mA
 * Temperature profile JEITA
 * Safety timer        8 hours
 */
bool bq24190_maintainHostMode();

bool bq24190_enableCharging(bool OTG);

#endif // BQ24190_H