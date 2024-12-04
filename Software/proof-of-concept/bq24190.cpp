#include "bq24190.h"

// https://www.ti.com/lit/an/slva704/slva704.pdf

// The I2C interface supports both
// standard mode (up to 100 kbits),
// and fast mode (up to 400 kbits).

#define DEBUG_BQ24190 Serial

extern bq24190_config_t bq24190;
char access_buffer[sizeof(uint8_t)];

// Table 15. REG08 System Status Register Description
const char* vbus_statuses[4] {
  "VBUS is Unknown (no input, or DPDM detection incomplete), ",
  "VBUS is USB host, ",
  "VBUS is Adapter port, ",
  "VBUS is OTG, "
};
const char* charger_statuses[4] {
  "Not Charging, ",
  "Pre-charge (<VBATLOWV), ",
  "Fast Charging, ",
  "Charge Termination Done, "
};
// Table 16. REG09 Fault Register Description
const char* charger_faults[4] {
  "Normal, ",
  "Input Fault (VBUS OVP or VBAT < VBUS < 3.8 V), ",
  "Thermal shutdown, ",
  "Charge Safety Timer Expiration, "
};
const char* ntc_faults[5] {
  "Normal (T2-T3), ",
  "Warm (T3-T5), ",
  "Cool (T1-T2), ",
  "Cold (< T1), ",
  "Hot (> T5), "
};

int bq24190_init(bq24190_config_t &config) {
  bq24190 = config;
  bq24190_modifyRegister(REG05, WATCHDOG, bq24190.i2c_watchdog_timer << 4);
  
  bq24190_modifyRegister(REG03, IPRECHG, (bq24190.pre_charge_current_max_milliamp - 128) << 3); 
  bq24190_modifyRegister(REG03, ITERM, (bq24190.termination_current_max_milliamp - 128) >> 3);

  bq24190_modifyRegister(REG04, VREG, (bq24190.charge_voltage_max_millivolt - 3504) >> 2);
  
  // Fast Charge current 
  bq24190_modifyRegister(REG02, ICHG, (bq24190.fast_charge_current_max_milliamp - 512) >> 6);
  // JEITA Low Temperature (0°C-10°C) Current Setting (50% or 20% ICHG)
  bq24190_modifyRegister(REG05, JEITA_ISET, bq24190.jeita_low_temperature_current_20_percent); 
  // JEITA Warm Temperature (45°C-60°C) Voltage Setting (VREG or VREG-200mV)
  bq24190_modifyRegister(REG07, JEITA_VSET, bq24190.jeita_warm_temperature_voltage_200_millivolt_offset);
  // Interrupt to Host (INT) Sources
  bq24190_modifyRegister(REG07, INT_MASK_CHRG_FAULT | INT_MASK_BAT_FAULT, bq24190.fault_sources);

  /**
   * Input Source Qualification
   * 1. VBUS voltage below 18 V (not in ACOV)
   * 2. VBUS voltage above 3.8 V when pulling 30 mA (poor source detection)
   * Once the input source passes all the conditions above,
   * the status register REG08[2] goes high and the PG pin
   * goes low. An INT is asserted to the host.
   *
   * If the device fails the poor source detection,
   * it will repeat the detection every 2 seconds.
   */
  uint8_t bytes_transferred = bq24190_readRegister(access_buffer, REG08);
  uint8_t vbus_status = ((access_buffer[0] & VBUS_STAT) >> 6); 
  uint8_t charge_status = ((access_buffer[0] & CHRG_STAT) >> 4);
  if (bytes_transferred > 0 && (access_buffer[0] & PG_STAT) >> 2) {
    // Input Source is qualified, set host dynamic power management
    bq24190_modifyRegister(REG00, VINDPM, (bq24190.input_source_voltage_max_millivolt - 3880) >> 2);
    bq24190_modifyRegister(REG00, IINLIM, (bq24190.input_source_current_max_milliamp - 100));
    // DPM mode?
    if (charge_status == 0) {
      // not currently charging
      bq24190_enableCharging(false);
      if (!(digitalRead(bq24190.pin_interrupt)) == 0) {
        #ifdef DEBUG_BQ24190
        Serial.printf("Interrupt Detected!!");
        #endif
      }
    }
  } else {
    // With an unqualified source, reconfigure watchdog timer.
    bq24190_modifyRegister(REG05, WATCHDOG, bq24190.i2c_watchdog_timer);
  }
  #ifdef DEBUG_BQ24190
  bq24190_readRegister(access_buffer, REG0A);
  Serial.printf("Part Number: %#x\n", access_buffer[0] & PN);
  #endif
  return 0;
}

int bq24190_readRegister(char *access_buffer, bq24190_register_map_t reg) {
  uint8_t bytes_transferred = 0;
  TwoWire *i2c = bq24190.i2c;
  i2c->beginTransmission(bq24190.sensor_address); // transmit sensor address
  i2c->write(reg);                  // transmit register address
  if (!(i2c->endTransmission(true))) {
    bytes_transferred = i2c->requestFrom(bq24190.sensor_address, sizeof(uint8_t), true);
    if (bytes_transferred > 0 && bytes_transferred == 1) {
      i2c->readBytes(access_buffer, bytes_transferred);
    }
  }
  return bytes_transferred;
}
/**
 *
When a fault occurs, the charger device sends out INT
and keeps the fault state in REG09 until the host reads
the fault register.

Before the host reads REG09 and all the faults
are cleared, the charger device would not send
any INT upon new faults.

In order to read the current fault status, the
host has to read REG09 two times consecutively.

The 1st reads fault register status from the last read
and the 2nd reads the current fault register status.
 */
int bq24190_readFaultRegister(char *access_buffer) {
  int bytes_transferred;
  /*
   * The following events will generate 256-us INT pulse.
   * USB/adapter source identified (through PSEL and OTG pins)
   * Good input source detected
   * VBUS - VBAT > VSLEEPZ
   * VBUS > VACOV
   * current limit above IBADSRC
   * Input removed
   * Charge Complete
   * Any FAULT event in REG09
   */
  bytes_transferred = bq24190_readRegister(access_buffer, REG09);
  if (bytes_transferred > 0) {
    // #ifdef DEBUG_BQ24190
    // Serial.printf("Present Faults: %s%s%s%s%s%s\n",
    //   ((access_buffer[0] & WATCHDOG_FAULT) >> 7)      ? "WATCHDOG, " : "",
    //   ((access_buffer[0] & BOOST_FAULT)    >> 6)      ? "BOOST, " : "",
    //   ((access_buffer[0] & CHRG_FAULT)     >> 4)  > 0 ? "CHARGE, " : "",
    //   ((access_buffer[0] & BAT_FAULT)      >> 3)      ? "BATTERY, " : "",
    //   ((access_buffer[0] & NTC_FAULT)          )  > 0 ? "NTC" : "",
    //   ((access_buffer[0])                      ) == 0 ? "NONE" : ""
    // );
    // #endif
  }
  bytes_transferred = bq24190_readRegister(access_buffer, REG09);
  
  uint8_t charger_fault = (access_buffer[0] & CHRG_FAULT) >> 4;
  uint8_t ntc_fault = access_buffer[0] & NTC_FAULT;

  #ifdef DEBUG_BQ24190
  DEBUG_BQ24190.printf("Power Faults: %s%s%s%s%s%s", // Enumerate each fault as presented
    ((access_buffer[0] & WATCHDOG_FAULT) >> 7) ? "WATCHDOG, " : "",
    ((access_buffer[0] & BOOST_FAULT)    >> 6) ? "BOOST, " : "",
    (charger_fault) > 0                        ? charger_faults[charger_fault] : "",
    ((access_buffer[0] & BAT_FAULT)      >> 3) ? "BATTERY OVP, " : "",
    (ntc_fault) > 0                            ? ntc_faults[ntc_fault] : "",
    ((access_buffer[0])                 ) == 0 ? "NONE; " : ""
  );
  if (access_buffer[0] != 0) {
    Serial.println();
  }
  #endif
  return bytes_transferred;
}

int bq24190_writeRegister(char *access_buffer, bq24190_register_map_t reg, uint8_t value) {
  int bytes_transferred = 0;
  // the fault register REG09 does not support
  // write, multi-read, or multi-write access.
  if (reg == REG09) {
    bytes_transferred = bq24190_readFaultRegister(access_buffer);
  } else {
    TwoWire *i2c = bq24190.i2c;
    i2c->beginTransmission(bq24190.sensor_address); // transmit sensor address
    bytes_transferred += i2c->write(reg);           // transmit register address
    bytes_transferred += i2c->write(value);         // transmit write value
    if (!(i2c->endTransmission(true))) {
      bytes_transferred = sizeof(uint8_t);
    }
  }
  return bytes_transferred;
}

int bq24190_modifyRegister(bq24190_register_map_t reg, uint8_t field, uint8_t value) {
  bool val;
  int length = bq24190_readRegister(access_buffer, reg);
  if (length == sizeof(uint8_t)) {
    char original = (uint8_t)access_buffer[0];
    bq24190_writeRegister(access_buffer, reg, (original & ~(field)) | value);
    bq24190_readRegister(access_buffer, reg);
    val = (original != (uint8_t)access_buffer[0]); 
    if (val == true) {
      #ifdef DEBUG_BQ24190
        DEBUG_BQ24190.printf("New REG%.2X contents: %#x\n", reg, access_buffer[0]);
      #endif
    }
  }
  return val;
}
/**
 * To keep the device in host mode, the host has to 
 * reset the watchdog timer by writing 1 to REG01[6]
 * before the watchdog timer expires (REG05[5:4]), or
 * disable watchdog timer by setting REG05[5:4] = 11
 */
bool bq24190_maintainHostMode() {
  bool val = false;
  int bytes_transferred = bq24190_readFaultRegister(access_buffer);
  // if Watchdog timer has not yet expired
  if (bytes_transferred > 0 && !((access_buffer[0] & WATCHDOG_FAULT) >> 7)) {
    // Read current I2C Watchdog Timer Setting
    bytes_transferred = bq24190_readRegister(access_buffer, REG05);
    timer_setting_t timer = (timer_setting_t)((access_buffer[0] & WATCHDOG) >> 4);
    // I2C Watchdog Timer Reset
    bq24190_modifyRegister(REG01, WATCHDOG_RESET, true);

    bytes_transferred = bq24190_readRegister(access_buffer, REG08);
    uint8_t vbus_status = ((access_buffer[0] & VBUS_STAT) >> 6); 
    uint8_t charge_status = ((access_buffer[0] & CHRG_STAT) >> 4);
    #ifdef DEBUG_BQ24190
    if (access_buffer[0] > 0) {
      DEBUG_BQ24190.printf("System Status: %s%s\n%s%s%s%s\n",
        vbus_statuses[vbus_status], //00
        charger_statuses[charge_status], //00
        ((access_buffer[0] & DPM_STAT)   >> 3) ? "VINDPM or IINDPM, " : "Not DPM, ", //0
        ((access_buffer[0] & PG_STAT)    >> 2) ? "Power Good, " : "Not Power Good, ", //0
        ((access_buffer[0] & THERM_STAT) >> 1) ? "In Thermal Regulation, " : "Normal, ", //0
        ((access_buffer[0] & VSYS_STAT)      ) ? "In VSYSMIN regulation (BAT < VSYSMIN)" : "Not in VSYSMIN regulation (BAT > VSYSMIN)"
      );
    } else {
      DEBUG_BQ24190.printf("Nominal Status (%#x)\n", access_buffer[0]);
    }
    #endif
  } else {
    // charger has entered default mode
    bq24190_init(bq24190);
  }
  return val;
}

typedef enum {
  // DISABLE = 0,
  BATTERY = 1,
  // USB OTG 5 V at 1.3 A Synchronous Boost Converter Operation
  OTG_BOOST // Boost requires OTG pin HIGH.
  //, OTG
} charger_config_t;

bool bq24190_enableCharging(bool OTG) { 
  bq24190_modifyRegister(REG05, WATCHDOG, DISABLE);
  charger_config_t charger_conf;
  if (OTG) {
    charger_conf = OTG_BOOST;
  } else {
    charger_conf = BATTERY;
  }
  bq24190_modifyRegister(REG01, CHG_CONFIG, (charger_conf << 4) & CHG_CONFIG); // Enable Battery Charging

  // 8.3.3.5.1 Termination when REG02[0] = 1
  // bq24190_modifyRegister(REG02, FORCE_20PCT, true); // 20% Current Limit


  
  // Charge Current Control

  /*
   * When the battery is charged to fully capacity, the
   * host disables charging through CE pin or REG01[5:4].
   */

  // 8.3.3.5.2 Termination when REG05[6] = 1
  // Charge Termination/Timer Control
  // bq24190_modifyRegister(REG05, TERM_STAT, true); // Termination Indicator Threshold

  /* 
   * The charging cycle is still on-going until the
   * current falls below the termination threshold.
   */
  return true;
}