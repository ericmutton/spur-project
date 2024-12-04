#include "pcf8575.h"

// TODO: Phase out adafruit BusIO dependency
#include <Adafruit_PCF8575.h>
Adafruit_PCF8575 pcf;
const pcf8575_address_map_t rows[4] = {PORT17, PORT16, PORT15, PORT14};
const pcf8575_address_map_t cols[4] = {PORT13, PORT12, PORT11, PORT10};

#define DEBUG_PCF8575 Serial

extern pcf8575_config_t pcf8575;
char previous_key_scanned;

bool pcf8575_portMode(uint8_t port, uint8_t mode) {
  return pcf.pinMode(port, mode);
}

int pcf8575_readPort(uint8_t port) {
  return pcf.digitalRead((uint8_t)port);
  // uint8_t bytes_transferred = 0;
  // TwoWire *i2c = pcf8575.i2c;
  // i2c->beginTransmission(pcf8575.sensor_read_address); // transmit sensor address
  // i2c->write(port);                                    // transmit port address
  // if (!(i2c->endTransmission(true))) {
  //   bytes_transferred = i2c->requestFrom(pcf8575.sensor_read_address, sizeof(uint16_t), true);
  //   if (bytes_transferred > 0 && bytes_transferred == sizeof(uint16_t)) {
  //     i2c->readBytes(scan_buffer, bytes_transferred);
  //   }
  // }
  // return bytes_transferred;
}

// int pcf8575_pinMode(uint8_t access_buffer, pcf8575_address_map_t port, uint8_t value) {
//   int bytes_transferred = 0;
//   strcpy(access_buffer, "");
//   TwoWire *i2c = pcf8575.i2c;
//   i2c->beginTransmission(pcf8575.sensor_read_address); // transmit sensor address
//   if ((value == INPUT) || (value == INPUT_PULLUP)) {
//     access_buffer |= (uint8_t *)(1UL << port);
//   } else {
//     access_buffer &= (uint8_t *)(~(1UL << port));
//   }
//   bytes_transferred += i2c->write((uint8_t)&access_buffer); // transmit register address
//   if (!(i2c->endTransmission(true))) {
//     bytes_transferred = sizeof(uint16_t);
//   }
//   return bytes_transferred;
// }

int pcf8575_writePort(uint8_t port, uint8_t value) {
  return pcf.digitalWrite(port, value);
  // int bytes_transferred = 0;
  // TwoWire *i2c = pcf8575.i2c;
  // i2c->beginTransmission(pcf8575.sensor_read_address); // transmit sensor address
  // bytes_transferred += i2c->write(port);               // transmit register address
  // bytes_transferred += i2c->write(value);              // transmit write value
  // if (!(i2c->endTransmission(true))) {
  //   bytes_transferred = sizeof(uint8_t);
  // }
  // return bytes_transferred;
}

// void vProcessInterface( void *pvParameter1, uint32_t ulParameter2 )
// {
//     BaseType_t xInterfaceToService;
//     xInterfaceToService = ( BaseType_t ) ulParameter2;

//     Serial.println("about to start a scan process.") ;
//     /* ...Perform the processing here... */
//     xTaskCreatePinnedToCore(
//     pcf8575_scanKeys
//     ,  "scanKeypad"
//     ,  1024  // Stack size
//     ,  NULL
//     ,  1  // Priority
//     ,  NULL 
//     ,  ARDUINO_RUNNING_CORE);

// }

// static void IRAM_ATTR portChanged(void) {
//   BaseType_t xHigherPriorityTaskWoken;
//   xHigherPriorityTaskWoken = pdFALSE;
//   xTimerPendFunctionCallFromISR( vProcessInterface,
//     NULL,
//     ( uint32_t ) 0,
//     &xHigherPriorityTaskWoken );
//   portYIELD_FROM_ISR();
// }

int pcf8575_init(pcf8575_config_t &configuration) {
  pcf8575 = configuration;
  pcf.begin(pcf8575.sensor_address, pcf8575.i2c);
  pinMode(pcf8575.pin_interrupt, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(pcf8575.pin_interrupt), portChanged, FALLING);
  return 0;
}

char keypad_layout[16] = {
  'A','B','C','D',
  '1','2','3','*',
  '4','5','6','0',
  '7','8','9','#'
};
int pcf8575_scanKeys() {
  int val = -1;
  int row = 0, col = 0;
  for (row = 0; row < 4; row++) {
    pcf8575_writePort(rows[0], HIGH);
    pcf8575_writePort(rows[1], HIGH);
    pcf8575_writePort(rows[2], HIGH);
    pcf8575_writePort(rows[3], HIGH);
    for (col = 0; col < 4; col++) {
      pcf8575_writePort(rows[row], LOW); // Pull-down to scan PORT
      if (pcf8575_readPort(cols[col]) == LOW) {
        #ifdef DEBUG_PCF8575
        DEBUG_PCF8575.printf("Read in pushbutton (%d,%d) via (PORT%.2d,PORT%.2d)\n", row, col, rows[row]+2, cols[col]+2);
        #endif
        val = (row * 4) + col;
        break;
      }
    }
    pcf8575_writePort(rows[row], HIGH); // Pull-up PORTs
  }
  return val;
}

char pcf8575_readKeypad(int &key_ghost_count) {
  int key_index;
  char key_scanned = '\0';
  key_index = pcf8575_scanKeys();
  if (key_index != -1) {
    key_scanned = keypad_layout[key_index];
    if (key_scanned != previous_key_scanned) {
      key_ghost_count = 0;
      previous_key_scanned = key_scanned;
      #ifdef DEBUG_PCF8575
      DEBUG_PCF8575.printf("New Key Press: %c\n", key_scanned);
      #endif
    } else {
      #ifdef DEBUG_PCF8575
      DEBUG_PCF8575.printf("Same Key Pressed (%d) times: %c\n", key_ghost_count+1, key_scanned);
      #endif
      key_ghost_count++;
    }
  }
  return key_scanned;
}