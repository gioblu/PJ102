
// EEPROM
#include <EEPROM.h>
// Power management, sleep and watchdog functions
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
// i2c
#include <TinyWireM.h>

#define HDC1080_ADDRESS         0x40
#define HDC1080_TEMPERATURE     0x00
#define HDC1080_HUMIDITY        0x01
#define HDC1080_CONFIGURATION   0x02
#define HDC1080_MANUFACTURER_ID 0xFE
#define HDC1080_DEVICE_ID       0xFF
#define HDC1080_SERIAL_ID_FIRST 0xFB
#define HDC1080_SERIAL_ID_MID   0xFC
#define HDC1080_SERIAL_ID_LAST  0xFD

// PJON configuration
#define PJON_MAX_PACKETS           0 // No internal buffer (less memory used)
#define PJON_PACKET_MAX_LENGTH    20
#define SWBB_READ_DELAY            6 // SoftwareBitBang timing configuration
// PJON instantiation
#include <PJONSoftwareBitBang.h>
PJONSoftwareBitBang bus;

// PJ102 configuration
#define MODULE_VERSION             2
#define MODULE_ACCEPT_CONFIG    true
#define MODULE_SAMPLE_RATE      2000 // Sample rate
#define MODULE_CALIBRATION       -40 // Self-heating is approximately 4 deg

uint8_t recipient_id;
bool accept_config_change;
uint32_t last_sample;
int8_t calibration;

// Data structure
struct sensor_record {float h; float t;};
sensor_record record;
sensor_record buffer;

double readTemperature() {
    uint8_t msb;
    uint8_t lsb;
    uint16_t rawVal;
    TinyWireM.beginTransmission(HDC1080_ADDRESS);
    TinyWireM.send(HDC1080_TEMPERATURE);
    TinyWireM.endTransmission();
    delay(15);
    TinyWireM.requestFrom(HDC1080_ADDRESS,2);
    msb = TinyWireM.receive();
    lsb = TinyWireM.receive();
    rawVal = (uint16_t)((msb << 8) | lsb);
    return (rawVal / 65536.0) * 165.0 - 40.0;
}

double readHumidity() {
    uint8_t msb;
    uint8_t lsb;
    uint16_t rawVal;
    TinyWireM.beginTransmission(HDC1080_ADDRESS);
    TinyWireM.send(HDC1080_HUMIDITY);
    TinyWireM.endTransmission();
    delay(10);
    TinyWireM.requestFrom(HDC1080_ADDRESS, 2);
    msb = TinyWireM.receive();
    lsb = TinyWireM.receive();
    rawVal = (uint16_t)((msb << 8) | lsb);
  return (rawVal / 65536.0) * 100.0;
}

void setup() {
  power_adc_disable(); // If the ADC is not required ~320uA are spared
  TinyWireM.begin();
  // Writing default configuration in EEPROM
  if(
    EEPROM.read(5) != 'P' ||
    EEPROM.read(6) != 'J' ||
    EEPROM.read(7) != '1' ||
    EEPROM.read(8) != '0' ||
    EEPROM.read(9) != '2' ||
    EEPROM.read(10) != MODULE_VERSION
  ) EEPROM_write_default_configuration();
  EEPROM_read_configuration();
  // Use pin 1 for PJON communicaton
  bus.strategy.set_pin(1);
  // Begin PJON communication
  bus.begin();
  // Register the receiver callback called when a packet is received
  bus.set_receiver(receiver_function);
}

void EEPROM_read_configuration() {
  bus.set_id(EEPROM.read(0));
  recipient_id = EEPROM.read(1);
  calibration = EEPROM.read(2);
  accept_config_change = EEPROM.read(11);
};

void EEPROM_write_default_configuration() {
  // PJ102 ID
  EEPROM.update(0, PJON_NOT_ASSIGNED);
  // Recipient ID
  EEPROM.update(1, PJON_MASTER_ID);
  // Calibration
  EEPROM.update(2, MODULE_CALIBRATION);
  // Module name
  EEPROM.update(5, 'P');
  EEPROM.update(6, 'J');
  EEPROM.update(7, '1');
  EEPROM.update(8, '0');
  EEPROM.update(9, '2');
  EEPROM.update(10, MODULE_VERSION);
  // Accept incoming configuration
  EEPROM.update(11, MODULE_ACCEPT_CONFIG);
};

void loop() {
  bus.receive(1000);
}

void receiver_function(uint8_t *payload, uint16_t length, const PJON_Packet_Info &info) {
  bool is_master =
    (info.tx.id == PJON_MASTER_ID) || (info.tx.id == recipient_id);

  // Info request
  if(payload[0] == '?') {
    uint8_t module_name[6] = {
      EEPROM.read(5),
      EEPROM.read(6),
      EEPROM.read(7),
      EEPROM.read(8),
      EEPROM.read(9),
      EEPROM.read(10)
    };
    bus.send_packet(recipient_id, module_name, 6);
  }

  // Sleep and then transmit
  if(is_master && (payload[0] == 'T')) {
    sleep_for(payload[1] << 8 | payload[2] & 0xFF);
  }

  // Read DHT sensor
  if(is_master && ((payload[0] == 'S') || (payload[0] == 'E') || (payload[0] == 'T'))) {
    if((millis() - last_sample) > MODULE_SAMPLE_RATE) {
      buffer.h = (float)readHumidity();
      buffer.t = (float)readTemperature() + ((float)calibration / 10.0);
      if(!isnan(buffer.h) && !isnan(buffer.t))
        record = buffer;
      last_sample = millis();
    }
  }
  // Transmit latest sample
  if(is_master && ((payload[0] == 'G') || (payload[0] == 'E') || (payload[0] == 'T')))
    bus.send_packet(recipient_id, &record, sizeof(record));

  // Return if configuration is blocked
  if(!accept_config_change) return;

  if(is_master && (payload[0] == 'C')) {
    calibration = (int8_t)payload[1];
    EEPROM.update(2, payload[1]);
  }

  // DEVICE ID UPDATE
  if(is_master && (payload[0] == 'I')) {
    bus.set_id(payload[1]);
    EEPROM.update(0, payload[1]);
  }
  // RECIPIENT ID UPDATE
  if(is_master && (payload[0] == 'R')) {
    recipient_id = payload[1];
    EEPROM.update(1, recipient_id);
  }
  // DANGER ZONE
  // Attention when X is received configuration is set to default
  if(is_master && (payload[0] == 'X')) {
    EEPROM_write_default_configuration();
    EEPROM_read_configuration();
  }
  // Attention when Q is received the module will stop to accept commands
  if(is_master && (payload[0] == 'Q')) {
    accept_config_change = false;
    EEPROM.update(11, 0);
  }
};

void sleep_for(int32_t t) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  while(t > 0) {
    if(t >= 8000) {
      setup_watchdog(9);
      sleep_mode();
      t -= 8000;
    } else if(t >= 4000) {
      setup_watchdog(8);
      sleep_mode();
      t -= 4000;
    } else if(t >= 2000) {
      setup_watchdog(7);
      sleep_mode();
      t -= 2000;
    } else if(t >= 1000) {
      setup_watchdog(6);
      sleep_mode();
      t -= 1000;
    } else if(t >= 500) {
      setup_watchdog(5);
      sleep_mode();
      t -= 500;
    } else if(t >= 250) {
      setup_watchdog(4);
      sleep_mode();
      t -= 250;
    } else if(t >= 128) {
      setup_watchdog(3);
      sleep_mode();
      t -= 128;
    } else if(t >= 64) {
      setup_watchdog(2);
      sleep_mode();
      t -= 64;
    } else if(t >= 32) {
      setup_watchdog(1);
      sleep_mode();
      t -= 32;
    } else if(t >= 16) {
      setup_watchdog(0);
      sleep_mode();
      t -= 16;
    } else {
      delay(t);
      break;
    }
  }
};

// Sets the watchdog timer to wake us up, but not reset
// 0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
// 6=1sec, 7=2sec, 8=4sec, 9=8sec
// http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {
  if(timerPrescaler > 9)
    timerPrescaler = 9; // Limit incoming amount to legal settings

  byte bb = timerPrescaler & 7;
  if(timerPrescaler > 7)
    bb |= (1<<5); // Set the special 5th bit if necessary

  // This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); // Clear the watch dog reset
  WDTCR |= (1<<WDCE) | (1<<WDE); // Set WD_change enable, set WD enable
  WDTCR = bb; // Set new watchdog timeout value
  WDTCR |= _BV(WDIE); // Interrupt enable, avoid unit resetting after each int
};

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  // NOTHING HERE BY DEFAULT
};
