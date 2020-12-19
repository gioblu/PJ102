
// Use sleep to enable low power features
// #include <avr/sleep.h>

#include <DHT.h>

// PJ102 DHT22 temperature and humidity sensor
// SoftwareBitBang timing configuration
#define SWBB_READ_DELAY 6

// P103 software version
#define MODULE_VERSION          1
// P103 by default accepts configuratio change
#define MODULE_ACCEPT_CONFIG true
// P103 sample rate
#define MODULE_SAMPLE_RATE   2000

// PJON configuration
// Do not use internal packet buffer (reduces memory footprint)
#define PJON_MAX_PACKETS        0
#define PJON_PACKET_MAX_LENGTH 20

#include <PJONSoftwareBitBang.h>
#include <EEPROM.h>

// Instantiate PJON
PJONSoftwareBitBang bus;

uint8_t recipient_id;
bool accept_config_change;
uint16_t interval;
uint32_t time;
uint32_t last_sample;

#define DhtPin 0
#define DhtType DHT22
DHT dht (DhtPin, DhtType);

struct sensor_record {float h; float t;};

sensor_record record;
sensor_record buffer;

void setup() {
  // power_adc_disable(); // If the ADC is not required ~320uA are spared

  dht.begin();
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
  time = millis();
}

void EEPROM_read_configuration() {
  bus.set_id(EEPROM.read(0));
  recipient_id = EEPROM.read(1);
  interval = EEPROM.read(3) << 8 | EEPROM.read(4) & 0xFF;
  accept_config_change = EEPROM.read(11);
};

void EEPROM_write_default_configuration() {
  // PJ102 ID
  EEPROM.update(0, PJON_NOT_ASSIGNED);
  // Recipient ID
  EEPROM.update(1, PJON_MASTER_ID);
  // Default interval
  EEPROM.update(3, 0);
  EEPROM.update(4, 0);
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
  if(interval && ((millis() - time) > interval)) {
    if((millis() - last_sample) > MODULE_SAMPLE_RATE) {
      record.h = dht.readHumidity();
      record.t = dht.readTemperature();
      // Check if any reads failed and exit early (to try again)
      if(isnan(record.h) || isnan(record.t)) return;
      last_sample = millis();
      bus.send_packet(recipient_id, &record, sizeof(record));
      time = millis();
    }
  }
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
  // Read DHT sensor
  if(is_master && ((payload[0] == 'S') || (payload[0] == 'E'))) {
    if((millis() - last_sample) > MODULE_SAMPLE_RATE) {
      buffer.h = dht.readHumidity();
      buffer.t = dht.readTemperature();
      if(!isnan(buffer.h) && !isnan(buffer.t))
        record = buffer;
      last_sample = millis();
    }
  }
  // Transmit latest sample
  if(is_master && ((payload[0] == 'G') || (payload[0] == 'E')))
    bus.send_packet(recipient_id, &record, sizeof(record));

  // Return if configuration is blocked
  if(!accept_config_change) return;

  // TRANSMISSION INTERVAL
  if(is_master && (payload[0] == 'T')) {
    interval = payload[1] << 8 | payload[2] & 0xFF;
    if(interval && (interval < MODULE_SAMPLE_RATE))
      interval = MODULE_SAMPLE_RATE;
    EEPROM.update(3, payload[1]);
    EEPROM.update(4, payload[2]);
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
