
#include <Wire.h>
#include "odrive.h"


// See odrive.h for a description
bool I2C_transaction(uint8_t slave_addr, const uint8_t * tx_buffer, size_t tx_length, uint8_t * rx_buffer, size_t rx_length) {
  // transmit
  if (tx_buffer) {
    Wire.beginTransmission(slave_addr);
    if (Wire.write(tx_buffer, tx_length) != tx_length)
      return false;
    bool should_stop = !rx_buffer;
    if (Wire.endTransmission(should_stop) != 0)
      return false;
  }

  // receive
  if (rx_buffer) {
    while(Wire.available()) Wire.read(); // flush input buffer
    if (Wire.requestFrom(slave_addr, (uint8_t)rx_length, (uint8_t)true /* stop after receiving */) != rx_length)
      return false;
    for (size_t i = 0; i < rx_length; ++i)
      rx_buffer[i] = Wire.read();
  }

  return true;
}


void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
}

byte odrive_num = 7;
byte x;

void loop() {
  bool success;

  float val;
  success = odrive::read_property<odrive::VBUS_VOLTAGE>(odrive_num, &val);
  if (success) {
    Serial.println(val, HEX);
  } else {
    Serial.println("error");
  }

  success = odrive::write_property<odrive::TEST_PROPERTY>(odrive_num, x++);
  if (!success)
    Serial.println("error");

  success = odrive::trigger<odrive::SAVE_CONFIGURATION>(odrive_num);
  if (!success)
    Serial.println("error");
  else
    Serial.println("saved config");

  delay(500);
}

