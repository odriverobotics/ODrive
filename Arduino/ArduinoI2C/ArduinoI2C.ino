
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


int set_and_save_configuration(uint8_t odrive_num, uint8_t axis_num) {
  bool success;
  success = odrive::clear_errors(odrive_num, axis_num);
  if (!success)
    return __LINE__;

  // select hall effect mode
  bool user_config_loaded = false;
  success = odrive::read_property<odrive::USER_CONFIG_LOADED>(odrive_num, &user_config_loaded);
  if (!success)
    return __LINE__;
  if (user_config_loaded) {
    Serial.println("ODrive already configured");
    return 0;
  }

  // select hall effect mode
  success = odrive::write_axis_property<odrive::AXIS__ENCODER__CONFIG__MODE>(odrive_num, axis_num, 1);
  if (!success)
    return __LINE__;

  // configure encoder counts per revolution (6 hall effect states * 12 pole pairs)
  success = odrive::write_axis_property<odrive::AXIS__ENCODER__CONFIG__CPR>(odrive_num, axis_num, 72);
  if (!success)
    return __LINE__;

  // disable velocity integrator
  success = odrive::write_axis_property<odrive::AXIS__CONTROLLER__CONFIG__VEL_INTEGRATOR_GAIN>(odrive_num, axis_num, 0);
  if (!success)
    return __LINE__;

  // select velocity control
  success = odrive::write_axis_property<odrive::AXIS__CONTROLLER__CONFIG__CONTROL_MODE>(odrive_num, axis_num, 2);
  if (!success)
    return __LINE__;

  // set velocity controller P-gain
  success = odrive::write_axis_property<odrive::AXIS__CONTROLLER__CONFIG__VEL_GAIN>(odrive_num, axis_num, 0.005f);
  if (!success)
    return __LINE__;

  // request state: motor calibration
  success = odrive::write_axis_property<odrive::AXIS__REQUESTED_STATE>(odrive_num, axis_num, 4);
  if (!success)
    return __LINE__;

  delay(6000);

  // check if the axis is in idle and no errors occurred
  if (!odrive::check_axis_state(odrive_num, axis_num, 1))
    return __LINE__;

  // ensure that the motor calibration is considered valid after power cycle
  success = odrive::write_axis_property<odrive::AXIS__MOTOR__CONFIG__PRE_CALIBRATED>(odrive_num, axis_num, true);
  if (!success)
    return __LINE__;

  // request state: encoder calibration
  success = odrive::write_axis_property<odrive::AXIS__REQUESTED_STATE>(odrive_num, axis_num, 7);
  if (!success)
    return __LINE__;

  delay(12000);

  // check if the axis is in idle and no errors occurred
  if (!odrive::check_axis_state(odrive_num, axis_num, 1))
    return __LINE__;

  // ensure that the encoder calibration is considered valid after power cycle
  success = odrive::write_axis_property<odrive::AXIS__ENCODER__CONFIG__PRE_CALIBRATED>(odrive_num, axis_num, true);
  if (!success)
    return __LINE__;

  // store the configuration to NVM
  // Caution: this operation is usually instantaneous but after every couple of hundred calls it will
  // take around 1 second (because a flash page needs to be erased).
  success = odrive::trigger<odrive::SAVE_CONFIGURATION>(odrive_num);
  if (!success)
    return __LINE__;
  return 0;
}


byte odrive_num = 7;
byte axis_num = 0;
bool do_setup = true;

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
  Serial.println("Hello World!");

  if (do_setup) {
    Serial.println("Starting ODrive setup...");
    int error_line = set_and_save_configuration(odrive_num, axis_num);
    if (error_line != 0) {
      Serial.print("ODrive setup failed at line ");
      Serial.print(error_line);
      Serial.println();
      return;
    }
    Serial.println("ODrive setup succeeded!");
    do_setup = false;
  }
}


void loop() {
  bool success;
  delay(500);

  success = odrive::check_axis_state(odrive_num, axis_num, 8);
  if (!success) {
    Serial.println("not in closed loop control - entering closed loop control");

    // clear previous error state
    success = odrive::clear_errors(odrive_num, axis_num);
    if (!success) {
      Serial.println("could not enter closed loop control");
      return;
    }

    // request velocity 0
    success = odrive::write_axis_property<odrive::AXIS__CONTROLLER__VEL_SETPOINT>(odrive_num, axis_num, 0);
    if (!success) {
      Serial.println("could not enter closed loop control");
      return;
    }
    
    // request state: closed loop control
    success = odrive::write_axis_property<odrive::AXIS__REQUESTED_STATE>(odrive_num, axis_num, 8);
    if (!success) {
      Serial.println("could not enter closed loop control");
      return;
    }

    success = odrive::check_axis_state(odrive_num, axis_num, 8);
    if (!success) {
      Serial.println("could not enter closed loop control");
      return;
    }
  }

  success = odrive::write_axis_property<odrive::AXIS__CONTROLLER__VEL_SETPOINT>(odrive_num, axis_num, 72 * 5);
  if (!success) {
    Serial.println("error");
    return;
  }

  delay(500);

  success = odrive::write_axis_property<odrive::AXIS__CONTROLLER__VEL_SETPOINT>(odrive_num, axis_num, -72 * 5);
  if (!success) {
    Serial.println("error");
    return;
  }

  // print Vbus to show liveness
  float vbus;
  success = odrive::read_property<odrive::VBUS_VOLTAGE>(odrive_num, &vbus);
  if (!success) {
    Serial.println("error");
    return;
  }
  Serial.println(vbus);
}

