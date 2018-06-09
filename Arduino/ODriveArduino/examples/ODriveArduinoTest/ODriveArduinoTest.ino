
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(8, 9); //RX (ODrive TX), TX (ODrive RX)

// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino alpha.");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  for (int motor = 0; motor < 2; ++motor) {
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_CURRENT_CONTROL_CURRENT_LIM, 10.0f);  // [A]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_LIMIT, 20000.0f);                 // [counts/s]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_POS_GAIN, 20.0f);                     // [(counts/s) / counts]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_GAIN, 15.0f/10000.0f);            // [A/(counts/s)]
    odrive.SetParameter(motor, odrive.PARAM_FLOAT_VEL_INTEGRATOR_GAIN, 0.0f/10000.0f); // [A/(counts/s * s)]
  }

  Serial.println("Ready!");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    // Sinusoidal test move
    if (c == 's') {
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 20000.0f * cos(ph);
        float pos_m1 = 20000.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    if (c == 'b') {
      Serial << "Vbus voltage: " << odrive.getBusVoltage() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          Serial << odrive.GetParameter(motor, odrive.PARAM_FLOAT_ENCODER_PLL_POS) << '\t';
        }
        Serial << '\n';
      }
    }
  }
}
