
import test_runner

import time
import math
import os

from odrive.enums import *
from test_runner import *


teensy_code_template = """
float position = 0; // between 0 and 1
float velocity = 1; // [position per second]

void setup() {
  pinMode({pwm_gpio}, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  int high_microseconds = 1000 + (int)(position * 1000.0f);

  digitalWrite({pwm_gpio}, HIGH);
  delayMicroseconds(high_microseconds);
  digitalWrite({pwm_gpio}, LOW);

  // Wait for a total of 20ms.
  // delayMicroseconds() only works well for values <= 16383
  delayMicroseconds(10000 - high_microseconds);
  delayMicroseconds(10000);

  position += velocity * 0.02;
  while (position > 1.0)
      position -= 1.0;
}
"""


class TestPwmInput():
    """
    Verifies the PWM input.

    The Teensy generates a PWM signal that goes from 0% (1ms high) to 100% (2ms high)
    in 1 second and then resumes at 0%.

    Note: this test is currently only written for ODrive 3.6 (or similar GPIO layout).
    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            # Run a separate test for each PWM-capable GPIO. Use different min/max settings for each test.
            yield (odrive, 1, -50, 200, list(testrig.get_connected_components(odrive.gpio1, TeensyGpio)))
            yield (odrive, 2, 20, 400, list(testrig.get_connected_components(odrive.gpio2, TeensyGpio)))
            yield (odrive, 3, -1000, 0, list(testrig.get_connected_components(odrive.gpio3, TeensyGpio)))
            yield (odrive, 4, -20000, 20000, list(testrig.get_connected_components(odrive.gpio4, TeensyGpio)))

    def run_test(self, odrive: ODriveComponent, odrive_gpio_num: int, min_val: float, max_val: float, teensy_gpio: Component, logger: Logger):
        teensy = teensy_gpio.parent
        code = teensy_code_template.replace("{pwm_gpio}", str(teensy_gpio.num))
        teensy.compile_and_program(code)

        logger.debug("Set up PWM input...")
        odrive.unuse_gpios()

        pwm_mapping = [
            odrive.handle.config.gpio1_pwm_mapping,
            odrive.handle.config.gpio2_pwm_mapping,
            odrive.handle.config.gpio3_pwm_mapping,
            odrive.handle.config.gpio4_pwm_mapping
        ][odrive_gpio_num - 1]

        pwm_mapping.endpoint = odrive.handle.axis0.controller._remote_attributes['input_pos']
        pwm_mapping.min = min_val
        pwm_mapping.max = max_val
        
        odrive.save_config_and_reboot()
        
        data = record_log(lambda: [odrive.handle.axis0.controller.input_pos], duration=5.0)
        
        full_scale = max_val - min_val
        slope, offset, fitted_curve = fit_sawtooth(data, min_val, max_val)
        test_assert_eq(slope, full_scale / 1.0, accuracy=0.001)
        test_curve_fit(data, fitted_curve, max_mean_err = full_scale * 0.05, inlier_range = full_scale * 0.05, max_outliers = len(data[:,0]) * 0.01)



if __name__ == '__main__':
    test_runner.run(TestPwmInput())
