
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
            if odrive.yaml['board-version'].startswith('v3.'):
                # Run a separate test for each PWM-capable GPIO. Use different min/max settings for each test.
                test_cases = [(1, -50, 200, odrive.gpio1),
                              (2, 20, 400, odrive.gpio2),
                              (3, -1000, 0, odrive.gpio3),
                              (4, -20000, 20000, odrive.gpio4)]
            elif odrive.yaml['board-version'].startswith('v4.'):
                # Run a separate test for each PWM-capable GPIO. Use different min/max settings for each test.
                test_cases = [(14, -50, 200, odrive.gpio14),
                              (19, 20, 400, odrive.gpio19),
                              (20, -20000, 20000, odrive.gpio20),
                              (21, -1000, 0, odrive.gpio21)]
            else:
                raise Exception(f"unknown board version {odrive.yaml['board-version']}")

            for test_case in test_cases:
                yield AnyTestCase(*[(odrive,) + tuple(test_case[:-1]) + (teensy_gpio,tf,) for teensy_gpio, tf in testrig.get_connected_components(test_case[-1], TeensyGpio)])

    def run_test(self, odrive: ODriveComponent, odrive_gpio_num: int, min_val: float, max_val: float, teensy_gpio: Component, logger: Logger):
        teensy = teensy_gpio.parent
        code = teensy_code_template.replace("{pwm_gpio}", str(teensy_gpio.num))
        teensy.compile_and_program(code)

        logger.debug("Set up PWM input...")
        odrive.disable_mappings()

        setattr(odrive.handle.config, f'gpio{odrive_gpio_num}_mode', GPIO_MODE_PWM)
        pwm_mapping = getattr(odrive.handle.config, f'gpio{odrive_gpio_num}_pwm_mapping')
        pwm_mapping.endpoint = odrive.handle.axis0.controller._input_pos_property
        pwm_mapping.min = min_val
        pwm_mapping.max = max_val
        
        odrive.save_config_and_reboot()
        
        data = record_log(lambda: [odrive.handle.axis0.controller.input_pos], duration=5.0)
        
        full_scale = max_val - min_val
        slope, offset, fitted_curve = fit_sawtooth(data, min_val, max_val)
        test_assert_eq(slope, full_scale / 1.0, accuracy=0.001)
        test_curve_fit(data, fitted_curve, max_mean_err = full_scale * 0.05, inlier_range = full_scale * 0.05, max_outliers = len(data[:,0]) * 0.01)

tests = [TestPwmInput()]

if __name__ == '__main__':
    test_runner.run(tests)
