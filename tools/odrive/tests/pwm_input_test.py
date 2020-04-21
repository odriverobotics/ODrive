
import test_runner

import time
import math
import os

from odrive.enums import errors
from test_runner import *


teensy_code_template = """
float position = 0; // between 0 and 1
float velocity = 1; // [position per second]

void setup() {
{setup_code}
}

// the loop routine runs over and over again forever:
void loop() {
  int high_microseconds = 1000 + (int)(position * 1000.0f);

{set_high_code}
  delayMicroseconds(high_microseconds);
{set_low_code}

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

    This test takes about 1min.

    Note: this test is currently only written for ODrive 3.6 (or similar GPIO layout).
    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            # Find the Teensy that is connected to gpios 1-4 of the ODrive and the corresponding Teensy GPIOs

            gpio_conns = [
                testrig.get_directly_connected_components(odrive.gpio1),
                testrig.get_directly_connected_components(odrive.gpio2),
                testrig.get_directly_connected_components(odrive.gpio3),
                testrig.get_directly_connected_components(odrive.gpio4)
            ]

            valid_combinations = [
                [combination[0].parent] + list(combination)
                for combination in itertools.product(*gpio_conns)
                if ((len(set(c.parent for c in combination)) == 1) and isinstance(combination[0].parent, TeensyComponent))
            ]

            yield (odrive, valid_combinations)

    def run_delta_test(self, attr, with_min, with_max, timeout = 5.0):
        rounds_per_s = 1.0
        units_per_s = (with_max - with_min) * rounds_per_s
        step_size = units_per_s * 0.02 # 20ms per step
        min_val = math.inf
        max_val = -math.inf
        cumulative_delta = 0

        rate = units_per_s
        rate_gain = 1 / 0.5 # 200ms time constant

        # Could do something fancy like fit a piecewise linear function

        start = time.monotonic()
        i = 0
        while True:
            now = time.monotonic()
            new_val = attr.get_value()
            min_val = min(min_val, new_val)
            max_val = max(max_val, new_val)

            if i > 0:
                dt = now - before
                delta = modpm(new_val - last_val, with_max - with_min)
                cumulative_delta += delta

                # low pass filter rate
                rate += (delta / dt - rate) * min(rate_gain * dt, 1)
                #print("rate", rate)
            
                # After 500ms verify the rate
                if now - start > 0.5:
                    # 40% seems like a very large range. With a smaller range
                    # the test tends to fail. May want to investigate if this 
                    # is only because of the non-realtimeness of the tester
                    # of if it's an actual problem. Looks fine on software oscilloscope.
                    test_assert_eq(rate, units_per_s, accuracy=0.4)
 
            before = now
            last_val = new_val
 
            if (now - start > timeout):
                break
            time.sleep(0.005) # PWM time resolution is 20ms, so let's read a bit slower.
            i += 1
        
        # Check the total incement during this time
        test_assert_eq(cumulative_delta, timeout * units_per_s, accuracy=0.1)

        # Check that the minimum and maximum values were observed
        test_assert_eq(min_val, with_min, range = step_size)
        test_assert_eq(max_val, with_max, range = step_size)

    def run_test(self, odrive: ODriveComponent, teensy: TeensyComponent, teensy_gpio1: Component, teensy_gpio2: Component, teensy_gpio3: Component, teensy_gpio4: Component, logger: Logger):
        # TODO: test each GPIO separately
        
        setup_code = "\n".join("  pinMode(" + str(gpio.num) + ", OUTPUT);" for gpio in [teensy_gpio1, teensy_gpio2, teensy_gpio3, teensy_gpio4])
        set_high_code = "\n".join("  digitalWrite(" + str(gpio.num) + ", HIGH);" for gpio in [teensy_gpio1, teensy_gpio2, teensy_gpio3, teensy_gpio4])
        set_low_code = "\n".join("  digitalWrite(" + str(gpio.num) + ", LOW);" for gpio in [teensy_gpio1, teensy_gpio2, teensy_gpio3, teensy_gpio4])

        code = teensy_code_template.replace("{setup_code}", setup_code).replace("{set_high_code}", set_high_code).replace("{set_low_code}", set_low_code)
        teensy.compile_and_program(code)

        time.sleep(1.0) # wait for PLLs to stabilize

        logger.debug("Set up PWM input...")
        odrive.handle.erase_configuration()
        odrive.handle.config.enable_uart = False
        odrive.handle.config.gpio1_pwm_mapping.endpoint = odrive.handle.axis0.controller._remote_attributes['input_pos']
        odrive.handle.config.gpio1_pwm_mapping.min = -50
        odrive.handle.config.gpio1_pwm_mapping.max = 200
        odrive.handle.config.gpio2_pwm_mapping.endpoint = odrive.handle.axis1.controller._remote_attributes['input_pos']
        odrive.handle.config.gpio2_pwm_mapping.min = 20
        odrive.handle.config.gpio2_pwm_mapping.max = 400
        odrive.handle.config.gpio3_pwm_mapping.endpoint = odrive.handle.axis0.controller._remote_attributes['input_vel']
        odrive.handle.config.gpio3_pwm_mapping.min = -1000
        odrive.handle.config.gpio3_pwm_mapping.max = 0
        odrive.handle.config.gpio4_pwm_mapping.endpoint = odrive.handle.axis1.controller._remote_attributes['input_vel']
        odrive.handle.config.gpio4_pwm_mapping.min = -20000
        odrive.handle.config.gpio4_pwm_mapping.max = 20000
        
        odrive.save_config_and_reboot()

        logger.debug("Check if PWM on GPIO1 works...")
        self.run_delta_test(odrive.handle.axis0.controller._remote_attributes['input_pos'], -50, 200)
        logger.debug("Check if PWM on GPIO2 works...")
        self.run_delta_test(odrive.handle.axis1.controller._remote_attributes['input_pos'], 20, 400)
        logger.debug("Check if PWM on GPIO3 works...")
        self.run_delta_test(odrive.handle.axis0.controller._remote_attributes['input_vel'], -1000, 0)
        logger.debug("Check if PWM on GPIO4 works...")
        self.run_delta_test(odrive.handle.axis1.controller._remote_attributes['input_vel'], -20000, 20000)


if __name__ == '__main__':
    test_runner.run(TestPwmInput())
