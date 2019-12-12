
import test_runner

import time
import math
import os

import fibre
from fibre.utils import Logger
from odrive.enums import errors
from test_runner import ODriveTestContext, test_assert_eq, program_teensy

#def modpm(val, lower_bound, upper_bound):
#    return ((val - lower_bound) % (upper_bound - lower_bound)) - lower_bound

def modpm(val, range):
    return ((val + (range / 2)) % range) - (range / 2)

class TestPwmInput():
    """
    Verifies the PWM input.

    The Teensy generates a PWM signal that goes from 0% (1ms high) to 100% (2ms high)
    in 1 second and then resumes at 0%.

    This test takes about 1min.

    Note: this test is currently only written for ODrive 3.6 (or similar GPIO layout).
    """

    def is_compatible(self, odrive: ODriveTestContext):
        return True

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

    def run_test(self, odrive: ODriveTestContext, logger: Logger):
        # TODO: test each GPIO separately
        hexfile = 'pwm_sim.ino.hex'
        program_teensy(os.path.join(os.path.dirname(__file__), hexfile), 26, logger)
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
        
        # Save and reboot
        odrive.handle.save_configuration()
        try:
            odrive.handle.reboot()
        except fibre.ChannelBrokenException:
            pass # this is expected
        odrive.handle = None
        time.sleep(2)
        odrive.make_available(logger)

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
