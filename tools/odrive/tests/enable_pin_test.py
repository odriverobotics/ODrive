
import test_runner

import time
from math import pi
import os

from fibre.utils import Logger
from test_runner import *
from odrive.enums import *


class TestEnablePin():
    """
    Tests if the startup sequence and closed loop control states are correctly
    governed by the enable pin.
    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            for num in range(2):
                encoders = testrig.get_connected_components({
                    'a': (odrive.encoders[num].a, False),
                    'b': (odrive.encoders[num].b, False)
                }, EncoderComponent)
                motors = testrig.get_connected_components(odrive.axes[num], MotorComponent)
                en_gpio = list(testrig.get_connected_components((odrive.gpio8, False), LinuxGpioComponent))

                for motor, encoder in itertools.product(motors, encoders):
                    if encoder.impl in testrig.get_connected_components(motor):
                        yield (odrive.axes[num], motor, encoder, en_gpio)

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, en_gpio: LinuxGpioComponent, logger: Logger):
        axis = axis_ctx.handle
        cpr = int(enc_ctx.yaml['cpr'])
        
        en_gpio.config(output=True)
        en_gpio.write(False)

        # Reboot into clean state to ensure that the ODrive is uncalibrated.
        axis_ctx.parent.erase_config_and_reboot()
        
        # Configuring the enable pin should not require a reboot
        axis_ctx.handle.config.en_gpio_pin = 8
        axis_ctx.handle.config.enable_pin_active_low = False
        axis_ctx.handle.config.use_enable_pin = True

        axis_ctx.handle.config.startup_motor_calibration = True
        axis_ctx.handle.config.startup_encoder_offset_calibration = True
        axis_ctx.handle.config.startup_closed_loop_control = True

        axis_ctx.handle.clear_errors()

        time.sleep(1)

        # TODO: prevent ODrive from entering closed loop control when requested_state is set and enable pin is disabled

        logger.debug("triggering startup sequence through enable line")
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        en_gpio.write(True)
        time.sleep(0.01) # let the signal settle
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_MOTOR_CALIBRATION)
        
        time.sleep(0.5)

        logger.debug("cancelling startup sequence through enable line")
        # Abort startup sequence after 1 second
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_MOTOR_CALIBRATION)
        en_gpio.write(False)
        time.sleep(0.01)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        time.sleep(0.5)

        # Restart start sequence
        logger.debug("triggering startup sequence through enable line")
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        en_gpio.write(True)
        time.sleep(0.01)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_MOTOR_CALIBRATION)

        # Wait for sequence to complete
        time.sleep(15) # actual time: ~14s

        logger.debug("exiting closed loop control through enable line")
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
        en_gpio.write(False)
        time.sleep(0.01)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        # Reenable axis and verify that it immediately goes to closed loop control
        logger.debug("entering and exiting closed loop control through enable line")
        time.sleep(0.5)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        en_gpio.write(True)
        time.sleep(0.01)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.5)
        en_gpio.write(False)
        time.sleep(0.01)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)


if __name__ == '__main__':
    test_runner.run([
        TestEnablePin()
    ])
