
import test_runner

import struct
import asyncio
import time

from fibre.utils import Logger
from odrive.enums import *
from test_runner import *

class TestStepDir():
    """
    Tests Step/Dir input.
    Not all possible combinations are tested, but each axis and each GPIO
    participates in at least one test case.

    The tests are conducted while the axis is in idle.
    """
    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            gpio_conns = [
                list(testrig.get_connected_components((odrive.gpio1, False), LinuxGpioComponent)),
                list(testrig.get_connected_components((odrive.gpio2, False), LinuxGpioComponent)),
                #list(testrig.get_connected_components((odrive.gpio3, False), LinuxGpioComponent)), # connected to LPF on test rig
                #list(testrig.get_connected_components((odrive.gpio4, False), LinuxGpioComponent)), # connected to LPF on test rig
                list(testrig.get_connected_components((odrive.gpio5, False), LinuxGpioComponent)),
                list(testrig.get_connected_components((odrive.gpio6, False), LinuxGpioComponent)),
                list(testrig.get_connected_components((odrive.gpio7, False), LinuxGpioComponent)),
                list(testrig.get_connected_components((odrive.gpio8, False), LinuxGpioComponent)),
            ]

            yield (odrive.axes[0], 1, gpio_conns[0], 2, gpio_conns[1])
            yield (odrive.axes[0], 5, gpio_conns[2], 6, gpio_conns[3])
            yield (odrive.axes[0], 7, gpio_conns[4], 8, gpio_conns[5]) # broken
           # yield (odrive.axes[0], 7, gpio_conns[6], 8, gpio_conns[7]) # broken
            
            yield (odrive.axes[1], 7, gpio_conns[4], 8, gpio_conns[5])

    def run_test(self, axis: ODriveAxisComponent,  step_gpio_num: int, step_gpio: LinuxGpioComponent, dir_gpio_num: int, dir_gpio: LinuxGpioComponent, logger: Logger):
        step_gpio.config(output=True)
        step_gpio.write(False)
        dir_gpio.config(output=True)
        dir_gpio.write(True)

        if axis.num == 0:
            axis.parent.handle.config.enable_uart = False
        axis.handle.config.enable_step_dir = True
        axis.handle.config.step_dir_always_on = True # needed for testing
        axis.handle.config.step_gpio_pin = step_gpio_num
        axis.handle.config.dir_gpio_pin = dir_gpio_num
        request_state(axis, AXIS_STATE_IDLE) # apply step_dir_always_on config


        ref = axis.handle.controller.input_pos
        axis.handle.config.turns_per_step = turns_per_step = 10

        # On the RPi 4 a ~5kHz GPIO signal can be generated from Python

        for i in range(100):
            step_gpio.write(True)
            step_gpio.write(False)
            test_assert_eq(axis.handle.controller.input_pos, ref + (i + 1) * turns_per_step, range = 0.4 * turns_per_step)

        ref = axis.handle.controller.input_pos
        dir_gpio.write(False)

        for i in range(100):
            step_gpio.write(True)
            step_gpio.write(False)
            test_assert_eq(axis.handle.controller.input_pos, ref - (i + 1) * turns_per_step, range = 0.4 * turns_per_step)

        ref = axis.handle.controller.input_pos
        dir_gpio.write(True)
        axis.handle.config.turns_per_step = turns_per_step = 1

        for i in range(100):
            step_gpio.write(True)
            step_gpio.write(False)
            test_assert_eq(axis.handle.controller.input_pos, ref + (i + 1) * turns_per_step, range = 0.4 * turns_per_step)

        ref = axis.handle.controller.input_pos
        axis.handle.config.turns_per_step = turns_per_step = -1

        for i in range(100):
            step_gpio.write(True)
            step_gpio.write(False)
            test_assert_eq(axis.handle.controller.input_pos, ref + (i + 1) * turns_per_step, range = 0.4 * abs(turns_per_step))


if __name__ == '__main__':
    test_runner.run(TestStepDir())
