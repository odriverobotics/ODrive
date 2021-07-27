
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
            def stepdir_test_case(axis, step_gpio_num, dir_gpio_num):
                alternatives = []
                for step_ctrl_gpio, tf1 in testrig.get_connected_components((getattr(odrive, f'gpio{step_gpio_num}'), False), LinuxGpioComponent):
                    for dir_ctrl_gpio, tf2 in testrig.get_connected_components((getattr(odrive, f'gpio{dir_gpio_num}'), False), LinuxGpioComponent):
                        alternatives.append((axis, step_gpio_num, step_ctrl_gpio, dir_gpio_num, dir_ctrl_gpio, TestFixture.all_of(tf1, tf2)))
                return AnyTestCase(*alternatives)

            yield stepdir_test_case(odrive.axes[0], 1, 2)
            yield stepdir_test_case(odrive.axes[0], 5, 6)
            yield stepdir_test_case(odrive.axes[0], 7, 8)
            
            # test other axes
            for i in range(1, len(odrive.axes)):
                yield stepdir_test_case(odrive.axes[i], 7, 8)

    def run_test(self, axis: ODriveAxisComponent, step_gpio_num: int, step_gpio: LinuxGpioComponent, dir_gpio_num: int, dir_gpio: LinuxGpioComponent, logger: Logger):
        step_gpio.config(output=True)
        step_gpio.write(False)
        dir_gpio.config(output=True)
        dir_gpio.write(True)

        axis.parent.erase_config_and_reboot()
        setattr(axis.parent.handle.config, 'gpio' + str(step_gpio_num) + '_mode', GPIO_MODE_DIGITAL)
        setattr(axis.parent.handle.config, 'gpio' + str(dir_gpio_num) + '_mode', GPIO_MODE_DIGITAL)
        axis.parent.save_config_and_reboot()
        axis.handle.config.enable_step_dir = True
        axis.handle.config.step_dir_always_on = True # needed for testing
        axis.handle.config.step_gpio_pin = step_gpio_num
        axis.handle.config.dir_gpio_pin = dir_gpio_num
        request_state(axis, AXIS_STATE_IDLE) # apply step_dir_always_on config


        ref = axis.handle.steps
        turns_per_step = 1
        axis.handle.controller.config.steps_per_circular_range = 1.0 / turns_per_step

        # On the RPi 4 a ~5kHz GPIO signal can be generated from Python

        for i in range(100):
            step_gpio.write(True)
            test_assert_eq(axis.handle.steps, ref + (i + 1) * turns_per_step, range = 0.4 * turns_per_step)
            step_gpio.write(False)
            test_assert_eq(axis.handle.steps, ref + (i + 1) * turns_per_step, range = 0.4 * turns_per_step)

        ref = axis.handle.steps
        dir_gpio.write(False)

        for i in range(100):
            step_gpio.write(True)
            test_assert_eq(axis.handle.steps, ref - (i + 1) * turns_per_step, range = 0.4 * turns_per_step)
            step_gpio.write(False)
            test_assert_eq(axis.handle.steps, ref - (i + 1) * turns_per_step, range = 0.4 * turns_per_step)

        ref = axis.handle.steps
        dir_gpio.write(True)
        turns_per_step = 1
        axis.handle.controller.config.steps_per_circular_range = 1.0 / turns_per_step

        for i in range(100):
            step_gpio.write(True)
            test_assert_eq(axis.handle.steps, ref + (i + 1) * turns_per_step, range = 0.4 * turns_per_step)
            step_gpio.write(False)
            test_assert_eq(axis.handle.steps, ref + (i + 1) * turns_per_step, range = 0.4 * turns_per_step)

#        ref = axis.handle.steps
#        turns_per_step = -1
#        axis.handle.controller.config.steps_per_circular_range = 1.0 / turns_per_step
#
#        for i in range(100):
#            step_gpio.write(True)
#            test_assert_eq(axis.handle.steps, ref + (i + 1) * turns_per_step, range = 0.4 * abs(turns_per_step))
#            step_gpio.write(False)
#            test_assert_eq(axis.handle.steps, ref + (i + 1) * turns_per_step, range = 0.4 * abs(turns_per_step))

tests = [TestStepDir()]

if __name__ == '__main__':
    test_runner.run(tests)
