
import test_runner

import time
from math import pi
import os

import fibre
from fibre.utils import Logger
from test_runner import *

class TestStoreAndReboot():
    """
    Stores the current configuration to NVM and reboots.
    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            yield (odrive, None)

    def run_with_values(self, odrive: ODriveComponent, values: list, logger: Logger):
        logger.debug("storing configuration and rebooting...")

        for value in values:
            odrive.handle.config.brake_resistance = value
        
        odrive.save_config_and_reboot()

        odrive.prepare(logger)

        logger.debug("verifying configuration after reboot...")
        test_assert_eq(odrive.handle.config.brake_resistance, values[-1], accuracy=0.01)

    def run_test(self, odrive: ODriveComponent, logger: Logger):
        self.run_with_values(odrive, [0.5, 1.0, 1.5], logger)
        self.run_with_values(odrive, [2.5, 3.7], logger)
        self.run_with_values(odrive, [0.47], logger)

tests = [TestStoreAndReboot()]

if __name__ == '__main__':
    test_runner.run(tests)
