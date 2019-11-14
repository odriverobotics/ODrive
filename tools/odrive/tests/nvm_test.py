
import test_runner

import time
from math import pi
import os

import fibre
from fibre.utils import Logger
from test_runner import ODriveTestContext, test_assert_eq

class TestStoreAndReboot():
    """
    Stores the current configuration to NVM and reboots.
    """

    def is_compatible(self, odrive: ODriveTestContext):
        return True

    def run_with_values(self, values, odrive: ODriveTestContext, logger: Logger):
        logger.debug("storing configuration and rebooting...")

        for value in values:
            odrive.handle.config.brake_resistance = value
        
        odrive.handle.save_configuration()
        try:
            odrive.handle.reboot()
        except fibre.ChannelBrokenException:
            pass # this is expected
        odrive.handle = None
        time.sleep(2)

        odrive.make_available(logger)

        logger.debug("verifying configuration after reboot...")
        test_assert_eq(odrive.handle.config.brake_resistance, values[-1], accuracy=0.01)

    def run_test(self, odrive: ODriveTestContext, logger):
        self.run_with_values([0.5, 1.0, 1.5], odrive, logger)
        self.run_with_values([2.5, 3.7], odrive, logger)
        self.run_with_values([0.47], odrive, logger)

if __name__ == '__main__':
    test_runner.run(TestStoreAndReboot())
