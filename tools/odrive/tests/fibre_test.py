
import test_runner

import time

from fibre.utils import Logger
from odrive.enums import *
from test_runner import *

class FibreFunctionalTest():
    """
    Tests basic protocol functionality.
    """

    def get_test_cases(self, testrig: TestRig):
        return [(odrv, None) for odrv in testrig.get_components(ODriveComponent)]

    def run_test(self, odrive: ODriveComponent, logger: Logger):
        # Test property read/write
        odrive.handle.test_property = 42
        test_assert_eq(odrive.handle.test_property, 42)
        odrive.handle.test_property = 0xffffffff
        test_assert_eq(odrive.handle.test_property, 0xffffffff)

        # Test function call
        val = odrive.handle.get_adc_voltage(2) # ADC pin on both ODrive 3 and 4
        test_assert_within(val, 0.01, 3.29)

        # Test custom setter (aka property write hook)
        odrive.handle.axis0.motor.config.phase_resistance = 1
        odrive.handle.axis0.motor.config.phase_inductance = 1
        odrive.handle.axis0.motor.config.current_control_bandwidth = 1000
        old_gain = odrive.handle.axis0.motor.current_control.p_gain
        test_assert_eq(old_gain, 1000, accuracy=0.0001) # must be non-zero for subsequent check to work
        odrive.handle.axis0.motor.config.current_control_bandwidth /= 2
        test_assert_eq(odrive.handle.axis0.motor.current_control.p_gain, old_gain / 2, accuracy=0.0001)

class FibreBurnInTest():
    """
    Tests continuous usage of the protocol.
    """

    def get_test_cases(self, testrig: TestRig):
        return [(odrv, None) for odrv in testrig.get_components(ODriveComponent)]

    def run_test(self, odrive: ODriveComponent, logger: Logger):
        data = record_log(lambda: [odrive.handle.vbus_voltage], duration=10.0)
        expected_data = np.mean(data[:,1]) * np.ones(data[:,1].size)
        test_curve_fit(data, expected_data, max_mean_err = 0.1, inlier_range = 0.5, max_outliers = 0)

tests = [
    FibreFunctionalTest(),
    FibreBurnInTest(),
]

if __name__ == '__main__':
    test_runner.run(tests)
