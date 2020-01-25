
import test_runner

import time
from math import pi
import os

from fibre.utils import Logger
from test_runner import AxisTestContext, MotorTestContext, test_assert_eq, test_assert_no_error, request_state
from odrive.enums import *

def modpm(val, range):
    return ((val + (range / 2)) % range) - (range / 2)

class TestMotorCalibration():
    """
    Runs the motor calibration and checks if the measurements match the expectation.
    """

    def is_compatible(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext):
        return axis_ctx.yaml == motor_ctx.yaml['name'] # check if connected

    def run_test(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, logger: Logger):
        # reset old calibration values
        axis_ctx.handle.motor.config.phase_resistance = 0.0
        axis_ctx.handle.motor.config.phase_inductance = 0.0

        axis_ctx.handle.clear_errors()

        # run calibration
        request_state(axis_ctx, AXIS_STATE_MOTOR_CALIBRATION)
        time.sleep(6)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        # check if measurements match expectation
        test_assert_eq(axis_ctx.handle.motor.config.phase_resistance, float(motor_ctx.yaml['phase-resistance']), accuracy=0.2)
        test_assert_eq(axis_ctx.handle.motor.config.phase_inductance, float(motor_ctx.yaml['phase-inductance']), accuracy=0.5)


class TestDisconnectedMotorCalibration():
    """
    Tests if the motor calibration fails as expected if the phases are floating.
    """

    def is_compatible(self, axis_ctx: AxisTestContext):
        return axis_ctx.yaml == 'floating'

    def run_test(self, axis_ctx: AxisTestContext, logger: Logger):
        axis = axis_ctx.handle

        # reset old calibration values
        axis_ctx.handle.motor.config.phase_resistance = 0.0
        axis_ctx.handle.motor.config.phase_inductance = 0.0

        axis_ctx.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_MOTOR_CALIBRATION)
        time.sleep(6)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis_ctx.handle.error, errors.axis.ERROR_MOTOR_FAILED)
        test_assert_eq(axis_ctx.handle.motor.error, errors.motor.ERROR_PHASE_RESISTANCE_OUT_OF_RANGE)


if __name__ == '__main__':
    test_runner.run([
        TestMotorCalibration(),
        TestDisconnectedMotorCalibration()
    ])
