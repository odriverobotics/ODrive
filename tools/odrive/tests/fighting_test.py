
import test_runner

import time
from math import pi, inf
import os

from fibre.utils import Logger
from test_runner import *
from odrive.enums import *

class FightingTest():
    def get_test_cases(self, testrig: TestRig):
        # Iterate through all pairs of axis+motor+encoder combos on the test rig
        # and return the ones that are connected through the motor shafts.
        all_combos = testrig.get_closed_loop_combos()
        for (axis0, motor0, encoder0, test_fixture0), (axis1, motor1, encoder1, test_fixture1) in itertools.combinations(all_combos, 2):
            is_connected, test_fixture2 = testrig.check_connection(motor0.shaft, motor1.shaft)
            if not is_connected:
                continue # These two motors are not on the same physical axis
            test_fixture = TestFixture.all_of(test_fixture0, test_fixture1, test_fixture2)
            yield axis0, motor0, encoder0, axis1, motor1, encoder1, test_fixture

    def run_test(self, axis0_ctx: ODriveAxisComponent, motor0_ctx: MotorComponent, enc0_ctx: EncoderComponent,
                       axis1_ctx: ODriveAxisComponent, motor1_ctx: MotorComponent, enc1_ctx: EncoderComponent, logger: Logger):

        with SafeTerminator(logger, axis0_ctx, axis1_ctx):
            nominal_rps = 3.0
            nominal_vel = nominal_rps
            logger.debug(f'Testing closed loop velocity control at {nominal_rps} rounds/s...')

            axis0_ctx.handle.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis0_ctx.handle.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            axis0_ctx.handle.controller.input_vel = 0

            request_state(axis0_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
            axis0_ctx.handle.controller.config.vel_limit = nominal_vel
            axis0_ctx.handle.controller.config.vel_limit_tolerance = 2
            axis0_ctx.handle.controller.input_vel = nominal_vel

            data = record_log(lambda: [axis0_ctx.handle.encoder.vel_estimate, axis1_ctx.handle.encoder.vel_estimate, axis0_ctx.handle.encoder.pos_estimate], duration=5.0)

            test_assert_no_error(axis0_ctx)
            test_assert_eq(axis0_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
            request_state(axis0_ctx, AXIS_STATE_IDLE)

            # axis0: encoder.vel_estimate
            slope, offset, fitted_curve = fit_line(data[:,(0,1)])
            test_assert_eq(slope, 0.0, range = nominal_vel * 0.02)
            test_assert_eq(offset, nominal_vel, accuracy = 0.05)
            test_curve_fit(data[:,(0,1)], fitted_curve, max_mean_err = nominal_vel * 0.3, inlier_range = nominal_vel * 0.5, max_outliers = len(data[:,0]) * 0.1)

            # axis1: encoder.vel_estimate
            slope, offset, fitted_curve = fit_line(data[:,(0,2)])
            test_assert_eq(slope, 0.0, range = nominal_vel * 0.02)
            test_assert_eq(offset, -nominal_vel, accuracy = 0.05)
            test_curve_fit(data[:,(0,2)], fitted_curve, max_mean_err = nominal_vel * 0.3, inlier_range = nominal_vel * 0.5, max_outliers = len(data[:,0]) * 0.1)

            # axis0: encoder.pos_estimate
            slope, offset, fitted_curve = fit_line(data[:,(0,3)])
            test_assert_eq(slope, nominal_vel, accuracy = 0.01)
            test_curve_fit(data[:,(0,3)], fitted_curve, max_mean_err = nominal_vel * 0.01, inlier_range = nominal_vel * 0.1, max_outliers = len(data[:,0]) * 0.01)

tests = [
    FightingTest()
]

if __name__ == '__main__':
    test_runner.run(tests)
