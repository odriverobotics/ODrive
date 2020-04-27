
import test_runner

import time
from math import pi
import os

from fibre.utils import Logger
from test_runner import *
from odrive.enums import *


class TestClosedLoopControl():
    """
    """

    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            for num in range(2):
                encoders = testrig.get_connected_components({
                    'a': (odrive.encoders[num].a, False),
                    'b': (odrive.encoders[num].b, False)
                }, EncoderComponent)
                motors = testrig.get_connected_components(odrive.axes[num], MotorComponent)

                for motor, encoder in itertools.product(motors, encoders):
                    if encoder.impl in testrig.get_connected_components(motor):
                        yield (odrive.axes[num], motor, encoder)

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        axis = axis_ctx.handle
        time.sleep(1.0) # wait for PLLs to stabilize

        # Make sure there are no funny configurations active
        logger.debug('Setting up clean configuration...')
        axis_ctx.parent.erase_config_and_reboot()

        # Set motor calibration values
        axis_ctx.handle.motor.config.phase_resistance = float(motor_ctx.yaml['phase-resistance'])
        axis_ctx.handle.motor.config.phase_inductance = float(motor_ctx.yaml['phase-inductance'])
        axis_ctx.handle.motor.config.pre_calibrated = True

        # Set calibration settings
        axis_ctx.handle.motor.config.direction = 0
        axis_ctx.handle.encoder.config.use_index = False
        axis_ctx.handle.encoder.config.calib_scan_omega = 12.566 # 2 electrical revolutions per second
        axis_ctx.handle.encoder.config.calib_scan_distance = 50.265 # 8 revolutions
        axis_ctx.handle.encoder.config.bandwidth = 1000


        axis_ctx.handle.clear_errors()

        logger.debug('Calibrating encoder offset...')
        request_state(axis_ctx, AXIS_STATE_ENCODER_OFFSET_CALIBRATION)

        time.sleep(9) # actual calibration takes 8 seconds

        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        nominal_rps = 1.0
        nominal_vel = float(enc_ctx.yaml['cpr']) * nominal_rps
        logger.debug(f'Testing closed loop velocity control at {nominal_rps} rounds/s...')

        axis_ctx.handle.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        axis_ctx.handle.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis_ctx.handle.controller.input_vel = 0

        request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
        axis_ctx.handle.controller.input_vel = nominal_vel

        data = record_log(lambda: [axis_ctx.handle.encoder.vel_estimate, axis_ctx.handle.encoder.pos_estimate], duration=5.0)

        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
        test_assert_no_error(axis_ctx)
        request_state(axis_ctx, AXIS_STATE_IDLE)

        # encoder.vel_estimate
        slope, offset, fitted_curve = fit_line(data[:,(0,1)])
        test_assert_eq(slope, 0.0, range = nominal_vel * 0.02)
        test_assert_eq(offset, nominal_vel, accuracy = 0.05)
        test_curve_fit(data[:,(0,1)], fitted_curve, max_mean_err = nominal_vel * 0.3, inlier_range = nominal_vel * 0.5, max_outliers = len(data[:,0]) * 0.1)

        # encoder.pos_estimate
        slope, offset, fitted_curve = fit_line(data[:,(0,2)])
        test_assert_eq(slope, nominal_vel, accuracy = 0.01)
        test_curve_fit(data[:,(0,2)], fitted_curve, max_mean_err = nominal_vel * 0.01, inlier_range = nominal_vel * 0.1, max_outliers = len(data[:,0]) * 0.01)


        logger.debug(f'Testing closed loop position control...')
        
        axis_ctx.handle.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        axis_ctx.handle.controller.input_pos = 0
        axis_ctx.handle.controller.config.vel_limit = float(enc_ctx.yaml['cpr']) * 5.0 # max 5 rps
        axis_ctx.handle.encoder.set_linear_count(0)

        request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

        # Test small position changes
        axis_ctx.handle.controller.input_pos = 5000
        time.sleep(0.3)
        test_assert_no_error(axis_ctx)
        test_assert_eq(axis_ctx.handle.encoder.pos_estimate, 5000, range=2000) # large range needed because of cogging torque
        axis_ctx.handle.controller.input_pos = -5000
        time.sleep(0.3)
        test_assert_no_error(axis_ctx)
        test_assert_eq(axis_ctx.handle.encoder.pos_estimate, -5000, range=2000)
        
        axis_ctx.handle.controller.input_pos = 0
        time.sleep(0.3)

        nominal_vel = float(enc_ctx.yaml['cpr']) * 5.0
        axis_ctx.handle.controller.input_pos = nominal_vel * 2.0 # 10 turns (takes 2 seconds)
        
        # Test large position change with bounded velocity
        data = record_log(lambda: [axis_ctx.handle.encoder.vel_estimate, axis_ctx.handle.encoder.pos_estimate], duration=4.0)
        
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
        test_assert_no_error(axis_ctx)
        request_state(axis_ctx, AXIS_STATE_IDLE)

        data_motion = data[data[:,0] < 1.9]
        data_still = data[data[:,0] > 2.1]

        # encoder.vel_estimate
        slope, offset, fitted_curve = fit_line(data_motion[:,(0,1)])
        test_assert_eq(slope, 0.0, range = nominal_vel * 0.05)
        test_assert_eq(offset, nominal_vel, accuracy = 0.05)
        test_curve_fit(data_motion[:,(0,1)], fitted_curve, max_mean_err = nominal_vel * 0.05, inlier_range = nominal_vel * 0.1, max_outliers = len(data[:,0]) * 0.01)

        # encoder.pos_estimate
        slope, offset, fitted_curve = fit_line(data_motion[:,(0,2)])
        test_assert_eq(slope, nominal_vel, accuracy = 0.01)
        test_curve_fit(data_motion[:,(0,2)], fitted_curve, max_mean_err = nominal_vel * 0.01, inlier_range = nominal_vel * 0.1, max_outliers = len(data[:,0]) * 0.01)

        # encoder.vel_estimate
        slope, offset, fitted_curve = fit_line(data_still[:,(0,1)])
        test_assert_eq(slope, 0.0, range = nominal_vel * 0.05)
        test_assert_eq(offset, 0.0, range = nominal_vel * 0.05)
        test_curve_fit(data_still[:,(0,1)], fitted_curve, max_mean_err = nominal_vel * 0.05, inlier_range = nominal_vel * 0.1, max_outliers = len(data[:,0]) * 0.01)

        # encoder.pos_estimate
        slope, offset, fitted_curve = fit_line(data_still[:,(0,2)])
        test_assert_eq(slope, 0.0, range = nominal_vel * 0.05)
        test_assert_eq(offset, nominal_vel*2, range = nominal_vel * 0.02)
        test_curve_fit(data_still[:,(0,2)], fitted_curve, max_mean_err = nominal_vel * 0.01, inlier_range = nominal_vel * 0.01, max_outliers = len(data[:,0]) * 0.01)



if __name__ == '__main__':
    test_runner.run([
        TestClosedLoopControl()
    ])
