
import test_runner

import time
from math import pi, inf
import os

from fibre.utils import Logger
from test_runner import *
from odrive.enums import *


class TestClosedLoopControlBase():
    """
    Base class for close loop control tests.
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

    def prepare(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
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


        # Return a context that can be used in a with-statement.
        class safe_terminator():
            def __enter__(self):
                pass
            def __exit__(self, exc_type, exc_val, exc_tb):
                logger.debug('clearing config...')
                axis_ctx.handle.requested_state = AXIS_STATE_IDLE
                time.sleep(0.005)
                axis_ctx.parent.erase_config_and_reboot()
        return safe_terminator()


class TestClosedLoopControl(TestClosedLoopControlBase):
    """
    Tests position and velocity control
    """

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        with self.prepare(axis_ctx, motor_ctx, enc_ctx, logger):
            nominal_rps = 1.0
            nominal_vel = float(enc_ctx.yaml['cpr']) * nominal_rps
            logger.debug(f'Testing closed loop velocity control at {nominal_rps} rounds/s...')

            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
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
            
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
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


class TestRegenProtection(TestClosedLoopControlBase):
    """
    Tries to brake with a disabled brake resistor.
    This should result in a low level error disabling all power outputs.
    """

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        with self.prepare(axis_ctx, motor_ctx, enc_ctx, logger):
            nominal_rps = 6.0
            nominal_vel = float(enc_ctx.yaml['cpr']) * nominal_rps
        
            # Accept a bit of noise on Ibus
            axis_ctx.parent.handle.config.dc_max_negative_current = -0.2

            logger.debug(f'Brake control test from {nominal_rps} rounds/s...')
            
            axis_ctx.handle.controller.config.vel_limit = float(enc_ctx.yaml['cpr']) * 10.0 # max 10 rps
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis_ctx.handle.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

            # accelerate...
            axis_ctx.handle.controller.input_vel = nominal_vel
            time.sleep(1.0)
            test_assert_no_error(axis_ctx)

            # ... and brake
            axis_ctx.handle.controller.input_vel = 0
            time.sleep(1.0)
            test_assert_no_error(axis_ctx)

            # once more, but this time without brake resistor
            axis_ctx.parent.handle.config.brake_resistance = 0

            # accelerate...
            axis_ctx.handle.controller.input_vel = nominal_vel
            time.sleep(1.0)
            test_assert_no_error(axis_ctx)

            # ... and brake
            axis_ctx.handle.controller.input_vel = 0 # this should fail almost instantaneously
            time.sleep(0.1)
            test_assert_eq(axis_ctx.handle.error, AXIS_ERROR_MOTOR_DISARMED | AXIS_ERROR_BRAKE_RESISTOR_DISARMED)
            test_assert_eq(axis_ctx.handle.motor.error, MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT)


class TestVelLimitInCurrentControl(TestClosedLoopControlBase):
    """
    Ensures that the current setpoint in current control is always within the
    parallelogram that arises from -Ilim, +Ilim, vel_limit and vel_gain.
    """

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        with self.prepare(axis_ctx, motor_ctx, enc_ctx, logger):
            max_rps = 20.0
            max_vel = float(enc_ctx.yaml['cpr']) * max_rps
            absolute_max_vel = max_vel * 1.2
            max_current = 3.0

            axis_ctx.handle.controller.config.vel_gain /= 10 # reduce the slope to make it easier to see what's going on
            vel_gain = axis_ctx.handle.controller.config.vel_gain
            logger.debug(f'vel gain is {vel_gain}')

            axis_ctx.handle.controller.config.vel_limit = max_vel
            axis_ctx.handle.controller.config.vel_limit_tolerance = inf # disable hard limit on velocity
            axis_ctx.handle.motor.config.current_lim = max_current
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_CURRENT_CONTROL

            # Returns the expected limited setpoint for a given velocity and current
            def get_expected_setpoint(input_setpoint, velocity):
                return clamp(clamp(input_setpoint, (velocity + max_vel) * -vel_gain, (velocity - max_vel) * -vel_gain), -max_current, max_current)

            def data_getter():
                # sample velocity twice to avoid systematic bias
                velocity0 = axis_ctx.handle.encoder.vel_estimate
                current_setpoint = axis_ctx.handle.motor.current_control.Iq_setpoint
                velocity1 = axis_ctx.handle.encoder.vel_estimate
                velocity = ((velocity0 + velocity1) / 2)
                # Abort immediately if the absolute limits are exceeded
                test_assert_within(current_setpoint, -max_current, max_current)
                test_assert_within(velocity, -absolute_max_vel, absolute_max_vel)
                return input_current, velocity, current_setpoint, get_expected_setpoint(input_current, velocity)

            axis_ctx.handle.controller.input_current = input_current = 0.0
            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

            # Move the system around its operating envelope
            axis_ctx.handle.controller.input_current = input_current = 2.0
            dataA = record_log(data_getter, duration=1.0)
            axis_ctx.handle.controller.input_current = input_current = -2.0
            dataA = np.concatenate([dataA, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_current = input_current = 4.0
            dataA = np.concatenate([dataA, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_current = input_current = -4.0
            dataA = np.concatenate([dataA, record_log(data_getter, duration=1.0)])

            # Shrink the operating envelope while motor is moving faster than the envelope allows
            max_rps = 5.0
            max_vel = float(enc_ctx.yaml['cpr']) * max_rps
            axis_ctx.handle.controller.config.vel_limit = max_vel

            # Move the system around its operating envelope
            axis_ctx.handle.controller.input_current = input_current = 2.0
            dataB = record_log(data_getter, duration=1.0)
            axis_ctx.handle.controller.input_current = input_current = -2.0
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_current = input_current = 4.0
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_current = input_current = -4.0
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])

            # Try the shrink maneuver again at positive velocity
            axis_ctx.handle.controller.config.vel_limit = 20.0 * float(enc_ctx.yaml['cpr'])
            axis_ctx.handle.controller.input_current = 4.0
            time.sleep(0.5)
            axis_ctx.handle.controller.config.vel_limit = max_vel

            axis_ctx.handle.controller.input_current = input_current = 2.0
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])

            test_assert_no_error(axis_ctx)

            axis_ctx.handle.requested_state=1
            
            test_curve_fit(dataA[:,(0,3)], dataA[:,4], max_mean_err=0.02, inlier_range=0.05, max_outliers=len(dataA[:,0]*0.01))
            test_curve_fit(dataB[:,(0,3)], dataB[:,4], max_mean_err=0.1, inlier_range=0.2, max_outliers=len(dataB[:,0])*0.01)



if __name__ == '__main__':
    test_runner.run([
        TestClosedLoopControl(),
        TestRegenProtection(),
        TestVelLimitInCurrentControl()
    ])
