
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
    pass

class TestClosedLoopControl(TestClosedLoopControlBase):
    """
    Tests position and velocity control
    """
    def get_test_cases(self, testrig: TestRig):
        return testrig.get_closed_loop_combos()

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        with SafeTerminator(logger, axis_ctx):
            nominal_rps = 3.0
            nominal_vel = nominal_rps
            logger.debug(f'Testing closed loop velocity control at {nominal_rps} rounds/s...')

            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis_ctx.handle.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            axis_ctx.handle.controller.input_vel = 0

            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
            axis_ctx.handle.controller.config.vel_limit = nominal_vel
            axis_ctx.handle.controller.config.vel_limit_tolerance = 2
            axis_ctx.handle.controller.input_vel = nominal_vel

            data = record_log(lambda: [axis_ctx.handle.encoder.vel_estimate, axis_ctx.handle.encoder.pos_estimate], duration=5.0)

            test_assert_no_error(axis_ctx)
            test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
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
            axis_ctx.handle.controller.input_vel = 0 # turn off velocity feed-forward
            axis_ctx.handle.controller.config.vel_limit = 5.0 # max 5 rps
            axis_ctx.handle.encoder.set_linear_count(0)

            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

            # Test small position changes
            test_pos = 5000 / float(enc_ctx.yaml['cpr'])
            axis_ctx.handle.controller.input_pos = test_pos
            time.sleep(0.3)
            test_assert_no_error(axis_ctx)
            test_assert_eq(axis_ctx.handle.encoder.pos_estimate, test_pos, range=0.4*test_pos) # large range needed because of cogging torque
            axis_ctx.handle.controller.input_pos = -1 * test_pos
            time.sleep(0.3)
            test_assert_no_error(axis_ctx)
            test_assert_eq(axis_ctx.handle.encoder.pos_estimate, -1 * test_pos, range=0.4*test_pos)
            
            axis_ctx.handle.controller.input_pos = 0
            time.sleep(0.3)

            nominal_vel = 5.0
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

    Note: If this test fails then try to run it at a DC voltage of 24V.
    Ibus seems to be more noisy/sensitive at lower DC voltages.
    """
    def get_test_cases(self, testrig: TestRig):
        return testrig.get_closed_loop_combos()

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        with SafeTerminator(logger, axis_ctx):
            nominal_rps = 15.0
            max_current = 20.0
        
            # Accept a bit of noise on Ibus
            axis_ctx.parent.handle.config.dc_max_negative_current = -0.5

            logger.debug(f'Brake control test from {nominal_rps} rounds/s...')
            
            axis_ctx.handle.controller.config.vel_limit = 25.0
            axis_ctx.handle.motor.config.current_lim = max_current
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis_ctx.handle.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
            # accelerate...
            axis_ctx.handle.controller.input_vel = nominal_rps
            time.sleep(1.0)
            test_assert_no_error(axis_ctx)

            # ... and brake
            axis_ctx.handle.controller.input_vel = 0
            time.sleep(1.0)
            test_assert_no_error(axis_ctx)


            logger.debug(f'Brake control test with brake resistor disabled')
            # once more, but this time without brake resistor
            axis_ctx.parent.handle.config.enable_brake_resistor = False
            # accelerate...
            axis_ctx.handle.controller.input_vel = nominal_rps
            time.sleep(1.0)
            test_assert_no_error(axis_ctx)

            # ... and brake
            axis_ctx.parent.handle.config.dc_max_negative_current = -0.2
            axis_ctx.handle.controller.input_vel = 10 # this should fail almost instantaneously
            time.sleep(0.1)
            test_assert_eq(axis_ctx.parent.handle.error, ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT)
            test_assert_eq(axis_ctx.handle.motor.error & MOTOR_ERROR_SYSTEM_LEVEL, MOTOR_ERROR_SYSTEM_LEVEL)
            test_assert_eq(axis_ctx.handle.error, 0)

            # Do test again with wrong brake resistance setting
            logger.debug(f'Brake control test with brake resistor = 100')
            axis_ctx.parent.handle.clear_errors()
            time.sleep(1.0)
            axis_ctx.parent.handle.config.brake_resistance = 100
            axis_ctx.parent.handle.config.dc_max_negative_current = -0.5
            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

            # accelerate...
            axis_ctx.handle.controller.input_vel = nominal_rps
            time.sleep(1.0)
            test_assert_no_error(axis_ctx)

            # ... and brake
            axis_ctx.handle.controller.input_vel = 0
            time.sleep(1.0) # expect DC_BUS_OVER_REGEN_CURRENT
            time.sleep(0.1)
            test_assert_eq(axis_ctx.parent.handle.error, ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT)
            test_assert_eq(axis_ctx.handle.motor.error & MOTOR_ERROR_SYSTEM_LEVEL, MOTOR_ERROR_SYSTEM_LEVEL)
            test_assert_eq(axis_ctx.handle.error, 0)




class TestVelLimitInTorqueControl(TestClosedLoopControlBase):
    """
    Ensures that the current setpoint in torque control is always within the
    parallelogram that arises from -Ilim, +Ilim, vel_limit and vel_gain.
    """
    def get_test_cases(self, testrig: TestRig):
        return testrig.get_closed_loop_combos()

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        with SafeTerminator(logger, axis_ctx):
            max_rps = 20.0
            max_vel = max_rps
            absolute_max_vel = max_vel * 1.2
            max_current = 30.0
            torque_constant = 0.0305 #correct for 5065 motor

            axis_ctx.handle.controller.config.vel_gain /= 10 # reduce the slope to make it easier to see what's going on
            vel_gain = axis_ctx.handle.controller.config.vel_gain
            direction = axis_ctx.handle.encoder.config.direction
            logger.debug(f'vel gain is {vel_gain}')

            axis_ctx.handle.controller.config.vel_limit = max_vel
            axis_ctx.handle.controller.config.vel_limit_tolerance = inf # disable hard limit on velocity
            axis_ctx.handle.motor.config.current_lim = max_current
            axis_ctx.handle.motor.config.torque_constant = torque_constant
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL

            # Returns the expected limited setpoint for a given velocity and current
            def get_expected_setpoint(input_setpoint, velocity):
                return clamp(clamp(input_setpoint / torque_constant, (velocity + max_vel) * -vel_gain / torque_constant, (velocity - max_vel) * -vel_gain / torque_constant), -max_current, max_current) * direction

            def data_getter():
                # sample velocity twice to avoid systematic bias
                velocity0 = axis_ctx.handle.encoder.vel_estimate
                current_setpoint = axis_ctx.handle.motor.current_control.Iq_setpoint
                velocity1 = axis_ctx.handle.encoder.vel_estimate
                velocity = ((velocity0 + velocity1) / 2)
                # Abort immediately if the absolute limits are exceeded
                test_assert_within(current_setpoint, -max_current, max_current)
                test_assert_within(velocity, -absolute_max_vel, absolute_max_vel)
                return input_torque, velocity, current_setpoint, get_expected_setpoint(input_torque, velocity)

            axis_ctx.handle.controller.input_torque = input_torque = 0.0
            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

            # Move the system around its operating envelope
            axis_ctx.handle.controller.input_torque = input_torque = 2.0 * torque_constant
            dataA = record_log(data_getter, duration=1.0)
            axis_ctx.handle.controller.input_torque = input_torque = -2.0 * torque_constant
            dataA = np.concatenate([dataA, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_torque = input_torque = 4.0 * torque_constant
            dataA = np.concatenate([dataA, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_torque = input_torque = -4.0 * torque_constant
            dataA = np.concatenate([dataA, record_log(data_getter, duration=1.0)])

            # Shrink the operating envelope while motor is moving faster than the envelope allows
            max_rps = 5.0
            max_vel = max_rps
            axis_ctx.handle.controller.config.vel_limit = max_vel

            # Move the system around its operating envelope
            axis_ctx.handle.controller.input_torque = input_torque = 2.0 * torque_constant
            dataB = record_log(data_getter, duration=1.0)
            axis_ctx.handle.controller.input_torque = input_torque = -2.0 * torque_constant
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_torque = input_torque = 4.0 * torque_constant
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_torque = input_torque = -4.0 * torque_constant
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])

            # Try the shrink maneuver again at positive velocity
            axis_ctx.handle.controller.config.vel_limit = 20.0
            axis_ctx.handle.controller.input_torque = 4.0 * torque_constant
            time.sleep(0.5)
            axis_ctx.handle.controller.config.vel_limit = max_vel

            axis_ctx.handle.controller.input_torque = input_torque = 2.0 * torque_constant
            dataB = np.concatenate([dataB, record_log(data_getter, duration=1.0)])

            test_assert_no_error(axis_ctx)

            axis_ctx.handle.requested_state=1
            
            test_curve_fit(dataA[:,(0,3)], dataA[:,4], max_mean_err=0.02, inlier_range=0.05, max_outliers=len(dataA[:,0]*0.01))
            test_curve_fit(dataB[:,(0,3)], dataB[:,4], max_mean_err=0.1, inlier_range=0.3, max_outliers=len(dataB[:,0])*0.01)

class TestTorqueLimit(TestClosedLoopControlBase):
    """
    Checks that the torque limit is respected in position, velocity, and torque control modes
    """
    def get_test_cases(self, testrig: TestRig):
        return testrig.get_closed_loop_combos()

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        with SafeTerminator(logger, axis_ctx):
            max_rps = 15.0
            max_vel = max_rps
            max_current = 30.0
            max_torque = 0.1 # must be less than max_current * torque_constant.
            torque_constant = axis_ctx.handle.motor.config.torque_constant

            test_pos = 5
            test_vel = 10
            test_torque = 0.5

            axis_ctx.handle.controller.config.vel_limit = max_vel
            axis_ctx.handle.motor.config.current_lim = max_current
            axis_ctx.handle.motor.config.torque_lim = inf #disable torque limit
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

            def data_getter():
                current_setpoint = axis_ctx.handle.motor.current_control.Iq_setpoint
                torque_setpoint = current_setpoint * torque_constant
                torque_limit = axis_ctx.handle.motor.config.torque_lim
                # Abort immediately if the absolute limits are exceeded
                test_assert_within(current_setpoint, -max_current, max_current)
                test_assert_within(torque_setpoint, -torque_limit, torque_limit)
                return max_current, current_setpoint, torque_limit, torque_setpoint

            # begin test
            axis_ctx.handle.motor.config.torque_lim = max_torque
            request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

            # step input positions
            logger.debug('input_pos step test')
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis_ctx.handle.controller.input_pos = test_pos
            dataPos = record_log(data_getter, duration=1.0)
            axis_ctx.handle.controller.input_pos = -test_pos
            dataPos = np.concatenate([dataPos, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_pos = test_pos
            dataPos = np.concatenate([dataPos, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_pos = -test_pos
            dataPos = np.concatenate([dataPos, record_log(data_getter, duration=1.0)])
            time.sleep(0.5)

            test_assert_no_error(axis_ctx)

            # step input velocities
            logger.debug('input_vel step test')
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis_ctx.handle.controller.input_vel = test_vel
            dataVel = record_log(data_getter, duration=1.0)
            axis_ctx.handle.controller.input_vel = -test_vel
            dataVel = np.concatenate([dataVel, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_vel = test_vel
            dataVel = np.concatenate([dataVel, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_vel = -test_vel
            dataVel = np.concatenate([dataVel, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_vel = 0
            time.sleep(0.5)

            # step input torques
            logger.debug('input_torque step test')
            axis_ctx.handle.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            axis_ctx.handle.controller.input_torque = test_torque
            dataTq = record_log(data_getter, duration=1.0)
            axis_ctx.handle.controller.input_torque = -test_torque
            dataTq = np.concatenate([dataTq, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_torque = test_torque
            dataTq = np.concatenate([dataTq, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_torque = -test_torque
            dataTq = np.concatenate([dataTq, record_log(data_getter, duration=1.0)])
            axis_ctx.handle.controller.input_torque = 0
            time.sleep(0.5)

            # did we pass?

            test_assert_no_error(axis_ctx)

            axis_ctx.handle.requested_state=1
tests = [
    TestClosedLoopControl(),
    TestRegenProtection(),
    TestVelLimitInTorqueControl(),
    TestTorqueLimit()
]

if __name__ == '__main__':
    test_runner.run(tests)
