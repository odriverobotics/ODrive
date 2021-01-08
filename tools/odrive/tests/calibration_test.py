
import test_runner

import time
from math import pi
import os

from fibre.utils import Logger
from test_runner import *
from odrive.enums import *


class TestMotorCalibration():
    """
    Runs the motor calibration (phase inductance and phase resistance measurement)
    and checks if the measurements match the expectation.
    """

    def get_test_cases(self, testrig: TestRig):
        """Returns all axes that are connected to a motor, along with the corresponding motor(s)"""
        for axis in testrig.get_components(ODriveAxisComponent):
            for motor, tf in testrig.get_connected_components({'phases': axis}, MotorComponent):
                yield (axis, motor, tf)

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, logger: Logger):
        # reset old calibration values

        axis_ctx.parent.erase_config_and_reboot()
        #if axis_ctx.handle.encoder.config.mode != ENCODER_MODE_INCREMENTAL:
        #    axis_ctx.handle.encoder.config.mode = ENCODER_MODE_INCREMENTAL
        #    axis_ctx.parent.save_config_and_reboot()

        axis_ctx.handle.motor.config.phase_resistance = 0.0
        axis_ctx.handle.motor.config.phase_inductance = 0.0
        axis_ctx.handle.motor.config.pre_calibrated = False
        axis_ctx.handle.config.enable_watchdog = False
        axis_ctx.parent.handle.config.dc_max_negative_current = -1.0

        axis_ctx.parent.handle.clear_errors()

        # run calibration
        request_state(axis_ctx, AXIS_STATE_MOTOR_CALIBRATION)
        time.sleep(6)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        # check if measurements match expectation
        test_assert_eq(axis_ctx.handle.motor.config.phase_resistance, float(motor_ctx.yaml['phase-resistance']), accuracy=0.2)
        test_assert_eq(axis_ctx.handle.motor.config.phase_inductance, float(motor_ctx.yaml['phase-inductance']), accuracy=0.5)
        test_assert_eq(axis_ctx.handle.motor.is_calibrated, True)


class TestDisconnectedMotorCalibration():
    """
    Tests if the motor calibration fails as expected if the phases are floating.
    """

    def get_test_cases(self, testrig: TestRig):
        """Returns all axes that are disconnected"""
        for axis in testrig.get_components(ODriveAxisComponent):
            if axis.yaml == 'floating':
                yield (axis, None)

    def run_test(self, axis_ctx: ODriveAxisComponent, logger: Logger):
        axis = axis_ctx.handle

        # reset old calibration values
        axis_ctx.handle.motor.config.phase_resistance = 0.0
        axis_ctx.handle.motor.config.phase_inductance = 0.0
        axis_ctx.handle.motor.config.pre_calibrated = False

        axis_ctx.parent.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_MOTOR_CALIBRATION)
        time.sleep(6)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis_ctx.handle.motor.error, MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE)


class TestEncoderDirFind():
    """
    Runs the encoder index search.
    """

    def get_test_cases(self, testrig: TestRig):
        return testrig.get_closed_loop_combos(init=False)

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        axis = axis_ctx.handle
        time.sleep(1.0) # wait for PLLs to stabilize

        # Set motor calibration values
        axis_ctx.handle.motor.config.phase_resistance = float(motor_ctx.yaml['phase-resistance'])
        axis_ctx.handle.motor.config.phase_inductance = float(motor_ctx.yaml['phase-inductance'])
        axis_ctx.handle.motor.config.pre_calibrated = True

        # Set calibration settings
        axis_ctx.handle.encoder.config.direction = 0
        axis_ctx.handle.config.calibration_lockin.vel = 12.566 # 2 electrical revolutions per second

        axis_ctx.parent.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_ENCODER_DIR_FIND)

        time.sleep(4) # actual calibration takes 3 seconds
        
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        test_assert_eq(axis_ctx.handle.encoder.config.direction in [-1, 1], True)


class TestEncoderOffsetCalibration():
    """
    Runs the encoder index search.
    """

    def get_test_cases(self, testrig: TestRig):
        return testrig.get_closed_loop_combos(init=False)

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, logger: Logger):
        axis = axis_ctx.handle
        time.sleep(1.0) # wait for PLLs to stabilize

        # Set motor calibration values
        axis_ctx.handle.motor.config.phase_resistance = float(motor_ctx.yaml['phase-resistance'])
        axis_ctx.handle.motor.config.phase_inductance = float(motor_ctx.yaml['phase-inductance'])
        axis_ctx.handle.motor.config.pre_calibrated = True

        # Set calibration settings
        axis_ctx.handle.encoder.config.direction = 0
        axis_ctx.handle.encoder.config.use_index = False
        axis_ctx.handle.encoder.config.calib_scan_omega = 12.566 # 2 electrical revolutions per second
        axis_ctx.handle.encoder.config.calib_scan_distance = 50.265 # 8 revolutions

        axis_ctx.parent.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_ENCODER_OFFSET_CALIBRATION)

        time.sleep(9.1) # actual calibration takes 9.0 seconds
        
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        test_assert_eq(axis_ctx.handle.encoder.is_ready, True)
        test_assert_eq(axis_ctx.handle.encoder.config.direction in [-1, 1], True)


class TestEncoderIndexSearch():
    """
    Runs the encoder index search.
    The index pin is triggered manually after three seconds from the testbench
    host's GPIO.
    """

    def get_test_cases(self, testrig: TestRig):
        for axis, motor, encoder, tf1 in testrig.get_closed_loop_combos(init=False):
            alternatives = []
            for z_gpio, tf2 in testrig.get_connected_components((axis.parent.encoders[axis.num].z, False), LinuxGpioComponent):
                alternatives.append((axis, motor, encoder, z_gpio, TestFixture.all_of(tf1, tf2)))
            yield AnyTestCase(*alternatives)

    def run_test(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, z_gpio: LinuxGpioComponent, logger: Logger):
        axis = axis_ctx.handle
        cpr = int(enc_ctx.yaml['cpr'])
        
        z_gpio.config(output=True)
        z_gpio.write(False)

        time.sleep(1.0) # wait for PLLs to stabilize

        # Set motor calibration values
        axis_ctx.handle.motor.config.phase_resistance = float(motor_ctx.yaml['phase-resistance'])
        axis_ctx.handle.motor.config.phase_inductance = float(motor_ctx.yaml['phase-inductance'])
        axis_ctx.handle.motor.config.pre_calibrated = True

        # Set calibration settings
        axis_ctx.handle.config.calibration_lockin.vel = 12.566 # 2 electrical revolutions per second

        axis_ctx.parent.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_ENCODER_INDEX_SEARCH)

        time.sleep(3)

        test_assert_eq(axis_ctx.handle.encoder.index_found, False)
        time.sleep(0.1)
        z_gpio.write(True)
        test_assert_eq(axis_ctx.handle.encoder.index_found, True)
        z_gpio.write(False)
        
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        test_assert_eq(axis_ctx.handle.encoder.shadow_count, 0.0, range=50)
        test_assert_eq(modpm(axis_ctx.handle.encoder.count_in_cpr, cpr), 0.0, range=50)
        test_assert_eq(axis_ctx.handle.encoder.pos_estimate, 0.0, range=50)
        test_assert_eq(modpm(axis_ctx.handle.encoder.pos_cpr_counts, cpr), 0.0, range=50)
        test_assert_eq(axis_ctx.handle.encoder.pos_abs, 0.0, range=50)

tests = [
    TestMotorCalibration(),
    TestDisconnectedMotorCalibration(),
    TestEncoderDirFind(),
    TestEncoderOffsetCalibration(),
    TestEncoderIndexSearch()
]

if __name__ == '__main__':
    test_runner.run(tests)
