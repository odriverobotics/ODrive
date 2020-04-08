
import test_runner

import time
from math import pi
import os

from fibre.utils import Logger
from test_runner import AxisTestContext, MotorTestContext, EncoderTestContext, test_assert_eq, test_assert_no_error, request_state, program_teensy
from odrive.enums import *

def modpm(val, range):
    return ((val + (range / 2)) % range) - (range / 2)


class TestMotorCalibration():
    """
    Runs the motor calibration (phase inductance and phase resistance measurement)
    and checks if the measurements match the expectation.
    """

    def is_compatible(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext):
        return axis_ctx.yaml == motor_ctx.yaml['name'] # check if connected

    def run_test(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, logger: Logger):
        # reset old calibration values
        axis_ctx.handle.motor.config.phase_resistance = 0.0
        axis_ctx.handle.motor.config.phase_inductance = 0.0
        axis_ctx.handle.motor.config.pre_calibrated = False

        axis_ctx.handle.clear_errors()

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

    def is_compatible(self, axis_ctx: AxisTestContext):
        return axis_ctx.yaml == 'floating'

    def run_test(self, axis_ctx: AxisTestContext, logger: Logger):
        axis = axis_ctx.handle

        # reset old calibration values
        axis_ctx.handle.motor.config.phase_resistance = 0.0
        axis_ctx.handle.motor.config.phase_inductance = 0.0
        axis_ctx.handle.motor.config.pre_calibrated = False

        axis_ctx.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_MOTOR_CALIBRATION)
        time.sleep(6)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis_ctx.handle.error, errors.axis.ERROR_MOTOR_FAILED)
        test_assert_eq(axis_ctx.handle.motor.error, errors.motor.ERROR_PHASE_RESISTANCE_OUT_OF_RANGE)


class TestEncoderDirFind():
    """
    Runs the encoder index search.
    """

    def is_compatible(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, enc_ctx: EncoderTestContext):
        return (axis_ctx.yaml == motor_ctx.yaml['name']) and (axis_ctx.num == enc_ctx.num) # check if connected

    def run_test(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, enc_ctx: EncoderTestContext, logger: Logger):
        axis = axis_ctx.handle
        # TODO: read teensy config from YAML file
        hexfile = 'encoder_pass_through.ino.hex'
        program_teensy(os.path.join(os.path.dirname(__file__), hexfile), 26, logger)
        time.sleep(1.0) # wait for PLLs to stabilize

        # Set motor calibration values
        axis_ctx.handle.motor.config.phase_resistance = float(motor_ctx.yaml['phase-resistance'])
        axis_ctx.handle.motor.config.phase_inductance = float(motor_ctx.yaml['phase-inductance'])
        axis_ctx.handle.motor.config.pre_calibrated = True

        # Set calibration settings
        axis_ctx.handle.motor.config.direction = 0
        axis_ctx.handle.config.calibration_lockin.vel = 12.566 # 2 electrical revolutions per second

        axis_ctx.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_ENCODER_DIR_FIND)

        time.sleep(4) # actual calibration takes 3 seconds
        
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        test_assert_eq(axis_ctx.handle.motor.config.direction in [-1, 1], True)


class TestEncoderOffsetCalibration():
    """
    Runs the encoder index search.
    """

    def is_compatible(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, enc_ctx: EncoderTestContext):
        return (axis_ctx.yaml == motor_ctx.yaml['name']) and (axis_ctx.num == enc_ctx.num) # check if connected

    def run_test(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, enc_ctx: EncoderTestContext, logger: Logger):
        axis = axis_ctx.handle
        # TODO: read teensy config from YAML file
        hexfile = 'encoder_pass_through.ino.hex'
        program_teensy(os.path.join(os.path.dirname(__file__), hexfile), 26, logger)
        time.sleep(1.0) # wait for PLLs to stabilize

        # Set motor calibration values
        axis_ctx.handle.motor.config.phase_resistance = float(motor_ctx.yaml['phase-resistance'])
        axis_ctx.handle.motor.config.phase_inductance = float(motor_ctx.yaml['phase-inductance'])
        axis_ctx.handle.motor.config.pre_calibrated = True

        # Set calibration settings
        axis_ctx.handle.motor.config.direction = 0
        enc_ctx.handle.config.use_index = False
        enc_ctx.handle.config.calib_scan_omega = 12.566 # 2 electrical revolutions per second
        enc_ctx.handle.config.calib_scan_distance = 50.265 # 8 revolutions

        axis_ctx.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_ENCODER_OFFSET_CALIBRATION)

        time.sleep(9) # actual calibration takes 8 seconds
        
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        test_assert_eq(enc_ctx.handle.is_ready, True)
        test_assert_eq(axis_ctx.handle.motor.config.direction in [-1, 1], True)


class TestEncoderIndexSearch():
    """
    Runs the encoder index search.
    The index pin is triggered manually after three seconds from the testbench
    host's GPIO.
    """

    def is_compatible(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, enc_ctx: EncoderTestContext):
        return (axis_ctx.yaml == motor_ctx.yaml['name']) and (axis_ctx.num == enc_ctx.num) # check if connected

    def run_test(self, axis_ctx: AxisTestContext, motor_ctx: MotorTestContext, enc_ctx: EncoderTestContext, logger: Logger):
        axis = axis_ctx.handle
        # TODO: read teensy config from YAML file
        hexfile = 'encoder_pass_through.ino.hex'
        program_teensy(os.path.join(os.path.dirname(__file__), hexfile), 26, logger)
        time.sleep(1.0) # wait for PLLs to stabilize

        # Set motor calibration values
        axis_ctx.handle.motor.config.phase_resistance = float(motor_ctx.yaml['phase-resistance'])
        axis_ctx.handle.motor.config.phase_inductance = float(motor_ctx.yaml['phase-inductance'])
        axis_ctx.handle.motor.config.pre_calibrated = True

        # Set calibration settings
        axis_ctx.handle.config.calibration_lockin.vel = 12.566 # 2 electrical revolutions per second

        axis_ctx.handle.clear_errors()

        # run test
        request_state(axis_ctx, AXIS_STATE_ENCODER_INDEX_SEARCH)

        time.sleep(3)

        test_assert_eq(enc_ctx.handle.index_found, False)
        with open("/sys/class/gpio/gpio{}/value".format(20), "w") as gpio:
            gpio.write("0")
        time.sleep(0.1)
        with open("/sys/class/gpio/gpio{}/value".format(20), "w") as gpio:
            gpio.write("1")
        test_assert_eq(enc_ctx.handle.index_found, True)
        
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)

        test_assert_eq(enc_ctx.handle.shadow_count, 0.0, range=20)
        test_assert_eq(enc_ctx.handle.count_in_cpr, 0.0, range=20)
        test_assert_eq(enc_ctx.handle.pos_estimate, 0.0, range=20)
        test_assert_eq(enc_ctx.handle.pos_cpr, 0.0, range=20)
        test_assert_eq(enc_ctx.handle.pos_abs, 0.0, range=20)


if __name__ == '__main__':
    test_runner.run([
        TestMotorCalibration(),
        TestDisconnectedMotorCalibration(),
        TestEncoderDirFind(),
        TestEncoderOffsetCalibration(),
        TestEncoderIndexSearch()
    ])
