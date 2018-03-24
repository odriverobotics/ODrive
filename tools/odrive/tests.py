
import subprocess
import shlex
import math
import time
import sys
import odrive.discovery
from odrive.enums import *

import abc
ABC = abc.ABC

class TestFailed(Exception):
    def __init__(self, message):
        Exception.__init__(self, message)

def test_assert_eq(observed, expected, range=None, accuracy=None):
    if range is None and accuracy is None and observed != expected:
        raise TestFailed("value mismatch: expected {} but observed {}".format(expected, observed))
    if not range is None and ((observed < expected - range) or (observed > expected + range)):
        raise TestFailed("value out of range: expected {}+-{} but observed {}".format(expected, range, observed))
    elif not accuracy is None and ((observed < expected * (1 - accuracy)) or (observed > expected * (1 + accuracy))):
        raise TestFailed("value out of range: expected {}+-{}% but observed {}".format(expected, accuracy*100.0, observed))

def run(command_line, logger, timeout=None):
    """
    Runs a shell command in the current directory
    """
    logger.debug("invoke: " + command_line)
    cmd = shlex.split(command_line)
    result = subprocess.run(cmd, timeout=timeout,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT)
    if result.returncode != 0:
        logger.error(result.stdout.decode(sys.stdout.encoding))
        raise TestFailed("command {} failed".format(command_line))

def rediscover(odrv_yaml):
    """
    Connects to the ODrive indicated by odrv_yaml
    """
    odrv = odrive.discovery.find_any(path="usb", serial_number=odrv_yaml['serial-number'], timeout=10)
    odrv_yaml['odrv'] = odrv
    for axis_idx, axis_yaml in enumerate(odrv_yaml['axes']):
        axis_yaml['axis'] = odrv.__dict__['axis{}'.format(axis_idx)]
    return odrv

class ODriveTest(ABC):
    """
    Tests inheriting from this class get full ownership of the ODrive
    being tested. However no guarantees are made for the mechanical
    state of the axes.
    """
    @abc.abstractmethod
    def run_test(self, odrv, odrv_config, logger):
        pass

class AxisTest(ABC):
    """
    Tests inheriting from this class get ownership of one axis of
    an ODrive. If the axis is mechanically coupled to another
    axis, the other axis is guaranteed to be disabled (high impedance)
    during this test.
    """
    @abc.abstractmethod
    def run_test(self, axis, axis_config, logger):
        pass

class DualAxisTest(ABC):
    """
    Tests using this scope get ownership of two axes that are mechanically
    coupled.
    """
    @abc.abstractmethod
    def run_test(self, axis0, axis0_config, axis1, axis1_config, logger):
        pass

class TestFlashAndErase(ODriveTest):
    def run_test(self, odrv, odrv_config, logger):
        run("make flash PROGRAMMER='" + odrv_config['programmer'] + "'", logger, timeout=20)
        # FIXME: device does not reboot correctly after erasing config this way
        #run("make erase_config PROGRAMMER='" + test_rig.programmer + "'", timeout=10)

        logger.debug("waiting for ODrive...")
        odrv = rediscover(odrv_config)
        # ensure the correct odrive is returned
        test_assert_eq(format(odrv.serial_number, 'x').upper(), odrv_config['serial-number'])

        # erase configuration and reboot
        logger.debug("erasing old configuration...")
        odrv.erase_configuration()
        #time.sleep(0.1)
        try:
            # FIXME: sometimes the device does not reappear after this ("no response - probably incompatible")
            # this is a firmware issue since it persists when unplugging/replugging
            # but goes away when power cycling the device
            odrv.reboot()
        except odrive.protocol.ChannelBrokenException:
            pass # this is expected
        time.sleep(0.5)

class TestSetup(ODriveTest):
    """
    Preconditions: ODrive is unconfigured and just rebooted
    """
    def run_test(self, odrv, odrv_config, logger):
        odrv = rediscover(odrv_config)

        # initial protocol tests and setup
        logger.debug("setting up ODrive...")
        odrv.config.enable_uart = True
        test_assert_eq(odrv.config.enable_uart, True)
        odrv.config.enable_uart = False
        test_assert_eq(odrv.config.enable_uart, False)
        odrv.config.brake_resistance = 1.0
        test_assert_eq(odrv.config.brake_resistance, 1.0)
        odrv.config.brake_resistance = odrv_config['brake-resistance']
        test_assert_eq(odrv.config.brake_resistance, odrv_config['brake-resistance'], accuracy=0.01)

        # firmware has 1500ms startup delay
        time.sleep(2)

        logger.debug("ensure we're in idle state")
        test_assert_eq(odrv.axis0.current_state, AXIS_STATE_IDLE)
        test_assert_eq(odrv.axis1.current_state, AXIS_STATE_IDLE)

def request_state(axis, state, expect_success=True):
    axis.requested_state = state
    time.sleep(0.001)
    if expect_success:
        test_assert_eq(axis.current_state, state)
    else:
        test_assert_eq(axis.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis.error, AXIS_ERROR_INVALID_STATE)
        axis.error = AXIS_ERROR_NO_ERROR # reset error

class TestMotorCalibration(AxisTest):
    """
    Tests motor calibration.
    The calibration results are compared against well known test rig values.
    Preconditions: The motor must be uncalibrated.
    Postconditions: The motor will be calibrated after this test.
    """
    def run_test(self, axis, axis_config, logger):
        logger.debug("try to enter closed loop control (should be rejected)")
        request_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL, expect_success=False)

        logger.debug("try to start encoder index search (should be rejected)")
        request_state(axis, AXIS_STATE_ENCODER_INDEX_SEARCH, expect_success=False)

        logger.debug("try to start encoder offset calibration (should be rejected)")
        request_state(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, expect_success=False)

        logger.debug("motor calibration (takes about 4.5 seconds)")
        axis.motor.config.pole_pairs = axis_config['motor-pole-pairs']
        request_state(axis, AXIS_STATE_MOTOR_CALIBRATION)
        time.sleep(6)
        test_assert_eq(axis.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis.error, AXIS_ERROR_NO_ERROR)
        test_assert_eq(axis.motor.config.phase_resistance, axis_config['motor-phase-resistance'], accuracy=0.1)
        test_assert_eq(axis.motor.config.phase_inductance, axis_config['motor-phase-inductance'], accuracy=0.5)
        axis.motor.config.pre_calibrated = True

class TestEncoderOffsetCalibration(AxisTest):
    """
    Tests encoder offset calibration.
    Preconditions: The encoder must be non-ready.
    Postconditions: The encoder will be ready after this test.
    """
    def run_test(self, axis, axis_config, logger):
        logger.debug("try to enter closed loop control (should be rejected)")
        request_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL, expect_success=False)

        logger.debug("encoder offset calibration (takes about 9.5 seconds)")
        axis.encoder.config.cpr = axis_config['encoder-cpr'] # TODO: test setting a wrong CPR
        request_state(axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
        # TODO: ensure the encoder calibration doesn't do crap
        time.sleep(11)
        test_assert_eq(axis.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis.error, AXIS_ERROR_NO_ERROR)
        test_assert_eq(axis.motor.config.direction, axis_config['motor-direction'])
        axis.encoder.config.pre_calibrated = True

class TestClosedLoopControl(AxisTest):
    """
    Tests closed loop position control and velocity control
    and verifies that the sensorless estimator works
    Precondition: The axis is calibrated and ready for closed loop control
    """
    def run_test(self, axis, axis_config, logger):
        logger.debug("closed loop control: test tiny position changes")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.001)
        test_assert_eq(axis.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.1) # give the PLL some time to settle
        test_assert_eq(axis.encoder.pll_pos, 0, range=300)
        axis.controller.set_pos_setpoint(1000, 0, 0)
        time.sleep(0.5)
        test_assert_eq(axis.encoder.pll_pos, 1000, range=200)
        axis.controller.set_pos_setpoint(-1000, 0, 0)
        time.sleep(0.5)
        test_assert_eq(axis.encoder.pll_pos, -1000, range=200)

        logger.debug("closed loop control: test vel_limit")
        axis.controller.set_pos_setpoint(50000, 0, 0)
        axis.controller.config.vel_limit = 40000
        time.sleep(0.3)
        test_assert_eq(axis.encoder.pll_vel, 40000, range=4000)
        expected_sensorless_estimation = 40000 * 2 * math.pi / axis_config['encoder-cpr'] * axis_config['motor-pole-pairs']
        test_assert_eq(axis.sensorless_estimator.pll_vel, expected_sensorless_estimation, range=50)
        time.sleep(3)
        test_assert_eq(axis.encoder.pll_vel, 0, range=1000)

class TestStoreAndReboot(ODriveTest):
    """
    Stores the current configuration to NVM and reboots.
    """
    def run_test(self, odrv, odrv_config, logger):
        logger.debug("storing configuration and rebooting...")
        odrv.save_configuration()
        try:
            odrv.reboot()
        except odrive.protocol.ChannelBrokenException:
            pass # this is expected
        time.sleep(2)

        odrv = rediscover(odrv_config)

        logger.debug("verifying configuration after reboot...")
        test_assert_eq(odrv.config.brake_resistance, odrv_config['brake-resistance'], accuracy=0.01)
        for axis_config in odrv_config['axes']:
            axis = axis_config['axis']
            test_assert_eq(axis.encoder.config.cpr, axis_config['encoder-cpr'])
            test_assert_eq(axis.motor.config.phase_resistance, axis_config['motor-phase-resistance'], accuracy=0.1)
            test_assert_eq(axis.motor.config.phase_inductance, axis_config['motor-phase-inductance'], accuracy=0.5)
