from __future__ import print_function
import subprocess
import shlex
import math
import time
import sys
import threading
import fibre
import odrive
from odrive.enums import *
import odrive.utils
import numpy as np

import functools
print = functools.partial(print, flush=True)

import abc
ABC = abc.ABC


class PreconditionsNotMet(Exception):
    pass


class AxisTestContext():
    def __init__(self, name: str, yaml: dict, odrv_ctx: ODriveTestContext):
        self.handle = None
        self.yaml = yaml
        self.name = name
        self.lock = threading.Lock()
        self.odrv_ctx = odrv_ctx

def get_errors(axis_ctx: AxisTestContext):
    errors = []
    if axis_ctx.handle.motor.error != 0:
        errors.append("motor failed with error 0x{:04X}".format(axis_ctx.handle.motor.error))
    if axis_ctx.handle.encoder.error != 0:
        errors.append("encoder failed with error 0x{:04X}".format(axis_ctx.handle.encoder.error))
    if axis_ctx.handle.sensorless_estimator.error != 0:
        errors.append("sensorless_estimator failed with error 0x{:04X}".format(axis_ctx.handle.sensorless_estimator.error))
    if axis_ctx.handle.error != 0:
        errors.append("axis failed with error 0x{:04X}".format(axis_ctx.handle.error))
    elif len(errors) > 0:
        errors.append("and by the way: axis reports no error even though there is one")
    return errors

def dump_errors(axis_ctx: AxisTestContext, logger):
    errors = get_errors(axis_ctx)
    if len(errors):
        logger.error("errors on " + axis_ctx.name)
        for error in errors:
            logger.error(error)

def clear_errors(axis_ctx: AxisTestContext):
    axis_ctx.handle.error = 0
    axis_ctx.handle.encoder.error = 0
    axis_ctx.handle.motor.error = 0
    axis_ctx.handle.sensorless_estimator.error = 0

def test_assert_no_error(axis_ctx: AxisTestContext):
    errors = get_errors(axis_ctx)
    if len(errors) > 0:
        raise TestFailed("\n".join(errors))

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

def request_state(axis_ctx: AxisTestContext, state, expect_success=True):
    axis_ctx.handle.requested_state = state
    time.sleep(0.001)
    if expect_success:
        test_assert_eq(axis_ctx.handle.current_state, state)
    else:
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis_ctx.handle.error, AXIS_ERROR_INVALID_STATE)
        axis_ctx.handle.error = AXIS_ERROR_NONE # reset error

def set_limits(axis_ctx: AxisTestContext, logger, vel_limit=20000, current_limit=10):
    """
    Sets the velocity and current limits for the axis, subject to the following constraints:
      - the arguments given to this function are not exceeded
      - max motor current is not exceeded
      - max brake resistor power divided by two is not exceeded (here velocity takes precedence over current)
    """
    max_rpm = vel_limit / axis_ctx.yaml['encoder-cpr'] * 60
    max_emf_voltage = max_rpm / axis_ctx.yaml['motor-kv']
    max_brake_power = axis_ctx.odrv_ctx.yaml['max-brake-power'] / 2 * 0.8 # 20% safety margin
    max_motor_current = max_brake_power / max_emf_voltage
    logger.debug("velocity limit = {} => V_emf = {:.3}V, I_lim = {:.3}A".format(vel_limit, max_emf_voltage, max_motor_current))

    # Bound current limit based on the motor's current limit and the brake resistor current limit
    current_limit = min(current_limit, axis_ctx.yaml['motor-max-current'], max_motor_current)
    # TODO: set as an atomic operation
    axis_ctx.handle.motor.config.current_lim = current_limit
    axis_ctx.handle.controller.config.vel_limit = vel_limit

def get_max_rpm(axis_ctx: AxisTestContext):

    # Calculate theoretical max velocity in rpm based on the nominal
    # V_bus and motor KV rating.
    # The KV-rating assumes square-waves on the motor phases (hexagonal space vector trajectory)
    # whereas the ODrive modulates the space vector around a circular trajectory.
    # See Fig 4.28 here: http://krex.k-state.edu/dspace/bitstream/handle/2097/1507/JamesMevey2009.pdf
    effective_bus_voltage = axis_ctx.odrv_ctx.yaml['vbus-voltage']
    effective_bus_voltage *= (2/math.sqrt(3)) / (4/math.pi) # roughtly 90%
    # The ODrive only goes to 80% modulation depth in order to save some time for the ADC measurements.
    # See FOC_current in motor.cpp.
    effective_bus_voltage *= 0.8

    # If we are using a higher bus voltage than rated: use rated voltage,
    # since that is an effective speed rating of the motor
    voltage_for_speed = min(effective_bus_voltage, axis_ctx.yaml['motor-max-voltage'])
    base_speed_rpm = voltage_for_speed * axis_ctx.yaml['motor-kv']

    #but don't go over encoder max rpm
    rated_rpm = min(base_speed_rpm, axis_ctx.yaml['encoder-max-rpm'])
    return rated_rpm

def get_sensorless_vel(axis_ctx: AxisTestContext, vel):
    return vel * 2 * math.pi / axis_ctx.yaml['encoder-cpr'] * axis_ctx.yaml['motor-pole-pairs']

class ODriveTest(ABC):
    """
    Tests inheriting from this class get full ownership of the ODrive
    being tested. However no guarantees are made for the mechanical
    state of the axes.
    The test can demand exclusive run time which means that the host will
    not run any other test at the same time. This can be used if the test
    invokes a command that's so lame that it can't run twice concurrently.
    """
    def __init__(self, exclusive=False):
        self._exclusive = exclusive
    def check_preconditions(self, odrv_ctx: ODriveTestContext, logger):
        pass
    @abc.abstractmethod
    def run_test(self, odrv_ctx: ODriveTestContext, logger):
        pass

class AxisTest(ABC):
    """
    Tests inheriting from this class get ownership of one axis of
    an ODrive. If the axis is mechanically coupled to another
    axis, the other axis is guaranteed to be disabled (high impedance)
    during this test.
    """
    def check_preconditions(self, axis_ctx: AxisTestContext, logger):
        test_assert_no_error(axis_ctx)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        if (abs(axis_ctx.handle.encoder.vel_estimate) > 100):
            logger.warn("axis still in motion, delaying 2 sec...")
            time.sleep(2)
        test_assert_eq(axis_ctx.handle.encoder.vel_estimate, 0, range=500)
        test_assert_eq(axis_ctx.odrv_ctx.handle.config.dc_bus_undervoltage_trip_level, axis_ctx.odrv_ctx.yaml['vbus-voltage'] * 0.85, accuracy=0.001)
        test_assert_eq(axis_ctx.odrv_ctx.handle.config.dc_bus_overvoltage_trip_level, axis_ctx.odrv_ctx.yaml['vbus-voltage'] * 1.08, accuracy=0.001)
        #test_assert_eq(axis_ctx.odrv_ctx.handle.config.dc_bus_undervoltage_trip_level, axis_ctx.odrv_ctx.yaml['vbus-voltage'] * 0.96, accuracy=0.001)
        #test_assert_eq(axis_ctx.odrv_ctx.handle.config.dc_bus_overvoltage_trip_level, axis_ctx.odrv_ctx.yaml['vbus-voltage'] * 1.04, accuracy=0.001)

    @abc.abstractmethod
    def run_test(self, axis_ctx: AxisTestContext, logger):
        pass

class DualAxisTest(ABC):
    """
    Tests using this scope get ownership of two axes that are mechanically
    coupled.
    """
    def check_preconditions(self, axis0_ctx: AxisTestContext, axis1_ctx: AxisTestContext, logger):
        test_assert_no_error(axis0_ctx)
        test_assert_no_error(axis1_ctx)
        test_assert_eq(axis0_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis1_ctx.handle.current_state, AXIS_STATE_IDLE)
        if (abs(axis0_ctx.handle.encoder.vel_estimate) > 100) or (abs(axis1_ctx.handle.encoder.vel_estimate) > 100):
            logger.warn("some axis still in motion, delaying 2 sec...")
            time.sleep(2)
        test_assert_eq(axis0_ctx.handle.encoder.vel_estimate, 0, range=500)
        test_assert_eq(axis1_ctx.handle.encoder.vel_estimate, 0, range=500)

    @abc.abstractmethod
    def run_test(self, axis0_ctx: AxisTestContext, axis1_ctx: AxisTestContext, logger):
        pass

class TestDiscoverAndGotoIdle(ODriveTest):
    def run_test(self, odrv_ctx: ODriveTestContext, logger):
        odrv_ctx.rediscover()
        clear_errors(odrv_ctx.axes[0])
        clear_errors(odrv_ctx.axes[1])
        request_state(odrv_ctx.axes[0], AXIS_STATE_IDLE)
        request_state(odrv_ctx.axes[1], AXIS_STATE_IDLE)

class TestFlashAndErase(ODriveTest):
    def __init__(self):
        ODriveTest.__init__(self, exclusive=True)
    def run_test(self, odrv_ctx: ODriveTestContext, logger):
        # Set board-version and compile
        with open("tup.config", mode="w") as tup_config:
            tup_config.write("CONFIG_STRICT=true\n")
            tup_config.write("CONFIG_BOARD_VERSION={}\n".format(odrv_ctx.yaml['board-version']))
        #exit(1)
        run("make", logger, timeout=10)
        run("make flash PROGRAMMER='" + odrv_ctx.yaml['programmer'] + "'", logger, timeout=20)
        # FIXME: device does not reboot correctly after erasing config this way
        #run("make erase_config PROGRAMMER='" + test_rig.programmer + "'", timeout=10)

        logger.debug("waiting for ODrive...")
        odrv_ctx.rediscover()
        # ensure the correct odrive is returned
        test_assert_eq(format(odrv_ctx.handle.serial_number, 'x').upper(), odrv_ctx.yaml['serial-number'])

        # erase configuration and reboot
        logger.debug("erasing old configuration...")
        odrv_ctx.handle.erase_configuration()
        #time.sleep(0.1)
        try:
            # FIXME: sometimes the device does not reappear after this ("no response - probably incompatible")
            # this is a firmware issue since it persists when unplugging/replugging
            # but goes away when power cycling the device
            odrv_ctx.handle.reboot()
        except fibre.ObjectLostError:
            pass # this is expected
        time.sleep(0.5)

class TestSetup(ODriveTest):
    """
    Preconditions: ODrive is unconfigured and just rebooted
    """
    def run_test(self, odrv_ctx: ODriveTestContext, logger):
        odrv_ctx.rediscover()

        # initial protocol tests and setup
        logger.debug("setting up ODrive...")
        odrv_ctx.handle.config.enable_uart = True
        test_assert_eq(odrv_ctx.handle.config.enable_uart, True)
        odrv_ctx.handle.config.enable_uart = False
        test_assert_eq(odrv_ctx.handle.config.enable_uart, False)
        odrv_ctx.handle.config.brake_resistance = 1.0
        test_assert_eq(odrv_ctx.handle.config.brake_resistance, 1.0)
        odrv_ctx.handle.config.brake_resistance = odrv_ctx.yaml['brake-resistance']
        test_assert_eq(odrv_ctx.handle.config.brake_resistance, odrv_ctx.yaml['brake-resistance'], accuracy=0.01)
        odrv_ctx.handle.config.dc_bus_undervoltage_trip_level = odrv_ctx.yaml['vbus-voltage'] * 0.85
        odrv_ctx.handle.config.dc_bus_overvoltage_trip_level = odrv_ctx.yaml['vbus-voltage'] * 1.08
        test_assert_eq(odrv_ctx.handle.config.dc_bus_undervoltage_trip_level, odrv_ctx.yaml['vbus-voltage'] * 0.85, accuracy=0.001)
        test_assert_eq(odrv_ctx.handle.config.dc_bus_overvoltage_trip_level, odrv_ctx.yaml['vbus-voltage'] * 1.08, accuracy=0.001)

        # firmware has 1500ms startup delay
        time.sleep(2)

        logger.debug("ensure we're in idle state")
        test_assert_eq(odrv_ctx.handle.axis0.current_state, AXIS_STATE_IDLE)
        test_assert_eq(odrv_ctx.handle.axis1.current_state, AXIS_STATE_IDLE)

class TestMotorCalibration(AxisTest):
    """
    Tests motor calibration.
    The calibration results are compared against well known test rig values.
    Preconditions: The motor must be uncalibrated.
    Postconditions: The motor will be calibrated after this test.
    """
    def check_preconditions(self, axis_ctx: AxisTestContext, logger):
        super(TestMotorCalibration, self).check_preconditions(axis_ctx, logger)
        test_assert_eq(axis_ctx.handle.motor.is_calibrated, False)

    def run_test(self, axis_ctx: AxisTestContext, logger):
        logger.debug("try to enter closed loop control (should be rejected)")
        request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL, expect_success=False)

        logger.debug("try to start encoder index search (should be rejected)")
        request_state(axis_ctx, AXIS_STATE_ENCODER_INDEX_SEARCH, expect_success=False)

        logger.debug("try to start encoder offset calibration (should be rejected)")
        request_state(axis_ctx, AXIS_STATE_ENCODER_OFFSET_CALIBRATION, expect_success=False)

        logger.debug("motor calibration (takes about 4.5 seconds)")
        axis_ctx.handle.motor.config.pole_pairs = axis_ctx.yaml['motor-pole-pairs']
        request_state(axis_ctx, AXIS_STATE_MOTOR_CALIBRATION)
        time.sleep(6)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)
        test_assert_eq(axis_ctx.handle.motor.config.phase_resistance, axis_ctx.yaml['motor-phase-resistance'], accuracy=0.2)
        test_assert_eq(axis_ctx.handle.motor.config.phase_inductance, axis_ctx.yaml['motor-phase-inductance'], accuracy=0.5)
        axis_ctx.handle.motor.config.pre_calibrated = True

class TestEncoderOffsetCalibration(AxisTest):
    """
    Tests encoder offset calibration.
    Preconditions: The encoder must be non-ready.
    Postconditions: The encoder will be ready after this test.
    """
    def __init__(self, pass_if_ready=False):
        AxisTest.__init__(self)
        self._pass_if_ready = pass_if_ready

    def check_preconditions(self, axis_ctx: AxisTestContext, logger):
        super(TestEncoderOffsetCalibration, self).check_preconditions(axis_ctx, logger)
        if not self._pass_if_ready:
            test_assert_eq(axis_ctx.handle.encoder.is_ready, False)

    def run_test(self, axis_ctx: AxisTestContext, logger):
        if (self._pass_if_ready and axis_ctx.handle.encoder.is_ready):
            logger.debug("encoder already ready, skipping this test")
            return

        logger.debug("try to enter closed loop control (should be rejected)")
        request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL, expect_success=False)

        logger.debug("encoder offset calibration (takes about 9.5 seconds)")
        axis_ctx.handle.encoder.config.cpr = axis_ctx.yaml['encoder-cpr'] # TODO: test setting a wrong CPR
        request_state(axis_ctx, AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
        # TODO: ensure the encoder calibration doesn't do crap
        time.sleep(11)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)
        test_assert_eq(axis_ctx.handle.motor.config.direction, axis_ctx.yaml['motor-direction'])
        axis_ctx.handle.encoder.config.pre_calibrated = True

class TestClosedLoopControl(AxisTest):
    """
    Tests closed loop position control and velocity control
    and verifies that the sensorless estimator works
    Precondition: The axis is calibrated and ready for closed loop control
    """
    def check_preconditions(self, axis_ctx: AxisTestContext, logger):
        super(TestClosedLoopControl, self).check_preconditions(axis_ctx, logger)
        test_assert_eq(axis_ctx.handle.motor.is_calibrated, True)
        test_assert_eq(axis_ctx.handle.encoder.is_ready, True)

    def run_test(self, axis_ctx: AxisTestContext, logger):
        logger.debug("closed loop control: test tiny position changes")
        axis_ctx.handle.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(0.001)
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.1) # give the PLL some time to settle
        init_pos = axis_ctx.handle.encoder.pos_estimate
        axis_ctx.handle.controller.set_pos_setpoint(init_pos+1000, 0, 0)
        time.sleep(0.5)
        test_assert_eq(axis_ctx.handle.encoder.pos_estimate, init_pos+1000, range=200)
        axis_ctx.handle.controller.set_pos_setpoint(init_pos-1000, 0, 0)
        time.sleep(0.5)
        test_assert_eq(axis_ctx.handle.encoder.pos_estimate, init_pos-1000, range=400)

        logger.debug("closed loop control: test vel_limit")
        axis_ctx.handle.controller.set_pos_setpoint(50000, 0, 0)
        axis_ctx.handle.controller.config.vel_limit = 40000
        time.sleep(0.3)
        test_assert_eq(axis_ctx.handle.encoder.vel_estimate, 40000, range=4000)
        expected_sensorless_estimation = 40000 * 2 * math.pi / axis_ctx.yaml['encoder-cpr'] * axis_ctx.yaml['motor-pole-pairs']
        test_assert_eq(axis_ctx.handle.sensorless_estimator.vel_estimate, expected_sensorless_estimation, range=50)
        time.sleep(3)
        test_assert_eq(axis_ctx.handle.encoder.vel_estimate, 0, range=1000)
        time.sleep(0.5)
        request_state(axis_ctx, AXIS_STATE_IDLE)

class TestHighVelocity(AxisTest):
    """
    Spins the motor up to it's max speed during a period of 10s.
    The commanded max speed is based on the motor's KV rating and nominal V_bus,
    however due to several factors the theoretical limit is about 72% of that.
    The test passes if the motor follows the commanded ramp closely up to 90% of
    the theoretical limit (and if no errors occur along the way).
    """
    def __init__(self, override_current_limit=None, load_current=0, brake=True):
        """
        param override_current_limit: If None, the test selects a current limit that is guaranteed
                                      not to fry the brake resistor. If you override the limit, you're
                                      on your own.
        """
        self._override_current_limit = override_current_limit
        self._load_current = load_current
        self._brake = brake

    def check_preconditions(self, axis_ctx: AxisTestContext, logger):
        # time.sleep(2.5) #delay in case load needs time to stop moving
        super(TestHighVelocity, self).check_preconditions(axis_ctx, logger)
        test_assert_eq(axis_ctx.handle.motor.is_calibrated, True)
        test_assert_eq(axis_ctx.handle.encoder.is_ready, True)

    def run_test(self, axis_ctx: AxisTestContext, logger):
        rated_limit = get_max_rpm(axis_ctx) / 60 * axis_ctx.yaml['encoder-cpr']
        expected_limit = rated_limit
        
        # TODO: remove the following two lines, but for now we want to stay away from the modulation depth limit
        expected_limit *= 0.6
        rated_limit = expected_limit

        # Add a 10% margin to account for 
        expected_limit *= 0.9

        logger.debug("rated max speed: {}, expected max speed: >= {}".format(rated_limit, expected_limit))
        #theoretical_limit = 100000
        
        # Set the current limit accordingly so we don't burn the brake resistor while slowing down
        if self._override_current_limit is None:
            set_limits(axis_ctx, logger, vel_limit=rated_limit, current_limit=50)
        else:
            axis_ctx.handle.motor.config.current_lim = self._override_current_limit
            axis_ctx.handle.controller.config.vel_limit = rated_limit
        axis_ctx.handle.controller.set_vel_setpoint(0, 0)
        request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

        logger.debug("Drive current {}A, Load current {}A".format(axis_ctx.handle.motor.config.current_lim, self._load_current))

        ramp_up_time = 15.0
        max_measured_vel = 0.0
        logger.debug("ramping to {} over {} s".format(rated_limit, ramp_up_time))
        t_0 = time.monotonic()
        last_print = t_0
        while True:
            ratio = (time.monotonic() - t_0) / ramp_up_time
            if ratio >= 1:
                break

            #TODO based on integrator gain and torque ramp rate
            expected_ramp_lag = 1.0 * (rated_limit / ramp_up_time)
            expected_lag = 0

            # While ramping up we want to remain within +-5% of the setpoint.
            # However we accept if we can only approach 80% of the theoretical limit.
            vel_setpoint = ratio * rated_limit
            expected_velocity = max(vel_setpoint - expected_lag, 0)
            vel_range = max(0.05*expected_velocity, max(expected_lag+expected_ramp_lag, 2000))
            if expected_velocity - vel_range > expected_limit:
                vel_range = expected_velocity - expected_limit

            # set and measure velocity
            axis_ctx.handle.controller.set_vel_setpoint(vel_setpoint, 0)
            measured_vel = axis_ctx.handle.encoder.vel_estimate
            max_measured_vel = max(measured_vel, max_measured_vel)
            test_assert_eq(measured_vel, expected_velocity, range=vel_range)
            test_assert_no_error(axis_ctx)

            # log progress
            if time.monotonic() - last_print > 1:
                last_print = time.monotonic()
                logger.debug("ramping up: commanded {}, expected {}, measured {} ".format(vel_setpoint, expected_velocity, measured_vel))

            time.sleep(0.001)

        logger.debug("reached top speed of {} counts/sec".format(max_measured_vel))

        if self._brake:
            axis_ctx.handle.controller.set_vel_setpoint(0, 0)
            time.sleep(0.5)
            # If the velocity integrator at work, it may now work against slowing down.
            test_assert_eq(axis_ctx.handle.encoder.vel_estimate, 0, range=rated_limit*0.3)
            # TODO: this is not a good bound, but the encoder float resolution results in a bad velocity estimate after this many turns
            time.sleep(0.5)
            test_assert_eq(axis_ctx.handle.encoder.vel_estimate, 0, range=2000)
            request_state(axis_ctx, AXIS_STATE_IDLE)
        test_assert_no_error(axis_ctx)


class TestHighVelocityInViscousFluid(DualAxisTest):
    """
    Runs TestHighVelocity on one motor while using the other motor as a load.
    The load is created by running velocity control with setpoint 0.
    """
    def __init__(self, load_current=10, driver_current=20):
        self._load_current = load_current
        self._driver_current = driver_current

    def run_test(self, axis0_ctx: AxisTestContext, axis1_ctx: AxisTestContext, logger):
        load_ctx = axis0_ctx
        driver_ctx = axis1_ctx
        if driver_ctx.name == 'top-odrive.black':
            # odrive.utils.start_liveplotter(lambda: [driver_ctx.odrv_ctx.handle.vbus_voltage])
            odrive.utils.start_liveplotter(lambda: [driver_ctx.handle.motor.current_control.Iq_measured, 
                                                    driver_ctx.handle.motor.current_control.Iq_setpoint])

        # Set up viscous fluid load
        logger.debug("activating load on {}...".format(load_ctx.name))
        load_ctx.handle.controller.config.vel_integrator_gain = 0
        load_ctx.handle.motor.config.current_lim = self._load_current
        load_ctx.odrv_ctx.handle.config.brake_resistance = 0 # disable brake resistance, the power will go into the bus
        load_ctx.handle.controller.set_vel_setpoint(0, 0)

        request_state(load_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

        driver_test = TestHighVelocity(
                override_current_limit=self._driver_current,
                load_current=self._load_current, brake=False)
        driver_test.check_preconditions(driver_ctx, logger)
        driver_test.run_test(driver_ctx, logger)

        # put load to idle as quickly as possible, otherwise, because the brake resistor is disabled,
        # it will try to put the braking power into the power rail where it has nowhere to go.
        request_state(load_ctx, AXIS_STATE_IDLE)
        request_state(driver_ctx, AXIS_STATE_IDLE)

class TestSelfLoadedPosVelDistribution(DualAxisTest):
    """
    Uses an ODrive mechanically connected to itself to test a distribution of
    speeds and currents. Since it's connected to itself, we can be a lot less
    strict about the brake resistor power use.
    """
    def __init__(self, rpm_range=1000, load_current_range=10, driver_current_lim=20):
        self._rpm_range = rpm_range
        self._load_current_range = load_current_range
        self._driver_current_lim = driver_current_lim

    def run_test(self, axis0_ctx: AxisTestContext, axis1_ctx: AxisTestContext, logger):
        load_ctx = axis0_ctx
        driver_ctx = axis1_ctx

        logger.debug("Iload range: {} A, Idriver: {} A".format(self._load_current_range, self._driver_current_lim))

        # max speed for rig in counts/s for each encoder (may be different CPR)
        max_rpm = min(self._rpm_range, get_max_rpm(driver_ctx), get_max_rpm(load_ctx))
        driver_max_speed = max_rpm / 60 * driver_ctx.yaml['encoder-cpr']
        load_max_speed = max_rpm / 60 * load_ctx.yaml['encoder-cpr']
        logger.debug("RPM range: {} = driver {} = load {}".format(max_rpm, driver_max_speed, load_max_speed))

        # Set up velocity controlled load
        logger.debug("activating load on {}".format(load_ctx.name))
        load_ctx.handle.controller.config.vel_integrator_gain = 0
        load_ctx.handle.controller.config.vel_limit = load_max_speed
        load_ctx.handle.motor.config.current_lim = 0 #load current to be set during runtime
        load_ctx.handle.controller.set_vel_setpoint(0, 0) # vel sign also set during runtime
        request_state(load_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

        # Set up velocity controlled driver
        logger.debug("activating driver on {}".format(driver_ctx.name))
        driver_ctx.handle.motor.config.current_lim = self._driver_current_lim
        driver_ctx.handle.controller.config.vel_limit = driver_max_speed
        driver_ctx.handle.controller.set_vel_setpoint(0, 0)
        request_state(driver_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

        # Spiral parameters
        command_rate = 500.0 #Hz (nominal, achived rate is less due to time.sleep approx)
        test_duration = 20.0 #s
        num_cycles = 3.0 # number of spiral "rotations"

        t_0 = time.monotonic()
        t_ratio = 0
        last_print = t_0
        while t_ratio < 1:
            t_ratio = (time.monotonic() - t_0) / test_duration
            phase = 2 * math.pi * num_cycles * t_ratio
            driver_speed = t_ratio * driver_max_speed * math.sin(phase)
            # print(driver_speed)
            driver_ctx.handle.controller.set_vel_setpoint(driver_speed, 0)
            load_current = t_ratio * self._load_current_range * math.cos(phase)
            Iload_mag = abs(load_current)
            Iload_sign = np.sign(load_current)
            # print("I: {}, vel {}".format(Iload_mag, Iload_sign * load_max_speed))
            load_ctx.handle.motor.config.current_lim = Iload_mag
            load_ctx.handle.controller.set_vel_setpoint(Iload_sign * load_max_speed, 0)

            test_assert_no_error(driver_ctx)
            test_assert_no_error(load_ctx)

            # log progress
            if time.monotonic() - last_print > 1:
                last_print = time.monotonic()
                logger.debug("Envelope  --  vel: {:.2f}, I: {:.2f}".format(t_ratio * driver_max_speed, t_ratio * self._load_current_range))

            time.sleep(1/command_rate)

        request_state(load_ctx, AXIS_STATE_IDLE)
        request_state(driver_ctx, AXIS_STATE_IDLE)
        test_assert_no_error(driver_ctx)
        test_assert_no_error(load_ctx)

class TestVelCtrlVsPosCtrl(DualAxisTest):
    """
    Uses one ODrive as a load operating in velocity control mode.
    The other ODrive tries to "fight" against the load in position mode.
    """
    def run_test(self, axis0_ctx: AxisTestContext, axis1_ctx: AxisTestContext, logger):
        load_ctx = axis0_ctx
        driver_ctx = axis1_ctx

        # Set up viscous fluid load
        logger.debug("activating load on {}...".format(load_ctx.name))
        load_ctx.handle.controller.config.vel_integrator_gain = 0
        load_ctx.handle.controller.vel_integrator_torque = 0
        set_limits(load_ctx, logger, vel_limit=100000, current_limit=50)
        load_ctx.handle.controller.set_vel_setpoint(0, 0)
        request_state(load_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

        # Turn to some position
        logger.debug("using {} as driver against load, vel=100000...".format(driver_ctx.name))
        set_limits(driver_ctx, logger, vel_limit=100000, current_limit=50)
        init_pos = driver_ctx.handle.encoder.pos_estimate
        driver_ctx.handle.controller.set_pos_setpoint(init_pos + 100000, 0, 0)
        request_state(driver_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
        for _ in range(int(4000/5)):
            logger.debug(str(driver_ctx.handle.motor.current_control.Iq_setpoint))
            time.sleep(0.005)

        test_assert_no_error(load_ctx)
        test_assert_no_error(driver_ctx)

        logger.debug("using {} as driver against load, vel=20000...".format(driver_ctx.name))
        set_limits(driver_ctx, logger, vel_limit=20000, current_limit=50)
        init_pos = driver_ctx.handle.encoder.pos_estimate
        driver_ctx.handle.controller.set_pos_setpoint(init_pos + 100000, 0, 0)
        request_state(driver_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
        #for _ in range(int(5*4000/5)):
        #    logger.debug(str(driver_ctx.handle.motor.current_control.Iq_setpoint))
        #    time.sleep(0.005)
        time.sleep(7)

        odrive.utils.print_drv_regs("load motor ({})".format(load_ctx.name), load_ctx.handle.motor)
        odrive.utils.print_drv_regs("driver motor ({})".format(driver_ctx.name), driver_ctx.handle.motor)

        test_assert_no_error(load_ctx)
        test_assert_no_error(driver_ctx)

        ## Turn to another position
        #logger.debug("controlling against load, vel=40000...")
        #set_limits(axis1_ctx, logger, vel_limit=40000, current_limit=20)
        #init_pos = axis1_ctx.handle.encoder.pos_estimate
        #axis1_ctx.handle.controller.set_pos_setpoint(init_pos + 100000, 0, 0)
        #request_state(axis1_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)


# ASCII protocol helper functions
def gcode_calc_checksum(data):
    from functools import reduce
    return reduce(lambda a, b: a ^ b, data)
def gcode_append_checksum(data):
    return data + b'*' + str(gcode_calc_checksum(data)).encode('ascii')
def get_lines(port):
    buf = port.get_bytes(512, time.monotonic() + 0.2)
    return [line.rstrip(b'\r') for line in buf.split(b'\n') if line.rstrip(b'\r')]

class TestAsciiProtocol(ODriveTest):
    def run_test(self, odrv_ctx: ODriveTestContext, logger):
        import odrive.serial_transport
        port = odrive.serial_transport.SerialStreamTransport(odrv_ctx.yaml['uart'], 115200)

        # send garbage to throw the device off track
        port.process_bytes(b"garbage\r\n\r\0trash\n")
        port.process_bytes(b"\n") # start a new clean line
        get_lines(port) # flush RX buffer

        # info command without checksum
        port.process_bytes(b"i\n")
        # check if it reports the serial number (among other things)
        lines = get_lines(port)
        expected_line = ('Serial number: ' + odrv_ctx.yaml['serial-number']).encode('ascii')
        if not expected_line in lines:
            raise Exception("expected {} in ASCII protocol response but got {}".format(expected_line, str(lines)))

        # info command with checksum
        port.process_bytes(gcode_append_checksum(b"i") + b" ; a useless comment\n")
        # check if it reports the serial number with checksum (among other things)
        lines = get_lines(port)
        expected_line = gcode_append_checksum(('Serial number: ' + odrv_ctx.yaml['serial-number']).encode('ascii'))
        if not expected_line in lines:
            raise Exception("expected {} in ASCII protocol response but got {}".format(expected_line, str(lines)))

        port.process_bytes(b"p 0 2000 -10 0.002\n")
        time.sleep(0.01) # 1ms is too short, 2ms usually works, 10ms for good measure
        test_assert_eq(odrv_ctx.handle.axis0.controller.pos_setpoint, 2000, accuracy=0.001)
        test_assert_eq(odrv_ctx.handle.axis0.controller.vel_setpoint, -10, accuracy=0.001)
        test_assert_eq(odrv_ctx.handle.axis0.controller.current_setpoint, 0.002, accuracy=0.001)

        port.process_bytes(b"v 1 -21.1 0.32\n")
        time.sleep(0.01)
        test_assert_eq(odrv_ctx.handle.axis1.controller.vel_setpoint, -21.1, accuracy=0.001)
        test_assert_eq(odrv_ctx.handle.axis1.controller.current_setpoint, 0.32, accuracy=0.001)

        port.process_bytes(b"c 0 0.1\n")
        time.sleep(0.01)
        test_assert_eq(odrv_ctx.handle.axis0.controller.current_setpoint, 0.1, accuracy=0.001)

        # write arbitrary parameter
        port.process_bytes(b"w axis0.controller.pos_setpoint -123.456 ; comment\n")
        time.sleep(0.01)
        test_assert_eq(odrv_ctx.handle.axis0.controller.pos_setpoint, -123.456, accuracy=0.001)

        port.process_bytes(b"r axis0.controller.pos_setpoint\n")
        lines = get_lines(port)
        expected_line = b'-123.4560'
        if lines != [expected_line]:
            raise Exception("expected {} in ASCII protocol response but got {}".format(expected_line, str(lines)))

        # read/write enums
        port.process_bytes(b"r axis0.error\n")
        lines = get_lines(port)
        expected_line = b'0'
        if lines != [expected_line]:
            raise Exception("expected {} in ASCII protocol response but got {}".format(expected_line, str(lines)))

        test_assert_eq(odrv_ctx.axes[0].handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
        port.process_bytes(b"w axis0.requested_state {}\n".format(AXIS_STATE_IDLE))
        time.sleep(0.01)
        test_assert_eq(odrv_ctx.axes[0].handle.current_state, AXIS_STATE_IDLE)

        # disable axes
        odrv_ctx.handle.axis0.controller.set_pos_setpoint(0, 0, 0)
        odrv_ctx.handle.axis1.controller.set_pos_setpoint(0, 0, 0)
        request_state(odrv_ctx.axes[0], AXIS_STATE_IDLE)
        request_state(odrv_ctx.axes[1], AXIS_STATE_IDLE)


class TestSensorlessControl(AxisTest):
    def run_test(self, axis_ctx: AxisTestContext, logger):
        odrv0.axis0.controller.config.vel_gain = 5 / get_sensorless_vel(axis_ctx, 10000)
        odrv0.axis0.controller.config.vel_integrator_gain = 10 / get_sensorless_vel(axis_ctx, 10000)
        target_vel = get_sensorless_vel(axis_ctx, 20000)
        axis_ctx.handle.controller.set_vel_setpoint(target_vel, 0)
        request_state(axis_ctx, AXIS_STATE_SENSORLESS_CONTROL)
        # wait for spinup
        time.sleep(2)
        test_assert_eq(odrv0.axis0.encoder.vel_estimate, target_vel, range=2000)

        request_state(axis_ctx, AXIS_STATE_IDLE)
