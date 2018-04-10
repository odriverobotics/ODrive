
import subprocess
import shlex
import math
import time
import sys
import threading
import odrive.discovery
from odrive.enums import *
import odrive.utils

import abc
ABC = abc.ABC

class TestFailed(Exception):
    def __init__(self, message):
        Exception.__init__(self, message)

class PreconditionsNotMet(Exception):
    pass

class ODriveTestContext():
    def __init__(self, name: str, yaml: dict):
        self.handle = None
        self.yaml = yaml
        self.name = name
        self.axes = []
        for axis_idx, axis_yaml in enumerate(yaml['axes']):
            axis_name = axis_yaml['name'] if 'name' in axis_yaml else '{}.axis{}'.format(name, axis_idx)
            self.axes.append(AxisTestContext(axis_name, axis_yaml, self))

    def rediscover(self):
        """
        Reconnects to the ODrive
        """
        self.handle = odrive.discovery.find_any(
            path="usb", serial_number=self.yaml['serial-number'], timeout=15)
        for axis_idx, axis_ctx in enumerate(self.axes):
            axis_ctx.handle = self.handle.__dict__['axis{}'.format(axis_idx)]

class AxisTestContext():
    def __init__(self, name: str, yaml: dict, odrv_ctx: ODriveTestContext):
        self.handle = None
        self.yaml = yaml
        self.name = name
        self.lock = threading.Lock()
        self.odrv_ctx = odrv_ctx

def test_assert_eq(observed, expected, range=None, accuracy=None):
    if range is None and accuracy is None and observed != expected:
        raise TestFailed("value mismatch: expected {} but observed {}".format(expected, observed))
    if not range is None and ((observed < expected - range) or (observed > expected + range)):
        raise TestFailed("value out of range: expected {}+-{} but observed {}".format(expected, range, observed))
    elif not accuracy is None and ((observed < expected * (1 - accuracy)) or (observed > expected * (1 + accuracy))):
        raise TestFailed("value out of range: expected {}+-{}% but observed {}".format(expected, accuracy*100.0, observed))

def test_assert_no_error(axis_ctx: AxisTestContext):
    errors = []
    if axis_ctx.handle.motor.error != 0:
        errors.append("motor failed with error {:04X}".format(axis_ctx.handle.motor.error))
    if axis_ctx.handle.encoder.error != 0:
        errors.append("encoder failed with error {:04X}".format(axis_ctx.handle.encoder.error))
    if axis_ctx.handle.sensorless_estimator.error != 0:
        errors.append("sensorless_estimator failed with error {:04X}".format(axis_ctx.handle.sensorless_estimator.error))
    if axis_ctx.handle.error != 0:
        errors.append("axis failed with error {:04X}".format(axis_ctx.handle.error))
    elif len(errors) > 0:
        errors.append("and by the way: axis reports no error even though there is one")
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
        axis_ctx.handle.error = AXIS_ERROR_NO_ERROR # reset error

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
        if (abs(axis_ctx.handle.encoder.pll_vel) > 500):
            logger.warn("axis still in motion, delaying 2 sec...")
            time.sleep(2)
        test_assert_eq(axis_ctx.handle.encoder.pll_vel, 0, range=500)

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
        test_assert_eq(axis0_ctx.handle.encoder.pll_vel, 0, range=1000)
        test_assert_eq(axis1_ctx.handle.encoder.pll_vel, 0, range=1000)

    @abc.abstractmethod
    def run_test(self, axis0_ctx: AxisTestContext, axis1_ctx: AxisTestContext, logger):
        pass

class TestDiscoverAndGotoIdle(ODriveTest):
    def run_test(self, odrv_ctx: ODriveTestContext, logger):
        odrv_ctx.rediscover()
        odrv_ctx.axes[0].handle.error = 0
        odrv_ctx.axes[1].handle.error = 0
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
        except odrive.protocol.ChannelBrokenException:
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
        init_pos = axis_ctx.handle.encoder.pll_pos
        axis_ctx.handle.controller.set_pos_setpoint(init_pos+1000, 0, 0)
        time.sleep(0.5)
        test_assert_eq(axis_ctx.handle.encoder.pll_pos, init_pos+1000, range=200)
        axis_ctx.handle.controller.set_pos_setpoint(init_pos-1000, 0, 0)
        time.sleep(0.5)
        test_assert_eq(axis_ctx.handle.encoder.pll_pos, init_pos-1000, range=400)

        logger.debug("closed loop control: test vel_limit")
        axis_ctx.handle.controller.set_pos_setpoint(50000, 0, 0)
        axis_ctx.handle.controller.config.vel_limit = 40000
        time.sleep(0.3)
        test_assert_eq(axis_ctx.handle.encoder.pll_vel, 40000, range=4000)
        expected_sensorless_estimation = 40000 * 2 * math.pi / axis_ctx.yaml['encoder-cpr'] * axis_ctx.yaml['motor-pole-pairs']
        test_assert_eq(axis_ctx.handle.sensorless_estimator.pll_vel, expected_sensorless_estimation, range=50)
        time.sleep(3)
        test_assert_eq(axis_ctx.handle.encoder.pll_vel, 0, range=1000)
        time.sleep(0.5)
        request_state(axis_ctx, AXIS_STATE_IDLE)

class TestStoreAndReboot(ODriveTest):
    """
    Stores the current configuration to NVM and reboots.
    """
    def run_test(self, odrv_ctx: ODriveTestContext, logger):
        logger.debug("storing configuration and rebooting...")
        odrv_ctx.handle.save_configuration()
        try:
            odrv_ctx.handle.reboot()
        except odrive.protocol.ChannelBrokenException:
            pass # this is expected
        time.sleep(2)

        odrv_ctx.rediscover()

        logger.debug("verifying configuration after reboot...")
        test_assert_eq(odrv_ctx.handle.config.brake_resistance, odrv_ctx.yaml['brake-resistance'], accuracy=0.01)
        for axis_ctx in odrv_ctx.axes:
            test_assert_eq(axis_ctx.handle.encoder.config.cpr, axis_ctx.yaml['encoder-cpr'])
            test_assert_eq(axis_ctx.handle.motor.config.phase_resistance, axis_ctx.yaml['motor-phase-resistance'], accuracy=0.15)
            test_assert_eq(axis_ctx.handle.motor.config.phase_inductance, axis_ctx.yaml['motor-phase-inductance'], accuracy=0.5)


class TestHighVelocityCtrl(AxisTest):
    """
    Spins the motor up to it's max speed during a period of 10s.
    The commanded max speed is based on the motor's KV rating and nominal V_bus,
    however due to several factors the theoretical limit is about 72% of that.
    The test passes if the motor follows the commanded ramp closely up to 90% of
    the theoretical limit (and if no errors occur along the way).
    """
    def check_preconditions(self, axis_ctx: AxisTestContext, logger):
        super(TestHighVelocityCtrl, self).check_preconditions(axis_ctx, logger)
        test_assert_eq(axis_ctx.handle.motor.is_calibrated, True)
        test_assert_eq(axis_ctx.handle.encoder.is_ready, True)

    def run_test(self, axis_ctx: AxisTestContext, logger):
        # Calculate theoretical max velocity in encoder counts per second based on the nominal
        # V_bus and motor KV rating
        max_rpm = axis_ctx.odrv_ctx.yaml['vbus-voltage'] * axis_ctx.yaml['motor-kv']
        rated_limit = max_rpm / 60 * axis_ctx.yaml['encoder-cpr']
        expected_limit = rated_limit

        # The KV-rating assumes square-waves on the motor phases (hexagonal space vector trajectory)
        # whereas the ODrive modulates the space vector around a circular trajectory.
        # See Fig 4.28 here: http://krex.k-state.edu/dspace/bitstream/handle/2097/1507/JamesMevey2009.pdf
        expected_limit *= (2/math.sqrt(3)) / (4/math.pi) # roughtly 90%

        # The ODrive only goes to 80% modulation depth in order to save some time for the ADC measurements.
        # See FOC_current in motor.cpp.
        expected_limit *= 0.8

        # Add a 10% margin to account for 
        expected_limit *= 0.9

        logger.debug("rated max speed: {}, expected max speed: >= {}".format(rated_limit, expected_limit))
        #theoretical_limit = 100000
        
        # Set the current limit accordingly so we don't burn the brake resistor while slowing down
        set_limits(axis_ctx, logger, vel_limit=rated_limit, current_limit=50)
        request_state(axis_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)


        ramp_up_time = 20.0
        t_0 = time.monotonic()
        last_print = t_0
        max_true_vel = 0.0
        while True:
            ratio = (time.monotonic() - t_0) / ramp_up_time
            if ratio >= 1:
                break

            # While ramping up we want to remain within +-5% of the setpoint.
            # However we accept if we can only approach 80% of the theoretical limit.
            vel_setpoint = ratio * rated_limit
            vel_range = max(0.05*vel_setpoint, 5000)
            if vel_setpoint - vel_range > expected_limit:
                vel_range = vel_setpoint - expected_limit

            # log progress
            if time.monotonic() - last_print > 1:
                last_print = time.monotonic()
                logger.debug("ramping up: now at " + str(vel_setpoint))

            # set and measure velocity
            axis_ctx.handle.controller.set_vel_setpoint(vel_setpoint, 0)
            true_vel = axis_ctx.handle.encoder.pll_vel
            max_true_vel = max(true_vel, max_true_vel)
            test_assert_eq(true_vel, vel_setpoint, range=vel_range)
            test_assert_no_error(axis_ctx)

            time.sleep(0.001)

        axis_ctx.handle.controller.set_vel_setpoint(0, 0)
        time.sleep(0.5)
        # TODO: this is not a good bound, but the encoder float resolution results in a bad velocity estimate after this many turns
        test_assert_eq(axis_ctx.handle.encoder.pll_vel, 0, range=2000)
        request_state(axis_ctx, AXIS_STATE_IDLE)


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
        load_ctx.handle.controller.vel_integrator_current = 0
        set_limits(load_ctx, logger, vel_limit=100000, current_limit=50)
        load_ctx.handle.controller.set_vel_setpoint(0, 0)
        request_state(load_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)

        # Turn to some position
        logger.debug("using {} as driver against load, vel=100000...".format(driver_ctx.name))
        set_limits(driver_ctx, logger, vel_limit=100000, current_limit=50)
        init_pos = driver_ctx.handle.encoder.pll_pos
        driver_ctx.handle.controller.set_pos_setpoint(init_pos + 100000, 0, 0)
        request_state(driver_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
        for _ in range(int(4000/5)):
            logger.debug(str(driver_ctx.handle.motor.current_control.Iq_setpoint))
            time.sleep(0.005)

        test_assert_no_error(load_ctx)
        test_assert_no_error(driver_ctx)

        logger.debug("using {} as driver against load, vel=20000...".format(driver_ctx.name))
        set_limits(driver_ctx, logger, vel_limit=20000, current_limit=50)
        init_pos = driver_ctx.handle.encoder.pll_pos
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
        #init_pos = axis1_ctx.handle.encoder.pll_pos
        #axis1_ctx.handle.controller.set_pos_setpoint(init_pos + 100000, 0, 0)
        #request_state(axis1_ctx, AXIS_STATE_CLOSED_LOOP_CONTROL)
