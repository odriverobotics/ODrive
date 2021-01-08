#!/usr/bin/env python3

# Provides utilities for standalone test scripts.
# This script is not intended to be run directly.

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

import stat
import odrive
from odrive.enums import *
import odrive.utils
import fibre
from fibre import Logger, Event
import argparse
import yaml
from inspect import signature
import itertools
import time
import tempfile
import io
import re
import datetime
from typing import Union, Tuple
import traceback

# needed for curve fitting
import numpy as np
import scipy.optimize
import scipy.ndimage.filters


# Assert utils ----------------------------------------------------------------#

class TestFailed(Exception):
    def __init__(self, message):
        Exception.__init__(self, message)

def test_assert_eq(observed, expected, range=None, accuracy=None):
    sign = lambda x: 1 if x >= 0 else -1

    # Comparision with absolute range
    if not range is None:
        if (observed < expected - range) or (observed > expected + range):
            raise TestFailed("value out of range: expected {}+-{} but observed {}".format(expected, range, observed))

    # Comparision with relative range
    elif not accuracy is None:
        if sign(observed) != sign(expected) or (abs(observed) < abs(expected) * (1 - accuracy)) or (abs(observed) > abs(expected) * (1 + accuracy)):
            raise TestFailed("value out of range: expected {}+-{}% but observed {}".format(expected, accuracy*100.0, observed))

    # Exact comparision
    else:
        if observed != expected:
            raise TestFailed("value mismatch: expected {} but observed {}".format(expected, observed))

def test_assert_within(observed, lower_bound, upper_bound, accuracy=0.0):
    """
    Checks if the value is within the closed interval [lower_bound, upper_bound]
    The permissible range can be expanded in both direction by the coefficiont "accuracy".
    I.e. accuracy of 1.0 would expand the range by a total factor of 3.0
    """

    lower_bound, upper_bound = (
        (lower_bound - (upper_bound - lower_bound) * accuracy),
        (upper_bound + (upper_bound - lower_bound) * accuracy)
    )

    if (observed < lower_bound) or (observed > upper_bound):
        raise TestFailed(f"the oberved value {observed} is outside the interval [{lower_bound}, {upper_bound}]")


# Other utils -----------------------------------------------------------------#

def disjoint_sets(list_of_sets: list):
    while len(list_of_sets):
        current_set, list_of_sets = list_of_sets[0], list_of_sets[1:]
        updated = True
        while updated:
            updated = False
            for i, s in enumerate(list_of_sets):
                if len(current_set.intersection(s)):
                    current_set = current_set.union(s)
                    list_of_sets = list_of_sets[:i] + list_of_sets[(i+1):]
                    updated = True
        yield current_set

def is_list_like(arg):
    return hasattr(arg, '__iter__') and not isinstance(arg, str)

def all_unique(lst):
    seen = list()
    return not any(i in seen or seen.append(i) for i in lst)

def modpm(val, range):
    return ((val + (range / 2)) % range) - (range / 2)

def clamp(val, lower_bound, upper_bound):
    return min(max(val, lower_bound), upper_bound)

def record_log(data_getter, duration=5.0):
    logger.debug(f"Recording log for {duration}s...")
    data = []
    start = time.monotonic()
    while time.monotonic() - start < duration:
        data.append((time.monotonic() - start,) + tuple(data_getter()))
    return np.array(data)

def save_log(data, id=None):
    import json
    filename = '/tmp/log{}.json'.format('' if id is None else str(id))
    with open(filename, 'w+') as fp:
        json.dump(data.tolist(), fp, indent=2)
    print(f'data saved to {filename}')

def fit_line(data):
    func = lambda x, a, b: x*a + b
    slope, offset = scipy.optimize.curve_fit(func, data[:,0], data[:,1], [1.0, 0])[0]
    return slope, offset, func(data[:,0], slope, offset)

def fit_sawtooth(data, min_val, max_val, sigma=10):
    """
    Fits the data to a sawtooth function.
    Returns the average absolute error and the number of outliers.
    The sample data must span at least one full period.
    data is expected to contain one row (t, y) for each sample.
    """
    
    # Sawtooth function with free parameters for period and x-shift
    func = lambda x, a, b: np.mod(a * x + b, max_val - min_val) + min_val
    
    # Fit period and x-shift
    mid_point = (min_val + max_val) / 2
    filtered_data = scipy.ndimage.filters.gaussian_filter(data[:,1], sigma=sigma)
    if max_val > min_val:
        zero_crossings = data[np.where((filtered_data[:-1] > mid_point) & (filtered_data[1:] < mid_point))[0], 0]
    else:
        zero_crossings = data[np.where((filtered_data[:-1] < mid_point) & (filtered_data[1:] > mid_point))[0], 0]

    
    if len(zero_crossings) == 0:
        # No zero-crossing - fit simple line
        slope, offset, _ = fit_line(data)

    elif len(zero_crossings) == 1:
        # One zero-crossing - fit line based on the longer half
        z_index = np.where(data[:,0] > zero_crossings[0])[0][0]
        if z_index > len(data[:,0]):
            slope, offset, _ = fit_line(data[:z_index])
        else:
            slope, offset, _ = fit_line(data[z_index:])

    else:
        # Two or more zero-crossings - determine period based on average distance between zero-crossings

        period = (zero_crossings[1:] - zero_crossings[:-1]).mean()
        slope = (max_val - min_val) / period

        #shift = scipy.optimize.curve_fit(lambda x, b: func(x, period, b), data[:,0], data[:,1], [0.0])[0][0]
        if np.std(np.mod(zero_crossings, period)) < np.std(np.mod(zero_crossings + period/2, period)):
            shift = np.mean(np.mod(zero_crossings, period))
        else:
            shift = np.mean(np.mod(zero_crossings + period/2, period)) - period/2
        offset = -slope * shift

    return slope, offset, func(data[:,0], slope, offset)

def test_curve_fit(data, fitted_curve, max_mean_err, inlier_range, max_outliers):
    diffs = data[:,1] - fitted_curve

    mean_err = np.abs(diffs).mean()
    if mean_err > max_mean_err:
        save_log(np.concatenate([data, np.array([fitted_curve]).transpose()], 1))
        raise TestFailed("curve fit has too large mean error: {} > {}".format(mean_err, max_mean_err))
    
    outliers = np.count_nonzero((diffs > inlier_range) | (diffs < -inlier_range))
    if outliers > max_outliers:
        save_log(np.concatenate([data, np.array([fitted_curve]).transpose()], 1))
        raise TestFailed("curve fit has too many outliers (err > {}): {} > {}".format(inlier_range, outliers, max_outliers))

def test_watchdog(axis, feed_func, logger: Logger):
    """
    Tests the watchdog of one axis, using the provided function to feed the watchdog.
    This test assumes that the testing host has no more than 300ms random delays.
    """
    start = time.monotonic()
    axis.config.enable_watchdog = False
    axis.error = 0
    axis.config.watchdog_timeout = 1.0
    axis.watchdog_feed()
    axis.config.enable_watchdog = True
    test_assert_eq(axis.error, 0)
    for _ in range(5): # keep the watchdog alive for 3.5 seconds
        time.sleep(0.7)
        logger.debug('feeding watchdog at {}s'.format(time.monotonic() - start))
        feed_func()
        err = axis.error
        logger.debug('checking error at {}s'.format(time.monotonic() - start))
        test_assert_eq(err, 0)
        
    logger.debug('letting watchdog expire...')
    time.sleep(1.3) # let the watchdog expire
    test_assert_eq(axis.error, AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)

class SafeTerminator():
    """
    Context that can be used in a "with" statement to facilitate safe shutdown
    of the test rig when the test ends (succeeds or fails).
    """
    def __init__(self, logger, *axes):
        self.logger = logger
        self.axes = axes
    def __enter__(self):
        pass
    def __exit__(self, exc_type, exc_val, exc_tb):
        # TODO: cut power supply
        self.logger.debug('clearing config...')
        idle_axes = []
        for axis_ctx in self.axes:
            try:
                axis_ctx.handle.requested_state = AXIS_STATE_IDLE
                idle_axes.append(axis_ctx)
            except:
                self.logger.error(f"can't put axis {axis_ctx} into idle")
        # TODO: review if erase_configuration is safe during active PWM
        time.sleep(0.005)
        for axis_ctx in set(axis for axis in idle_axes):
            axis_ctx.parent.erase_config_and_reboot()

# Test Components -------------------------------------------------------------#

class Component(object):
    def __init__(self, parent):
        self.parent = parent

class ODriveComponent(Component):
    def __init__(self, yaml: dict):
        self.handle = None
        self.yaml = yaml

        if yaml['board-version'].startswith("v3."):
            self.encoders = [ODriveEncoderComponent(self, 0, yaml['encoder0']), ODriveEncoderComponent(self, 1, yaml['encoder1'])]
            self.axes = [ODriveAxisComponent(self, 0, yaml['motor0']), ODriveAxisComponent(self, 1, yaml['motor1'])]
            gpio_nums = range(1,9)
        elif yaml['board-version'].startswith("v4."):
            self.encoders = [ODriveEncoderComponent(self, 0, yaml['encoder0'])]
            self.axes = [ODriveAxisComponent(self, 0, yaml['motor0'])]
            gpio_nums = range(23)
        else:
            raise Exception("unknown board version {}".format(yaml['board-version']))

        for i in gpio_nums:
            self.__setattr__('gpio' + str(i), Component(self))
        self.can = Component(self)
        self.sck = Component(self)
        self.miso = Component(self)
        self.mosi = Component(self)

    def get_subcomponents(self):
        for enc_ctx in self.encoders:
            yield 'encoder' + str(enc_ctx.num), enc_ctx
        for axis_ctx in self.axes:
            yield 'axis' + str(axis_ctx.num), axis_ctx
        for k in dir(self):
            if k.startswith('gpio'):
                yield k, getattr(self, k)
        yield 'can', self.can
        yield 'spi.sck', self.sck
        yield 'spi.miso', self.miso
        yield 'spi.mosi', self.mosi

    def prepare(self, logger: Logger):
        """
        Connects to the ODrive
        """
        if not self.handle is None:
            return

        logger.debug('waiting for {} ({})'.format(self.yaml['name'], self.yaml['serial-number']))
        self.handle = odrive.find_any(channel_termination_token=shutdown_token, serial_number=self.yaml['serial-number'], timeout=60)#, printer=print)
        assert(self.handle)
        #for axis_idx, axis_ctx in enumerate(self.axes):
        #    axis_ctx.handle = getattr(self.handle, f'axis{axis_idx}')
        for encoder_idx, encoder_ctx in enumerate(self.encoders):
            encoder_ctx.handle = getattr(self.handle, f'axis{encoder_idx}').encoder
        # TODO: distinguish between axis and motor context
        for axis_idx, axis_ctx in enumerate(self.axes):
            axis_ctx.handle = getattr(self.handle, f'axis{axis_idx}')

    def disable_mappings(self):
        for k in dir(self.handle.config):
            if re.match(r'gpio[0-9]+_pwm_mapping', k):
                getattr(self.handle.config, k).endpoint = None
        for k in dir(self.handle.config):
            if re.match(r'gpio[0-9]+_analog_mapping', k):
                getattr(self.handle.config, k).endpoint = None

    def save_config_and_reboot(self):
        try:
            self.handle.save_configuration()
        except fibre.ObjectLostError:
            pass # this is expected
        self.handle = None
        time.sleep(2)
        self.prepare(logger)

    def erase_config_and_reboot(self):
        try:
            self.handle.erase_configuration()
        except fibre.ObjectLostError:
            pass # this is expected
        self.handle = None
        time.sleep(2)
        self.prepare(logger)

class MotorComponent(Component):
    def __init__(self, yaml: dict):
        self.yaml = yaml
        self.shaft = Component(self)
        self.phases = Component(self)

    def get_subcomponents(self):
        return [('shaft', self.shaft), ('phases', self.phases)]

    def prepare(self, logger: Logger):
        pass

class ODriveAxisComponent(Component):
    def __init__(self, parent: ODriveComponent, num: int, yaml: dict):
        Component.__init__(self, parent)
        self.handle = None
        self.yaml = yaml # TODO: this is bad naming
        self.num = num

    def prepare(self, logger: Logger):
        self.parent.prepare(logger)

class ODriveEncoderComponent(Component):
    def __init__(self, parent: ODriveComponent, num: int, yaml: dict):
        Component.__init__(self, parent)
        self.handle = None
        self.yaml = yaml
        self.num = num
        self.z = Component(self)
        self.a = Component(self)
        self.b = Component(self)

    def get_subcomponents(self):
        return [('z', self.z), ('a', self.a), ('b', self.b)]

    def prepare(self, logger: Logger):
        self.parent.prepare(logger)

class EncoderComponent(Component):
    def __init__(self, parent: Component, yaml: dict):
        Component.__init__(self, parent)
        self.yaml = yaml
        self.z = Component(self)
        self.a = Component(self)
        self.b = Component(self)
        self.shaft = Component(self)

    def get_subcomponents(self):
        return [('z', self.z), ('a', self.a), ('b', self.b), ('shaft', self.shaft)]

class GeneralPurposeComponent(Component):
    def __init__(self, yaml: dict):
        self.components = {}
        for component_yaml in yaml.get('components', []):
            if component_yaml['type'] == 'can':
                self.components[component_yaml['name']] = CanInterfaceComponent(self, component_yaml)
            if component_yaml['type'] == 'uart':
                self.components[component_yaml['name']] = SerialPortComponent(self, component_yaml)
            if component_yaml['type'] == 'gpio':
                self.components['gpio' + str(component_yaml['num'])] = LinuxGpioComponent(self, component_yaml)

    def get_subcomponents(self):
        return self.components.items()

class LinuxGpioComponent(Component):
    def __init__(self, parent: Component, yaml: dict):
        Component.__init__(self, parent)
        self.num = int(yaml['num'])

    def config(self, output: bool):
        with open("/sys/class/gpio/gpio{}/direction".format(self.num), "w") as fp:
            fp.write('out' if output else '0')

    def write(self, state: bool):
        with open("/sys/class/gpio/gpio{}/value".format(self.num), "w") as fp:
            fp.write('1' if state else '0')


class SerialPortComponent(Component):
    def __init__(self, parent: Component, yaml: dict):
        Component.__init__(self, parent)
        self.yaml = yaml
        self.tx = Component(self)
        self.rx = Component(self)

    def get_subcomponents(self):
        yield 'tx', self.tx
        yield 'rx', self.rx

    def open(self, baudrate: int):
        import serial
        return serial.Serial(self.yaml['port'], baudrate, timeout=1)
    
class CanInterfaceComponent(Component):
    def __init__(self, parent: Component, yaml: dict):
        Component.__init__(self, parent)
        self.handle = None
        self.yaml = yaml
    
    def prepare(self, logger: Logger):
        if not self.handle is None:
            return

        import can
        self.handle = can.interface.Bus(bustype='socketcan', channel=self.yaml['interface'], bitrate=250000)

class TeensyGpio(Component):
    def __init__(self, parent: Component, num: int):
        Component.__init__(self, parent)
        self.num = num

class TeensyComponent(Component):
    def __init__(self, testrig, yaml: dict):
        self.testrig = testrig
        self.yaml = yaml
        if self.yaml['board-version'] == 'teensy:avr:teensy40':
            self.gpios = [TeensyGpio(self, i) for i in range(24)]
        elif self.yaml['board-version'] == 'teensy:avr:teensy41':
            self.gpios = [TeensyGpio(self, i) for i in range(42)]
        else:
            raise Exception(f"unknown Arduino board {self.yaml['board-version']}")
        self.routes = []
        self.previous_routes = object()

    def get_subcomponents(self):
        for i, gpio in enumerate(self.gpios):
            yield ('gpio' + str(i)), gpio
        yield 'program', Component(self)

    def add_route(self, input: TeensyGpio, output: TeensyGpio, noise_enable: TeensyGpio):
        self.routes.append((input, output, noise_enable))

    def commit_routing_config(self, logger: Logger):
        if self.previous_routes == self.routes:
            self.routes = []
            return

        code = ''
        code += 'bool noise = false;\n'
        code += 'void setup() {\n'
        for i, o, n in self.routes:
            code += '  pinMode({}, OUTPUT);\n'.format(o.num)
        code += '}\n'
        code += 'void loop() {\n'
        code += '  noise = !noise;\n'
        for i, o, n in self.routes:
            if n:
                # with noise enable
                code += '  digitalWrite({}, digitalRead({}) ? noise : digitalRead({}));\n'.format(o.num, n.num, i.num)
            else:
                # no noise enable
                code += '  digitalWrite({}, digitalRead({}));\n'.format(o.num, i.num)
        code += '}\n'

        self.compile_and_program(code)

        self.previous_routes = self.routes
        self.routes = []

    def compile(self, sketchfile, hexfile):
        env = os.environ.copy()
        env['ARDUINO_COMPILE_DESTINATION'] = hexfile
        run_shell(
            ['arduino', '--board', self.yaml['board-version'], '--verify', sketchfile],
            logger, env = env, timeout = 120)

    def program(self, hex_file_path: str, logger: Logger):
        """
        Programs the specified hex file onto the Teensy.
        To reset the Teensy, a GPIO of the local system must be connected to the
        Teensy's "Program" pin.
        """

        # todo: this should be treated like a regular setup resource
        program_gpio = self.testrig.get_directly_connected_components(self.testrig.get_component_name(self) + '.program')[0]

        # Put Teensy into program mode by pulling it's program pin down
        program_gpio.config(output = True)
        program_gpio.write(False)
        time.sleep(0.1)
        program_gpio.write(True)
        
        run_shell(["teensy-loader-cli", "-mmcu=" + self.yaml['board-version'].rpartition(':')[2].upper(), "-w", hex_file_path], logger, timeout = 5)
        time.sleep(0.5) # give it some time to boot

    def compile_and_program(self, code: str):
        with tempfile.TemporaryDirectory() as temp_dir:
            with open(os.path.join(temp_dir, 'code.ino'), 'w+') as code_fp:
                code_fp.write(code)
                code_fp.flush()
                code_fp.seek(0)
                print('Writing code to teensy: ')
                print(code_fp.read())
                with tempfile.NamedTemporaryFile(suffix='.hex') as hex_fp:
                    self.compile(code_fp.name, hex_fp.name)
                    self.program(hex_fp.name, logger)

class LowPassFilterComponent(Component):
    def __init__(self, parent: Component):
        Component.__init__(self, parent)
        self.en = Component(self)

    def get_subcomponents(self):
        yield 'en', self.en

class TestFixture(object):
    """
    Base class for test fixtures. A test fixture is what defines the
    prerequisites for a paricular test.
    """

    def all_of(*test_fixtures):
        test_fixtures = [(tf._subfixtures if isinstance(tf, CompositeTestFixture) else [[tf]])
                          for tf in test_fixtures if not tf is None]
        # Each item in test_fixtures is now in disjunctive normal form and we
        # flatten this into a single expression in disjunctive normal form
        combinations = []
        for combination in itertools.product([[]], *test_fixtures):
            # flatten list of lists
            combination = [item for c in combination for item in c]
            combinations.append(combination)

        if len(combinations) == 1 and len(combinations[0]) == 0:
            return None
        elif len(combinations) == 1 and len(combinations[0]) == 1:
            return combinations[0][0]
        else:
            return CompositeTestFixture(combinations)

    def any_of(*test_fixtures):
        test_fixtures = [(tf._subfixtures if isinstance(tf, CompositeTestFixture) else [[tf]])
                         for tf in test_fixtures]
        # Flatten list of lists into list
        combinations = [item for c in test_fixtures for item in c]
        if len(combinations) == 1 and len(combinations[0]) == 0:
            return None
        elif len(combinations) == 1 and len(combinations[0]) == 1:
            return combinations[0][0]
        else:
            return CompositeTestFixture(combinations)

class CompositeTestFixture(TestFixture):
    """
    Represents the combination of several test fixtures. The combinations are
    stored as a list of lists representing a disjunctive normal form.
    Do not construct this class directly, use TestFixture.any_of or
    TestFixture.all_of instead.
    """
    def __init__(self, subfixtures: list):
        self._subfixtures = subfixtures


class TeensyForwardingFixture(TestFixture):
    def __init__(self, teensy: TeensyComponent, high_z: TeensyGpio, low_z: TeensyGpio):
        self.teensy = teensy
        self.high_z = high_z
        self.low_z = low_z
        self.noise_enable = None
    
    def prepare(self, logger: Logger):
        self.teensy.add_route(self.high_z, self.low_z, self.noise_enable)

    def get_resources(self):
        return [(self.teensy, False), (self.high_z, True), (self.low_z, True), (self.noise_enable, True)]

class ClosedLoopControlFixture(TestFixture):
    def __init__(self, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent):
        self.axis_ctx, self.motor_ctx, self.enc_ctx = (axis_ctx, motor_ctx, enc_ctx)

    def get_resources(self):
        return []

    def prepare(self, logger: Logger):
        # Make sure no funny configuration is active
        logger.debug('Setting up clean configuration...')
        self.axis_ctx.parent.erase_config_and_reboot()

        # Set motor calibration values
        self.axis_ctx.handle.motor.config.phase_resistance = float(self.motor_ctx.yaml['phase-resistance'])
        self.axis_ctx.handle.motor.config.phase_inductance = float(self.motor_ctx.yaml['phase-inductance'])
        self.axis_ctx.handle.motor.config.pre_calibrated = True

        # Set brake resistor settings
        if 'brake-resistance' in self.axis_ctx.parent.yaml:
            logger.debug(f"brake resistor set to {self.axis_ctx.parent.yaml['brake-resistance']} Ohm")
            self.axis_ctx.parent.handle.config.brake_resistance = float(self.axis_ctx.parent.yaml['brake-resistance'])
            # The docs say this requires a reboot but here's a small secret:
            # Since the brake resistor is also started in clear_errors() this
            # circumvents the need for a reboot.
            self.axis_ctx.parent.handle.config.enable_brake_resistor = True
        else:
            logger.debug("brake resistor disabled")
            # TODO: set vbus voltage trip level based on yaml
            self.axis_ctx.parent.handle.config.dc_max_negative_current = -1
            self.axis_ctx.parent.handle.config.enable_brake_resistor = False

        # Set calibration settings
        self.axis_ctx.handle.encoder.config.direction = 0
        self.axis_ctx.handle.encoder.config.use_index = False
        self.axis_ctx.handle.encoder.config.calib_scan_omega = 12.566 # 2 electrical revolutions per second
        self.axis_ctx.handle.encoder.config.calib_scan_distance = 50.265 # 8 revolutions
        self.axis_ctx.handle.encoder.config.bandwidth = 1000

        self.axis_ctx.parent.handle.clear_errors()

        logger.debug('Calibrating encoder offset...')
        request_state(self.axis_ctx, AXIS_STATE_ENCODER_OFFSET_CALIBRATION)

        time.sleep(9) # actual calibration takes 8 seconds

        test_assert_eq(self.axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_no_error(self.axis_ctx)

class AnyTestCase():
    """
    Helper to specifiy that the test case may be run with any of the specified
    parameter combinations.
    """
    def __init__(self, *alternatives):
        self.alternatives = alternatives

class TestRig():
    def __init__(self, yaml: dict, logger: Logger):
        # Contains all components (including subcomponents).
        # Ports are components too.
        self.components_by_name = {} # {'name': object, ...}
        self.names_by_component = {} # {'name': object, ...}

        def add_component(name, component):
            self.components_by_name[name] = component
            self.names_by_component[component] = name
            if hasattr(component, 'get_subcomponents'):
                for subname, subcomponent in component.get_subcomponents():
                    add_component(name + '.' + subname, subcomponent)

        for component_yaml in yaml['components']:
            if component_yaml['type'] == 'odrive':
                add_component(component_yaml['name'], ODriveComponent(component_yaml))
            elif component_yaml['type'] == 'generalpurpose':
                add_component(component_yaml['name'], GeneralPurposeComponent(component_yaml))
            elif component_yaml['type'] == 'arduino':
                add_component(component_yaml['name'], TeensyComponent(self, component_yaml))
            elif component_yaml['type'] == 'motor':
                add_component(component_yaml['name'], MotorComponent(component_yaml))
            elif component_yaml['type'] == 'encoder':
                add_component(component_yaml['name'], EncoderComponent(self, component_yaml))
            elif component_yaml['type'] == 'lpf':
                add_component(component_yaml['name'], LowPassFilterComponent(self))
            else:
                logger.warn('test rig has unsupported component ' + component_yaml['type'])
                continue

        # List of disjunct sets, where each set holds references of the mutually connected components
        self.connections = []
        for connection_yaml in yaml['connections']:
            self.connections.append(set(self.components_by_name[name] for name in connection_yaml))
        self.connections = list(disjoint_sets(self.connections))

        # Dict for fast lookup of the connection sets for each port
        self.net_by_component = {}
        for s in self.connections:
            for port in s:
                self.net_by_component[port] = s

    def get_closed_loop_combos(self, init: bool = True):
        """
        Fetches all connected (odrive axis, motor, encoder) combos in the test rig.

        Returns a generator of tuples where each tuple follows the form:
        (axis, motor, encoder, test_fixture).
        """
        all_axes = self.get_components(ODriveAxisComponent)
        all_motors = self.get_components(MotorComponent)
        all_encoders = self.get_components(EncoderComponent)
        for axis, motor, encoder in itertools.product(all_axes, all_motors, all_encoders):
            is_connected, tf1 = self.check_connections([
                (axis, motor.phases),
                (motor.shaft, encoder.shaft),
                (encoder.a, axis.parent.encoders[axis.num].a),
                (encoder.b, axis.parent.encoders[axis.num].b)
            ])
            if not is_connected:
                continue
            tf2 = ClosedLoopControlFixture(axis, motor, encoder) if init else None
            test_fixture = TestFixture.all_of(tf1, tf2)
            yield axis, motor, encoder, test_fixture
    
    def check_connection(self, component1: Component, component2: Component, mode: str = 'indirect'):
        """
        Checks if the specified components are connected or connectable through
        a test fixture.

        If there are one-directional connections, they must point from
        component1 to component2.

        Returns: A tuple (connectable: bool, text_fixture: TestFixture). If
                 connectable is True then the test_fixture, if not None, must
                 be prepared before the components are actually connected.
        """
        assert(mode in ['direct', 'indirect'])

        net1 = self.net_by_component.get(component1, set([component1]))
        if component2 in net1:
            return True, None # The components are directly connected
        if mode == 'direct':
            return False, None
        
        net2 = self.net_by_component.get(component2, set([component2]))

        possible_test_fixtures = []

        for port1, port2 in itertools.product(net1, net2):
            if (isinstance(port1, TeensyGpio) and isinstance(port2, TeensyGpio) and port1.parent == port2.parent):
                possible_test_fixtures.append(TeensyForwardingFixture(port1.parent, port1, port2))
        
        if not len(possible_test_fixtures):
            return False, None # no forwarding is possible between the two components
        else:
            return True, TestFixture.any_of(*possible_test_fixtures)

    def check_connections(self, components: list, mode: str = 'indirect'):
        tfs = []
        for component1, component2 in components:
            is_connected, tf = self.check_connection(component1, component2, mode)
            if not is_connected:
                return False, None
            tfs.append(tf)
        return True, TestFixture.all_of(*tfs)


    def get_components(self, t: type):
        """Returns a tuple (name, component) for all components that are of the specified type"""
        return (comp for comp in self.names_by_component.keys() if isinstance(comp, t))
    
    def get_component_name(self, component: Component):
        #if isinstance(component, ProxiedComponent):
        #    return self.names_by_component[component.impl]
        #else:
            return self.names_by_component[component]

    def get_directly_connected_components(self, component: Union[str, Component]):
        """
        Returns all components that are directly connected to the specified
        component, excluding the specified component itself.
        """
        if isinstance(component, str):
            component = self.components_by_name[component]
        result = self.net_by_component.get(component, set([component]))
        return [c for c in result if (c != component)]

    def get_connected_components(self, src: Union[dict, Tuple[Union[Component, str], bool]], comp_type: type = None):
        """
        Returns all components that are either directly or indirectly (through a
        Teensy) connected to the specified component(s).

        component: Either:
            - A component object.
            - A component name given as string.
            - A tuple of the form (comp, dir) where comp is a component object
              or name and dir specifies the data direction.
              The direction is required if routing through a Teensy should be
              considered.
            - A dict {sumcomponent: val} where subcomponent is a string
              such as 'tx' or 'rx' and val is of one of the forms described above.
        
        A type can be specified to filter the connected components.
        """

        candidates = self.get_components(comp_type)

        if isinstance(src, dict):
            for candidate in candidates:
                subcomponents = dict(candidate.get_subcomponents())

                if len(set(src.keys()) - set(subcomponents.keys())):
                    continue # candidate doesn't have all of the requested ports

                test_fixtures = []
                for name, subsrc in src.items():
                    srcport, direction = subsrc if isinstance(subsrc, tuple) else (subsrc, False)
                    is_connected, test_fixture = (self.check_connection(srcport, subcomponents[name]) if direction else
                                                  self.check_connection(subcomponents[name], srcport))
                    if not is_connected:
                        break
                    test_fixtures.append(test_fixture)


                if len(test_fixtures) < len(src.items()):
                    continue # not all of the ports are connected to the candidate's ports
                
                yield candidate, TestFixture.all_of(*test_fixtures)

        else:
            src, direction = src if isinstance(src, tuple) else (src, False)
            assert(isinstance(src, Component))
            for candidate in candidates:
                is_connected, test_fixture = (self.check_connection(src, candidate) if direction else
                                              self.check_connection(candidate, src))
                if is_connected:
                    yield candidate, test_fixture

# Helper functions ------------------------------------------------------------#

def request_state(axis_ctx: ODriveAxisComponent, state, expect_success=True):
    axis_ctx.handle.requested_state = state
    time.sleep(0.001)
    if expect_success:
        test_assert_eq(axis_ctx.handle.current_state, state)
    else:
        test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)
        test_assert_eq(axis_ctx.handle.error, AXIS_ERROR_INVALID_STATE)
        axis_ctx.handle.error = AXIS_ERROR_NONE # reset error

def test_assert_no_error(axis_ctx: ODriveAxisComponent):
    any_error = (axis_ctx.handle.motor.error |
                 axis_ctx.handle.encoder.error |
                 #axis_ctx.handle.sensorless_estimator.error | # TODO: reenable
                 axis_ctx.handle.error) != 0 # TODO: this is not the complete list of components

    if any_error:
        lines = []
        odrive.utils.dump_errors(axis_ctx.parent.handle, printfunc = lines.append)
        raise TestFailed("\n".join(lines))

def run_shell(command_line, logger, env=None, timeout=None):
    """
    Runs a shell command in the current directory
    """
    import shlex
    import subprocess
    logger.debug("invoke: " + str(command_line))
    if isinstance(command_line, list):
        cmd = command_line
    else:
        cmd = shlex.split(command_line)
    result = subprocess.run(cmd, timeout=timeout,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=env)
    if result.returncode != 0:
        logger.error(result.stdout.decode(sys.stdout.encoding))
        raise TestFailed("command {} failed".format(command_line))


def render_html_summary(status, test_results, output_file):
    import jinja2
    with open(os.path.join(os.path.dirname(__file__), "results.html.j2")) as fp:
        env = jinja2.Environment()
        env.filters['passes'] = lambda x: [res for res in x if res == True]
        env.filters['fails'] = lambda x: [res for res in x if res != True]
        template = env.from_string(fp.read())
    html = template.render(
        status=status,
        date=datetime.datetime.utcnow(),
        test_results=test_results
    )
    with open(output_file, 'w') as fp:
        fp.write(html)

def is_feasible(params, test_fixtures):
    """
    Checks if the specified parameter and test fixture combination is feasible.
    A combination is feasible if none of the test fixture resources appear in 
    the parameters and if all of the exclusive-use test fixture resources are
    only used by one test fixture.
    """
    exclusive_tf_resources = []
    shared_tf_resources = set()
    for r, ex in [(r, ex) for tf in test_fixtures if not tf is None for r, ex in tf.get_resources() if not r is None]:
        if ex:
            exclusive_tf_resources.append(r)
        else:
            shared_tf_resources.add(r)
    if len(exclusive_tf_resources + list(shared_tf_resources)) > len(set(exclusive_tf_resources).union(shared_tf_resources)):
        return False # At least one exclusive-use resource is used twice in the test fixtures
    if len(set(exclusive_tf_resources).union(shared_tf_resources).intersection(params)):
        return False # At least one test fixture resource appears in the params too
    if len(shared_tf_resources.intersection(params)):
        return False # At least one test fixture resource appears in the params too
    return True

def run(tests):
    test_results = []

    if not isinstance(tests, list):
        tests = [tests]

    for test in tests:
        # The result of get_test_cases must be a generator or a list of which
        # each item is one of the following:
        #  - A tuple, each element of which is to be passed as an argument to
        #    the run_test() function, except the last argument. The last
        #    argument must be a TestFixture object.
        #  - An AnyTestCase object. In this case, each of the alternatives in
        #    the test case must follow the tuple form described above.
        #
        # All of the provided test-cases are executed. If none is provided,
        # a warning is reported. A warning is also reported if for a particular
        # test case no alternative is feasible (e.g. because there are component
        # conflicts).

        logger.debug("loading...")
        test_cases = list(test.get_test_cases(testrig))
        test_name = type(test).__name__
        test_case_results = []
        test_results.append((test_name, test_case_results))

        if len(test_cases) == 0:
            logger.warn(f'no test cases are available to conduct the test {test_name}')
            continue
        
        for test_case in test_cases:
            # Flatten all test case options and test fixture options into a list of candidates
            candidates = []
            for candidate in test_case.alternatives if isinstance(test_case, AnyTestCase) else [test_case]:
                test_fixture = candidate[-1]
                assert(isinstance(test_fixture, TestFixture) or test_fixture is None)
                if isinstance(test_fixture, CompositeTestFixture):
                    candidates += [tuple(candidate[:-1]) + (tf,) for tf in test_fixture._subfixtures]
                else:
                    candidates.append(tuple(candidate[:-1]) + (([] if test_fixture is None else [test_fixture]),))

            # Select the first candidate that is feasible
            params, test_fixture = (None, None)
            for candidate in candidates:
                if is_feasible(candidate[:-1], candidate[-1]):
                    params, test_fixture = (candidate[:-1], candidate[-1])
                    break

            if params is None:
                logger.warn(f'I found a {type(test).__name__} test case with {len(candidates)} possible parameter combination candidates but none of them is feasible.')
                continue

            logger.notify('* preparing {} with {}...'.format(type(test).__name__,
                    [(testrig.get_component_name(p) if isinstance(p, Component) else str(p)) for p in params]))

            # Prepare all components
            # TODO: less hardcoded priority assignment

            teensies = set()
            for param in params:
                #if isinstance(param, ProxiedComponent):
                #    continue
                if hasattr(param, 'prepare'):
                    param.prepare(logger)
            
            teensies = set()
            for tf in test_fixture:
                if isinstance(tf, ClosedLoopControlFixture):
                    continue
                tf.prepare(logger)
                if isinstance(tf, TeensyForwardingFixture):
                    teensies.add(tf.teensy)

            # Post-prepare step required if teensy-forwarding is involved
            for teensy in teensies:
                teensy.commit_routing_config(logger)

            for tf in test_fixture:
                if not isinstance(tf, ClosedLoopControlFixture):
                    continue
                tf.prepare(logger)

            logger.notify('* running {} on {}...'.format(type(test).__name__,
                    [(testrig.get_component_name(p) if isinstance(p, Component) else str(p)) for p in params]))

            try:
                test.run_test(*params, logger)
                test_case_results.append(True)
            except Exception as ex:
                traceback.print_exc()
                test_case_results.append(ex)

            if args.html:
                render_html_summary('running...', test_results, args.html)

    if args.html:
        render_html_summary('finished', test_results, args.html)

    passes = [res for t in test_results for res in t[1] if res == True]
    fails = [res for t in test_results for res in t[1] if res != True]
    if len(fails):
        logger.error(f'{len(fails)} out of {len(fails) + len(passes)} test cases failed.')
    else:
        logger.success('All tests passed!')
    shutdown_token.set()

    return test_results

# Load test engine ------------------------------------------------------------#

# Parse arguments
parser = argparse.ArgumentParser(description='ODrive automated test tool\n')
parser.add_argument("--ignore", metavar='DEVICE', action='store', nargs='+',
                    help="Ignore (disable) one or more components of the test rig")
                    # TODO: implement
parser.add_argument("--test-rig-yaml", type=argparse.FileType('r'),
                    help="Test rig YAML file. Can be omitted if the environment variable ODRIVE_TEST_RIG_NAME is set.")
parser.add_argument("--setup-host", action='store_true', default=False,
                    help="configure operating system functions such as GPIOs (requires root)")
parser.add_argument("--all", action='store_true', default=False,
                    help="Run all tests in the test runner's directory")
parser.add_argument("--html", type=str,
                    help="If provided, the an HTML summary is written to the specified file.")
parser.set_defaults(ignore=[])

args = parser.parse_args()

if args.test_rig_yaml is None:
    test_rig_name = os.environ.get('ODRIVE_TEST_RIG_NAME', '')
    if test_rig_name == '':
        print("You must either provide a --test-rig-yaml argument or set the environment variable ODRIVE_TEST_RIG_NAME.")
        sys.exit(1)
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), test_rig_name + '.yaml')
    args.test_rig_yaml = open(path, 'r')
    


# Load objects
test_rig_yaml = yaml.load(args.test_rig_yaml, Loader=yaml.BaseLoader)
logger = Logger()

testrig = TestRig(test_rig_yaml, logger)
shutdown_token = fibre.Event()

if args.setup_host:
    for gpio in testrig.get_components(LinuxGpioComponent):
        num = gpio.num
        logger.debug('exporting GPIO ' + str(num) + ' to user space...')
        if not os.path.isdir("/sys/class/gpio/gpio{}".format(num)):
            with open("/sys/class/gpio/export", "w") as fp:
                fp.write(str(num))
        os.chmod("/sys/class/gpio/gpio{}/value".format(num), stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO)
        os.chmod("/sys/class/gpio/gpio{}/direction".format(num), stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO)

    for port in testrig.get_components(SerialPortComponent):
        logger.debug('changing permissions on ' + port.yaml['port'] + '...')
        os.chmod(port.yaml['port'], stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO)

    if len(list(testrig.get_components(TeensyComponent))):
        # This breaks the annoying teensy loader that shows up on every compile
        logger.debug('modifying teensyduino installation...')
        if not os.path.isfile('/usr/share/arduino/hardware/tools/teensy_post_compile_old'):
            os.rename('/usr/share/arduino/hardware/tools/teensy_post_compile', '/usr/share/arduino/hardware/tools/teensy_post_compile_old')
        with open('/usr/share/arduino/hardware/tools/teensy_post_compile', 'w') as scr:
            scr.write('#!/usr/bin/env bash\n')
            scr.write('if [ "$ARDUINO_COMPILE_DESTINATION" != "" ]; then\n')
            scr.write('  cp -r ${2#-path=}/*.ino.hex ${ARDUINO_COMPILE_DESTINATION}\n')
            scr.write('fi\n')
        os.chmod('/usr/share/arduino/hardware/tools/teensy_post_compile', stat.S_IRWXU | stat.S_IRGRP | stat.S_IXGRP | stat.S_IROTH | stat.S_IXOTH)

    # Bring up CAN interface(s)
    for intf in testrig.get_components(CanInterfaceComponent):
        name = intf.yaml['interface']
        path = intf.yaml.get('path', None)
        logger.debug('bringing up {}...'.format(name))
        run_shell('ip link set dev {} down'.format(name), logger)
        if not path is None:
            run_shell(f'slcand -o -c -s5 \'{path}\' {name}', logger)
        else:
            run_shell('ip link set dev {} type can bitrate 250000'.format(name), logger)
            run_shell('ip link set dev {} type can loopback off'.format(name), logger)
        run_shell('ip link set dev {} up'.format(name), logger)


if __name__ == '__main__':
    test_scripts = []
    tests = []

    if args.all:
        test_scripts = [file for file in os.listdir(os.path.dirname(__file__))
                        if file.lower().endswith("_test.py")]

    for script in test_scripts:
        import importlib.util
        spec = importlib.util.spec_from_file_location("test_module", script)
        test_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(test_module)
        if not hasattr(test_module, 'tests') or not isinstance(test_module.tests, list):
            logger.error(f"{script} does not have a list named `tests`")
        tests += test_module.tests
    
    logger.notify(f"found {len(tests)} tests in {len(test_scripts)} modules")
    if any(tests):
        test_results = test_module.test_runner.run(tests)

