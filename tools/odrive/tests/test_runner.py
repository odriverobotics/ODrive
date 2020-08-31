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
from typing import Union, Tuple

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
        did_update = True
        while did_update:
            did_update = False
            for i, s in enumerate(list_of_sets):
                if len(current_set.intersection(s)):
                    current_set = current_set.union(s)
                    list_of_sets = list_of_sets[:i] + list_of_sets[(i+1):]
                    did_update = True
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

# Test Components -------------------------------------------------------------#

class Component(object):
    def __init__(self, parent):
        self.parent = parent

class ODriveComponent(Component):
    def __init__(self, yaml: dict):
        self.handle = None
        self.yaml = yaml
        #self.axes = [ODriveAxisComponent(None), ODriveAxisComponent(None)]
        self.encoders = [ODriveEncoderComponent(self, 0, yaml['encoder0']), ODriveEncoderComponent(self, 1, yaml['encoder1'])]
        self.axes = [ODriveAxisComponent(self, 0, yaml['motor0']), ODriveAxisComponent(self, 1, yaml['motor1'])]
        for i in range(1,9):
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
        for i in range(1,9):
            yield ('gpio' + str(i)), getattr(self, 'gpio' + str(i))
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
        self.handle = odrive.find_any(
            path="usb", serial_number=self.yaml['serial-number'], timeout=60)#, printer=print)
        assert(self.handle)
        #for axis_idx, axis_ctx in enumerate(self.axes):
        #    axis_ctx.handle = self.handle.__dict__['axis{}'.format(axis_idx)]
        for encoder_idx, encoder_ctx in enumerate(self.encoders):
            encoder_ctx.handle = self.handle.__dict__['axis{}'.format(encoder_idx)].encoder
        # TODO: distinguish between axis and motor context
        for axis_idx, axis_ctx in enumerate(self.axes):
            axis_ctx.handle = self.handle.__dict__['axis{}'.format(axis_idx)]

    def disable_mappings(self):
        self.handle.config.gpio1_pwm_mapping.endpoint = None # here
        self.handle.config.gpio2_pwm_mapping.endpoint = None
        self.handle.config.gpio3_pwm_mapping.endpoint = None
        self.handle.config.gpio4_pwm_mapping.endpoint = None
        self.handle.config.gpio3_analog_mapping.endpoint = None
        self.handle.config.gpio4_analog_mapping.endpoint = None

    def save_config_and_reboot(self):
        self.handle.save_configuration()
        try:
            self.handle.reboot()
        except fibre.ChannelBrokenException:
            pass # this is expected
        self.handle = None
        time.sleep(2)
        self.prepare(logger)

    def erase_config_and_reboot(self):
        try:
            self.handle.erase_configuration()
        except fibre.ChannelBrokenException:
            pass # this is expected
        self.handle = None
        time.sleep(2)
        self.prepare(logger)

class MotorComponent(Component):
    def __init__(self, yaml: dict):
        self.yaml = yaml

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

    def get_subcomponents(self):
        return [('z', self.z), ('a', self.a), ('b', self.b)]

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

    def get_subcomponents(self):
        yield 'tx', Component(self)
        yield 'rx', Component(self)

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
        self.gpios = [TeensyGpio(self, i) for i in range(24)]
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
            ['arduino', '--board', 'teensy:avr:teensy40', '--verify', sketchfile],
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
        
        run_shell(["teensy-loader-cli", "-mmcu=imxrt1062", "-w", hex_file_path], logger, timeout = 5)
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


class ProxiedComponent(Component):
    def __init__(self, impl, *gpio_tuples):
        """
        Each element in gpio_tuples should be a tuple of the form:
            (teensy: TeensyComponent, gpio_in, gpio_out, gpio_noise_enable)
        """
        Component.__init__(self, getattr(impl, 'parent', None))
        self.impl = impl
        assert(all([len(t) == 4 for t in gpio_tuples]))
        self.gpio_tuples = list(gpio_tuples)

    def __repr__(self):
        return testrig.get_component_name(self.impl) + ' (routed via ' + ', '.join((testrig.get_component_name(t) + ': ' + str(i.num) + ' => ' + str(o.num)) for t, i, o, n in self.gpio_tuples) + ')'

    def __eq__(self, obj):
        return isinstance(obj, ProxiedComponent) and (self.impl == obj.impl) #  and (self.gpio_tuples == obj.gpio_tuples)

    def prepare(self):
        for teensy, gpio_in, gpio_out, gpio_noise_enable in self.gpio_tuples:
            teensy.add_route(gpio_in, gpio_out, gpio_noise_enable)

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
            elif component_yaml['type'] == 'teensy':
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

    def get_components(self, t: type):
        """Returns a tuple (name, component) for all components that are of the specified type"""
        return (comp for comp in self.names_by_component.keys() if isinstance(comp, t))
    
    def get_component_name(self, component: Component):
        if isinstance(component, ProxiedComponent):
            return self.names_by_component[component.impl]
        else:
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

        if isinstance(src, dict):
            component_list = []
            for name, subsrc in src.items():
                component_list.append([c for c in self.get_connected_components(subsrc) if self.get_component_name(c).endswith('.' + name)])
            
            for combination in itertools.product(*component_list):
                if len(set(c.parent for c in combination)) != 1:
                    continue # parent of the components don't match
                proxied_dst = combination[0].parent
                if comp_type and not isinstance(proxied_dst, comp_type):
                    continue # not the requested type
                gpio_tuples = [c2 for c in combination for c2 in c.gpio_tuples if isinstance(c, ProxiedComponent)]
                if len(gpio_tuples):
                    yield ProxiedComponent(proxied_dst, *gpio_tuples)
                else:
                    yield proxied_dst

        else:

            if isinstance(src, tuple):
                src, dir = src
            else:
                dir = None

            for dst in self.get_directly_connected_components(src):
                if (not comp_type) or isinstance(dst, comp_type):
                    yield dst

                if (not dir is None) and isinstance(getattr(dst, 'parent', None), TeensyComponent):
                    teensy = dst.parent
                    for gpio2 in teensy.gpios:
                        for proxied_dst in self.get_directly_connected_components(gpio2):
                            if (not comp_type) or isinstance(proxied_dst, comp_type):
                                yield ProxiedComponent(proxied_dst, (teensy, dst if dir else gpio2, gpio2 if dir else dst, None))


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
                 axis_ctx.handle.sensorless_estimator.error |
                 axis_ctx.handle.error) != 0

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


def get_combinations(param_options):
    if isinstance(param_options, tuple):
        if len(param_options) > 0:
            for part1, part2 in itertools.product(
                    get_combinations(param_options[0]),
                    get_combinations(param_options[1:]) if (len(param_options) > 1) else [()]):
                assert(isinstance(part1, tuple))
                assert(isinstance(part2, tuple))
                yield part1 + part2
    elif is_list_like(param_options):
        for item in param_options:
            for c in get_combinations(item):
                yield c
    else:
        yield (param_options,)

def select_params(param_options):
    # Select parameters from the resource list
    # (this could be arbitrarily complex to improve parallelization of the tests)
    for combination in get_combinations(param_options):
        if all_unique([x for x in combination if isinstance(x, Component)]):
            return list(combination)

    return None

def run(tests):
    if not isinstance(tests, list):
        tests = [tests]

    for test in tests:
        # The result of get_test_cases can be described in ABNF grammar:
        #   test-case-list    = *arglist
        #   arglist           = *flexible-arg
        #   flexible-arg      = component / *argvariant
        #   argvariant        = component / arglist
        #
        # If for a particular test-case, the components are not given plainly
        # but in some selectable form, the test driver will select exactly one
        # of those options.
        # In other words, it will bring arglist from the form *flexible-arg
        # into the form *component before calling the test.
        #
        # All of the provided test-cases are executed. If none is provided,
        # a warning is reported. A warning is also reported if for a particular
        # test case no component combination can be resolved.

        test_cases = list(test.get_test_cases(testrig))

        if len(test_cases) == 0:
            logger.warn('no test cases are available to conduct the test {}'.format(type(test).__name__))
            continue
        
        for test_case in test_cases:
            params = select_params(test_case)
            if params is None:
                logger.warn('no resources are available to conduct the test {}'.format(type(test).__name__))
                continue

            logger.notify('* preparing {} with {}...'.format(type(test).__name__,
                    [(testrig.get_component_name(p) if isinstance(p, Component) else str(p)) for p in params]))
            
            teensies = set()
            for param in params:
                if isinstance(param, ProxiedComponent):
                    param.prepare()
                    for teensy, _, _, _ in param.gpio_tuples:
                        teensies.add(teensy)

            for teensy in teensies:
                teensy.commit_routing_config(logger)

            # prepare all components
            teensies = set()
            for param in params:
                if isinstance(param, ProxiedComponent):
                    continue
                if hasattr(param, 'prepare'):
                    param.prepare(logger)

            logger.notify('* running {} on {}...'.format(type(test).__name__,
                    [(testrig.get_component_name(p) if isinstance(p, Component) else str(p)) for p in params]))

            # Resolve routed components
            for i, param in enumerate(params):
                if isinstance(param, ProxiedComponent):
                    params[i] = param.impl

            test.run_test(*params, logger)


    logger.success('All tests passed!')


# Load test engine ------------------------------------------------------------#

# Parse arguments
parser = argparse.ArgumentParser(description='ODrive automated test tool\n')
parser.add_argument("--ignore", metavar='DEVICE', action='store', nargs='+',
                    help="Ignore (disable) one or more components of the test rig")
                    # TODO: implement
parser.add_argument("--test-rig-yaml", type=argparse.FileType('r'), required=True,
                    help="test rig YAML file")
parser.add_argument("--setup-host", action='store_true', default=False,
                    help="configure operating system functions such as GPIOs (requires root)")
parser.set_defaults(ignore=[])

args = parser.parse_args()

# Load objects
test_rig_yaml = yaml.load(args.test_rig_yaml, Loader=yaml.BaseLoader)
logger = Logger()

testrig = TestRig(test_rig_yaml, logger)


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
        logger.debug('bringing up {}...'.format(name))
        run_shell('ip link set dev {} down'.format(name), logger)
        run_shell('ip link set dev {} type can bitrate 250000'.format(name), logger)
        run_shell('ip link set dev {} type can loopback off'.format(name), logger)
        run_shell('ip link set dev {} up'.format(name), logger)

