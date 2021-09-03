
import test_runner

import struct
import can
import asyncio
import time
import math

from fibre.utils import Logger
from odrive.enums import *
from test_runner import *

# Each argument is described as tuple (name, format, scale).
# Struct format codes: https://docs.python.org/2/library/struct.html
command_set = {
    'heartbeat': (0x001, [('error', 'I', 1), ('current_state', 'B', 1), ('reserved', 'H', 1), ('controller_state', 'B', 1)]), # tested
    'estop': (0x002, []), # tested
    'get_motor_error': (0x003, [('motor_error', 'I', 1)]), # untested
    'get_encoder_error': (0x004, [('encoder_error', 'I', 1)]), # untested
    'get_sensorless_error': (0x005, [('sensorless_error', 'I', 1)]), # untested
    'set_node_id': (0x006, [('node_id', 'I', 1)]), # tested
    'set_requested_state': (0x007, [('requested_state', 'I', 1)]), # tested
    # 0x008 not yet implemented
    'get_encoder_estimates': (0x009, [('encoder_pos_estimate', 'f', 1), ('encoder_vel_estimate', 'f', 1)]), # partially tested
    'get_encoder_count': (0x00a, [('encoder_shadow_count', 'i', 1), ('encoder_count', 'i', 1)]), # partially tested
    'set_controller_modes': (0x00b, [('control_mode', 'i', 1), ('input_mode', 'i', 1)]), # tested
    'set_input_pos': (0x00c, [('input_pos', 'f', 1), ('vel_ff', 'h', 0.001), ('torque_ff', 'h', 0.001)]), # tested
    'set_input_vel': (0x00d, [('input_vel', 'f', 1), ('torque_ff', 'f', 1)]), # tested
    'set_input_torque': (0x00e, [('input_torque', 'f', 1)]), # tested
    'set_velocity_limit': (0x00f, [('velocity_limit', 'f', 1)]), # tested
    'start_anticogging': (0x010, []), # untested
    'set_traj_vel_limit': (0x011, [('traj_vel_limit', 'f', 1)]), # tested
    'set_traj_accel_limits': (0x012, [('traj_accel_limit', 'f', 1), ('traj_decel_limit', 'f', 1)]), # tested
    'set_traj_inertia': (0x013, [('inertia', 'f', 1)]), # tested
    'get_iq': (0x014, [('iq_setpoint', 'f', 1), ('iq_measured', 'f', 1)]), # untested
    'get_sensorless_estimates': (0x015, [('sensorless_pos_estimate', 'f', 1), ('sensorless_vel_estimate', 'f', 1)]), # untested
    'reboot': (0x016, []), # tested
    'get_vbus_voltage': (0x017, [('vbus_voltage', 'f', 1)]), # tested
    'clear_errors': (0x018, []), # partially tested
}

def command(bus, node_id_, extended_id, cmd_name, **kwargs):
    cmd_spec = command_set[cmd_name]
    cmd_id = cmd_spec[0]
    fmt = '<' + ''.join([f for (n, f, s) in cmd_spec[1]]) # all little endian

    if (sorted([n for (n, f, s) in cmd_spec[1]]) != sorted(kwargs.keys())):
        raise Exception("expected arguments: " + str([n for (n, f, s) in cmd_spec[1]]))

    fields = [((kwargs[n] / s) if f == 'f' else int(kwargs[n] / s)) for (n, f, s) in cmd_spec[1]]
    data = struct.pack(fmt, *fields)
    msg = can.Message(arbitration_id=((node_id_ << 5) | cmd_id), extended_id=extended_id, data=data)
    bus.send(msg)

async def record_messages(bus, node_id, extended_id, cmd_name, timeout = 5.0):
    """
    Returns an async generator that yields a dictionary for each CAN message that
    is received, provided that the CAN ID matches the expected value.
    """

    cmd_spec = command_set[cmd_name]
    cmd_id = cmd_spec[0]
    fmt = '<' + ''.join([f for (n, f, s) in cmd_spec[1]]) # all little endian

    reader = can.AsyncBufferedReader()
    notifier = can.Notifier(bus, [reader], timeout = timeout, loop = asyncio.get_event_loop())

    try:
        # The timeout in can.Notifier only triggers if no new messages are received at all,
        # so we need a second monitoring method.
        start = time.monotonic()
        while True:
            msg = await reader.get_message()
            if ((msg.arbitration_id == ((node_id << 5) | cmd_id)) and (msg.is_extended_id == extended_id) and not msg.is_remote_frame):
                fields = struct.unpack(fmt, msg.data[:(struct.calcsize(fmt))]) 
                res = {n: (fields[i] * s) for (i, (n, f, s)) in enumerate(cmd_spec[1])}
                res['t'] = time.monotonic()
                yield res
            if (time.monotonic() - start) > timeout:
                break
    finally:
        notifier.stop()

async def request(bus, node_id, extended_id, cmd_name, timeout = 1.0):
    cmd_spec = command_set[cmd_name]
    cmd_id = cmd_spec[0]

    msg_generator = record_messages(bus, node_id, extended_id, cmd_name, timeout)

    msg = can.Message(arbitration_id=((node_id << 5) | cmd_id), extended_id=extended_id, data=[], is_remote_frame=True)
    bus.send(msg)

    async for msg in msg_generator:
        return msg

    raise TimeoutError()

async def get_all(async_iterator):
    return [x async for x in async_iterator]


class TestSimpleCAN():
    def get_test_cases(self, testrig: TestRig):
        for odrive in testrig.get_components(ODriveComponent):
            can_interfaces = list(testrig.get_connected_components(odrive.can, CanInterfaceComponent))
            yield AnyTestCase(*[(odrive, intf, 0, False, tf) for intf, tf in can_interfaces]) # standard ID
            yield AnyTestCase(*[(odrive, intf, 0xfedcba, True, tf) for intf, tf in can_interfaces]) # extended ID

    def run_test(self, odrive: ODriveComponent, canbus: CanInterfaceComponent, node_id: int, extended_id: bool, logger: Logger):
        odrive.disable_mappings()
        if odrive.yaml['board-version'].startswith("v3."):
            odrive.handle.config.gpio15_mode = GPIO_MODE_CAN_A
            odrive.handle.config.gpio16_mode = GPIO_MODE_CAN_A
        elif odrive.yaml['board-version'].startswith("v4."):
            pass # CAN pin configuration is hardcoded
        else:
            raise Exception("unknown board version {}".format(odrive.yaml['board-version']))
        odrive.handle.config.enable_can_a = True
        odrive.save_config_and_reboot()

        axis = odrive.handle.axis0
        axis.config.enable_watchdog = False
        odrive.handle.clear_errors()
        axis.config.can.node_id = node_id
        axis.config.can.is_extended = extended_id
        time.sleep(0.1)
        
        def my_cmd(cmd_name, **kwargs): command(canbus.handle, node_id, extended_id, cmd_name, **kwargs)
        def my_req(cmd_name, **kwargs): return asyncio.run(request(canbus.handle, node_id, extended_id, cmd_name, **kwargs))
        def fence(): my_req('get_vbus_voltage') # fence to ensure the CAN command was sent
        def flush_rx():
            while not canbus.handle.recv(timeout = 0) is None: pass

        logger.debug('sending request...')
        test_assert_eq(my_req('get_vbus_voltage')['vbus_voltage'], odrive.handle.vbus_voltage, accuracy=0.01)

        my_cmd('set_node_id', node_id=node_id+20)
        time.sleep(0.1) # TODO: remove this hack (see note in firmware)
        asyncio.run(request(canbus.handle, node_id+20, extended_id, 'get_vbus_voltage'))
        test_assert_eq(axis.config.can.node_id, node_id+20)

        # Reset node ID to default value
        command(canbus.handle, node_id+20, extended_id, 'set_node_id', node_id=node_id)
        fence()
        test_assert_eq(axis.config.can.node_id, node_id)

        # Check that extended node IDs are not carelessly projected to 6-bit IDs
        extended_id = not extended_id
        my_cmd('estop') # should not be accepted
        extended_id = not extended_id
        fence()
        test_assert_eq(axis.error, AXIS_ERROR_NONE)

        axis.encoder.set_linear_count(123)
        flush_rx() # drop previous encoder estimates that were sent by the ODrive at a constant rate
        test_assert_eq(my_req('get_encoder_estimates')['encoder_pos_estimate'], 123.0 / axis.encoder.config.cpr, accuracy=0.01)
        test_assert_eq(my_req('get_encoder_count')['encoder_shadow_count'], 123.0, accuracy=0.01)

        my_cmd('clear_errors')
        fence()
        test_assert_eq(axis.error, 0)

        my_cmd('estop')
        fence()
        test_assert_eq(axis.error, AXIS_ERROR_ESTOP_REQUESTED)

        my_cmd('set_requested_state', requested_state=42) # illegal state - should assert axis error
        fence()
        test_assert_eq(axis.current_state, 1) # idle
        test_assert_eq(axis.error, AXIS_ERROR_ESTOP_REQUESTED | AXIS_ERROR_INVALID_STATE)

        my_cmd('clear_errors')
        fence()
        test_assert_eq(axis.error, 0)

        my_cmd('set_controller_modes', control_mode=1, input_mode=5) # current conrol, traprzoidal trajectory
        fence()
        test_assert_eq(axis.controller.config.control_mode, 1)
        test_assert_eq(axis.controller.config.input_mode, 5)

        # Reset to safe values
        my_cmd('set_controller_modes', control_mode=3, input_mode=1) # position control, passthrough
        fence()
        test_assert_eq(axis.controller.config.control_mode, 3)
        test_assert_eq(axis.controller.config.input_mode, 1)

        axis.controller.input_pos = 1234
        axis.controller.input_vel = 1234
        axis.controller.input_torque = 1234
        my_cmd('set_input_pos', input_pos=1.23, vel_ff=1.2, torque_ff=3.4)
        fence()
        test_assert_eq(axis.controller.input_pos, 1.23, range=0.1)
        test_assert_eq(axis.controller.input_vel, 1.2, range=0.01)
        test_assert_eq(axis.controller.input_torque, 3.4, range=0.001)

        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        my_cmd('set_input_vel', input_vel=-10.5, torque_ff=0.1234)
        fence()
        test_assert_eq(axis.controller.input_vel, -10.5, range=0.01)
        test_assert_eq(axis.controller.input_torque, 0.1234, range=0.01)

        axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        my_cmd('set_input_torque', input_torque=0.1)
        fence()
        test_assert_eq(axis.controller.input_torque, 0.1, range=0.01)

        my_cmd('set_velocity_limit', velocity_limit=2.345678)
        fence()
        test_assert_eq(axis.controller.config.vel_limit, 2.345678, range=0.001)

        my_cmd('set_traj_vel_limit', traj_vel_limit=123.456)
        fence()
        test_assert_eq(axis.trap_traj.config.vel_limit, 123.456, range=0.0001)

        my_cmd('set_traj_accel_limits', traj_accel_limit=98.231, traj_decel_limit=-12.234)
        fence()
        test_assert_eq(axis.trap_traj.config.accel_limit, 98.231, range=0.0001)
        test_assert_eq(axis.trap_traj.config.decel_limit, -12.234, range=0.0001)

        my_cmd('set_traj_inertia', inertia=55.086)
        fence()
        test_assert_eq(axis.controller.config.inertia, 55.086, range=0.0001)

        # any CAN cmd will feed the watchdog
        test_watchdog(axis, lambda: my_cmd('set_input_torque', input_torque=0.0), logger)

        logger.debug('testing heartbeat...')
        flush_rx() # Flush RX buffer to get a clean state for the heartbeat test
        heartbeats = asyncio.run(get_all(record_messages(canbus.handle, node_id, extended_id, 'heartbeat', timeout = 2.0)))
        test_assert_eq(len(heartbeats), 2.0 / 0.1, accuracy=0.05)
        test_assert_eq([msg['error'] for msg in heartbeats], [AXIS_ERROR_WATCHDOG_TIMER_EXPIRED] * len(heartbeats))
        test_assert_eq([msg['current_state'] for msg in heartbeats], [1] * len(heartbeats))

        logger.debug('testing reboot...')
        test_assert_eq(odrive.handle._on_lost.done(), False)
        my_cmd('reboot')
        time.sleep(0.5)
        if not odrive.handle._on_lost is None:
            raise TestFailed("device didn't seem to reboot")
        odrive.handle = None
        time.sleep(2.0)
        odrive.prepare(logger)

tests = [TestSimpleCAN()]

if __name__ == '__main__':
    test_runner.run(tests)
