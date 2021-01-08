# this test runs the motor using CAN
# TODO - run a motor using all common use cases (uart, step/dir, pwm)

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
    'heartbeat': (0x001, [('error', 'I', 1), ('current_state', 'I', 1)]), # tested
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

class TestSimpleCANClosedLoop():
    def prepare(self, odrive: ODriveComponent, canbus: CanInterfaceComponent, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, node_id: int, extended_id: bool, logger: Logger):
        # Make sure there are no funny configurations active
        logger.debug('Setting up clean configuration...')
        axis_ctx.parent.erase_config_and_reboot()
        axis_ctx.parent.handle.config.enable_brake_resistor = True
        axis_ctx.parent.save_config_and_reboot()

        # run calibration
        axis_ctx.handle.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while axis_ctx.handle.requested_state != AXIS_STATE_UNDEFINED or axis_ctx.handle.current_state != AXIS_STATE_IDLE:
            time.sleep(1)
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


    def get_test_cases(self, testrig: TestRig):
        for axis, motor, encoder, tf1 in testrig.get_closed_loop_combos(init=False):
            yield AnyTestCase(*[
                (axis.parent, canbus, axis, motor, encoder, 0, False, TestFixture.all_of(tf1, tf2))
                for canbus, tf2 in testrig.get_connected_components(axis.parent.can, CanInterfaceComponent)
            ])

    def run_test(self, odrive: ODriveComponent, canbus: CanInterfaceComponent, axis_ctx: ODriveAxisComponent, motor_ctx: MotorComponent, enc_ctx: EncoderComponent, node_id: int, extended_id: bool, logger: Logger):
        # this test is a sanity check to make sure that closed loop operation works
        # actual testing of closed loop functionality should be tested using closed_loop_test.py

        with self.prepare(odrive, canbus, axis_ctx, motor_ctx, enc_ctx, node_id, extended_id, logger):
            def my_cmd(cmd_name, **kwargs): command(canbus.handle, node_id, extended_id, cmd_name, **kwargs)
            def my_req(cmd_name, **kwargs): return asyncio.run(request(canbus.handle, node_id, extended_id, cmd_name, **kwargs))
            def fence(): my_req('get_vbus_voltage') # fence to ensure the CAN command was sent
            def flush_rx():
                while not canbus.handle.recv(timeout = 0) is None: pass
            
            axis_ctx.handle.config.enable_watchdog = False
            odrive.handle.clear_errors()
            axis_ctx.handle.config.can.node_id = node_id
            axis_ctx.handle.config.can.is_extended = extended_id
            time.sleep(0.1)

            my_cmd('set_node_id', node_id=node_id+20)
            flush_rx()
            asyncio.run(request(canbus.handle, node_id+20, extended_id, 'get_vbus_voltage'))
            test_assert_eq(axis_ctx.handle.config.can.node_id, node_id+20)

            # Reset node ID to default value
            command(canbus.handle, node_id+20, extended_id, 'set_node_id', node_id=node_id)
            fence()
            test_assert_eq(axis_ctx.handle.config.can.node_id, node_id)

            vel_limit = 15.0
            nominal_vel = 10.0
            axis_ctx.handle.controller.config.vel_limit = vel_limit
            axis_ctx.handle.motor.config.current_lim = 20.0

            my_cmd('set_requested_state', requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL)
            fence()
            test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_CLOSED_LOOP_CONTROL)
            test_assert_no_error(axis_ctx)

            start_pos = axis_ctx.handle.encoder.pos_estimate

            # position test
            logger.debug('Position control test')
            my_cmd('set_controller_modes', control_mode=CONTROL_MODE_POSITION_CONTROL, input_mode=INPUT_MODE_PASSTHROUGH) # position control, passthrough
            fence()
            my_cmd('set_input_pos', input_pos=1.0, vel_ff=0, torque_ff=0)
            fence()
            test_assert_eq(axis_ctx.handle.controller.input_pos, 1.0, range=0.1)
            time.sleep(2)
            test_assert_eq(axis_ctx.handle.encoder.pos_estimate, start_pos + 1.0, range=0.1)
            my_cmd('set_input_pos', input_pos=0, vel_ff=0, torque_ff=0)
            fence()
            time.sleep(2)

            test_assert_no_error(axis_ctx)

            # velocity test
            logger.debug('Velocity control test')
            my_cmd('set_controller_modes', control_mode=CONTROL_MODE_VELOCITY_CONTROL, input_mode=INPUT_MODE_PASSTHROUGH) # velocity control, passthrough
            fence()
            my_cmd('set_input_vel', input_vel = nominal_vel, torque_ff=0)
            fence()
            time.sleep(5)
            test_assert_eq(axis_ctx.handle.encoder.vel_estimate, nominal_vel, range=nominal_vel * 0.05) # big range here due to cogging and other issues
            my_cmd('set_input_vel', input_vel = 0, torque_ff=0)
            fence()
            time.sleep(2)

            test_assert_no_error(axis_ctx)

            # torque test
            logger.debug('Torque control test')
            my_cmd('set_controller_modes', control_mode=CONTROL_MODE_TORQUE_CONTROL, input_mode=INPUT_MODE_PASSTHROUGH) # torque control, passthrough
            fence()
            my_cmd('set_input_torque', input_torque=0.5)
            fence()
            time.sleep(5)
            test_assert_eq(axis_ctx.handle.controller.input_torque, 0.5, range=0.1)
            my_cmd('set_input_torque', input_torque = 0)
            fence()
            time.sleep(2)

            test_assert_no_error(axis_ctx)

            # go back to idle
            my_cmd('set_requested_state', requested_state = AXIS_STATE_IDLE)
            fence()
            test_assert_eq(axis_ctx.handle.current_state, AXIS_STATE_IDLE)

tests = [TestSimpleCANClosedLoop()]

if __name__ == '__main__':
    test_runner.run(tests)