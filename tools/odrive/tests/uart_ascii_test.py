
import test_runner

import struct
import time
import os
import io
import serial
import functools
import operator

from fibre.utils import Logger
from odrive.enums import errors
from test_runner import ODriveTestContext, test_assert_eq, program_teensy


def append_checksum(command):
    return command + b'*' + str(functools.reduce(operator.xor, command)).encode('ascii')

def strip_checksum(command):
    command, _, checksum = command.partition(b'*')
    test_assert_eq(int(checksum.strip()), functools.reduce(operator.xor, command))
    return command

def reset_state(ser):
    """Resets the state of the ASCII protocol by flushing all buffers"""
    ser.write(b'\n')
    ser.flushOutput()
    time.sleep(0.1)
    ser.flushInput()

class TestUartAscii():
    def is_compatible(self, odrive: ODriveTestContext):
        return True

    def run_test(self, odrive: ODriveTestContext, logger: Logger):
        """
        Tests the most important functions of the ASCII protocol.
        """

        # Disable noise
        with open("/sys/class/gpio/gpio{}/direction".format(20), "w") as fp:
            fp.write("out")
        with open("/sys/class/gpio/gpio{}/value".format(20), "w") as gpio:
            gpio.write("0")

        hexfile = 'uart_pass_through.ino.hex'
        program_teensy(os.path.join(os.path.dirname(__file__), hexfile), 26, logger)
        time.sleep(1.0)

        odrive.handle.axis0.config.enable_step_dir = False
        odrive.handle.config.enable_uart = True

        with serial.Serial('/dev/ttyS0', 115200, timeout=1) as ser:
            # reset port to known state
            reset_state(ser)

            # Read a top-level attribute
            ser.write(b'r vbus_voltage\n')
            response = float(ser.readline().strip())
            test_assert_eq(response, odrive.handle.vbus_voltage, accuracy=0.1)

            # Read an unknown attribute
            ser.write(b'r blahblah\n')
            response = ser.readline().strip()
            test_assert_eq(response, b'invalid property')

            # Send command with delays in between
            for byte in b'r vbus_voltage\n':
                ser.write([byte])
                time.sleep(0.1)
            response = float(ser.readline().strip())
            test_assert_eq(response, odrive.handle.vbus_voltage, accuracy=0.1)

            # Test GCode checksum and comments
            ser.write(b'r vbus_voltage *12\n') # invalid checksum
            test_assert_eq(ser.readline(), b'')
            ser.write(append_checksum(b'r vbus_voltage ') + b' ; this is a comment\n') # valid checksum
            response = float(strip_checksum(ser.readline()).strip())
            test_assert_eq(response, odrive.handle.vbus_voltage, accuracy=0.1)

            # Read an attribute with a long name
            ser.write(b'r axis0.motor.current_control.v_current_control_integral_d\n')
            response = float(ser.readline().strip())
            test_assert_eq(response, odrive.handle.axis0.motor.current_control.v_current_control_integral_d, accuracy=0.1)


        # ascii: `r vbus_voltage`



class TestUartNoise():
    def is_compatible(self, odrive: ODriveTestContext):
        return True

    def run_test(self, odrive: ODriveTestContext, logger: Logger):
        """
        Tests the most important functions of the ASCII protocol.
        """

        # Disable noise
        with open("/sys/class/gpio/gpio{}/direction".format(20), "w") as fp:
            fp.write("out")
        with open("/sys/class/gpio/gpio{}/value".format(20), "w") as gpio:
            gpio.write("0")

        hexfile = 'uart_pass_through.ino.hex'
        program_teensy(os.path.join(os.path.dirname(__file__), hexfile), 26, logger)
        time.sleep(1.0)

        odrive.handle.axis0.config.enable_step_dir = False
        odrive.handle.config.enable_uart = True

        with serial.Serial('/dev/ttyS0', 115200, timeout=1) as ser:
            # reset port to known state
            reset_state(ser)

            # Enable square wave of ~1.6MHz on the ODrive's RX line
            with open("/sys/class/gpio/gpio{}/value".format(20), "w") as gpio:
                gpio.write("1")

            time.sleep(0.1)
            reset_state(ser)

            # Read an attribute (should fail because the command is not passed through)
            ser.write(b'r vbus_voltage\n')
            test_assert_eq(ser.readline(), b'')

            # Disable square wave
            with open("/sys/class/gpio/gpio{}/value".format(20), "w") as gpio:
                gpio.write("0")

            # Give receiver some time to recover
            time.sleep(0.1)

            # reset port to known state
            reset_state(ser)

            # Try again
            ser.write(b'r vbus_voltage\n')
            response = float(ser.readline().strip())
            test_assert_eq(response, odrive.handle.vbus_voltage, accuracy=0.1)


if __name__ == '__main__':
    test_runner.run([
        TestUartAscii(),
        TestUartNoise()
    ])
