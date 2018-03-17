"""
Provides classes that implement the StreamSource/StreamSink and
PacketSource/PacketSink interfaces for serial ports.
"""

import os
import re
import time
import serial
import serial.tools.list_ports
import odrive.protocol

ODRIVE_BAUDRATE = 115200

class SerialStreamTransport(odrive.protocol.StreamSource, odrive.protocol.StreamSink):
    def __init__(self, port, baud):
        self._dev = serial.Serial(port, baud, timeout=1)

    def process_bytes(self, bytes):
        self._dev.write(bytes)

    def get_bytes(self, n_bytes, deadline):
        """
        Returns n bytes unless the deadline is reached, in which case the bytes
        that were read up to that point are returned. If deadline is None the
        function blocks forever. A deadline before the current time corresponds
        to non-blocking mode.
        """
        if deadline is None:
            self._dev.timeout = None
        else:
            self._dev.timeout = max(deadline - time.monotonic(), 0)
        return self._dev.read(n_bytes)

    def get_bytes_or_fail(self, n_bytes, deadline):
        result = self.get_bytes(n_bytes, deadline)
        if len(result) < n_bytes:
            raise odrive.protocol.TimeoutException("expected {} bytes but got only {}", n_bytes, len(result))
        return result


def find_dev_serial_ports():
    try:
        return ['/dev/' + x for x in os.listdir('/dev')]
    except FileNotFoundError:
        return []

def find_pyserial_ports():
    return [x.device for x in serial.tools.list_ports.comports()]


def discover_channels(path, serial_number, callback, cancellation_token, printer):
    """
    Scans for serial ports that match the path spec.
    This function blocks until cancellation_token is set.
    """
    if path == None:
        # This regex should match all desired port names on macOS,
        # Linux and Windows but might match some incorrect port names.
        regex = r'^(/dev/tty\.usbmodem.*|/dev/ttyACM.*|COM[0-9]+)$'
    else:
        regex = "^" + path + "$"

    known_devices = []
    def device_matcher(port_name):
        if port_name in known_devices:
            return False
        return bool(re.match(regex, port_name))

    while not cancellation_token.is_set():
        all_ports = find_pyserial_ports() + find_dev_serial_ports()
        new_ports = filter(device_matcher, all_ports)
        for port_name in new_ports:
            serial_device = SerialStreamTransport(port_name, ODRIVE_BAUDRATE)
            input_stream = odrive.protocol.PacketFromStreamConverter(serial_device)
            output_stream = odrive.protocol.PacketToStreamConverter(serial_device)
            channel = odrive.protocol.Channel(
                    "serial port {}@{}".format(port_name, ODRIVE_BAUDRATE),
                    input_stream, output_stream)
            channel.serial_device = serial_device
            callback(channel)
            known_devices.append(port_name)
        time.sleep(1)
