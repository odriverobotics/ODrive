"""
Provides functions for the discovery of ODrive devices
"""

import sys
import time
import json
import usb.core
import usb.util
import odrive.usbbulk
import odrive.mock_device
import odrive.util
import re
import serial
import time
import os
import odrive.protocol
import itertools
import struct

def noprint(x):
  pass


class SimpleDeviceProperty(property):
    def __init__(self, channel, id, type, struct_format, can_read, can_write):
        self._channel = channel
        self._id = id
        self._type = type
        self._struct_format = struct_format
        property.__init__(self,
            self.fget if can_read else None,
            self.fset if can_write else None)

    def fget(self, obj):
        size = struct.calcsize(self._struct_format)
        buffer = self._channel.remote_endpoint_operation(self._id, None, True, size)
        return struct.unpack(self._struct_format, buffer)[0]

    def fset(self, obj, value):
        if not isinstance(value, self._type):
            raise TypeError("expected value of type {}".format(self._type.__name__))
        buffer = struct.pack(self._struct_format, value)
        # TODO: Currenly we wait for an ack here. Settle on the default guarantee.
        self._channel.remote_endpoint_operation(self._id, buffer, True, 0)


def create_object(json_data, namespace, channel):
    """
    Creates an object that implements the specified JSON type description by
    communicating with the provided device object
    """

    # Build property list from JSON
    properties = {}
    for item in json_data:
        name = item.get("name", None)
        if name is None:
            sys.stderr.write("unnamed property in {}".format(namespace))
            continue

        type_str = item.get("type", None)
        if type_str is None:
            sys.stderr.write("property {} has no specified type".format(name))
            continue

        if type_str == "tree":
            properties[name] = create_object(item["content"], namespace + "." + item["name"], channel)
        else:
            if type_str == "float":
                property_type = float
                struct_format = "<f"
            elif type_str == "int":
                property_type = int
                struct_format = "<i"
            elif type_str == "bool":
                property_type = bool
                struct_format = "<?"
            elif type_str == "uint16":
                property_type = int
                struct_format = "<H"
            else:
                sys.stderr.write("property {} has unsupported type {}".format(name, type_str))
                continue

            id_str = item.get("id", None)
            if id_str is None:
                sys.stderr.write("property {} has specified ID".format(name))
                continue

            access_mode = item.get("mode", "rw")
            properties[name] = SimpleDeviceProperty(channel, id_str, property_type,
                                            struct_format,
                                            'r' in access_mode,
                                            'w' in access_mode)

    # Create a type from the property list and instantiate it
    jit_type = type(namespace, (object,), properties)
    new_object = jit_type()
    return new_object

class SerialDevice(odrive.protocol.StreamReader, odrive.protocol.StreamWriter):
    def __init__(self, port, baud):
        self._dev = serial.Serial(port, baud, timeout=1)

    def write_bytes(self, bytes):
        self._dev.write(bytes)

    def read_bytes(self, n_bytes, deadline):
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

    def read_bytes_or_fail(self, n_bytes, deadline):
        result = self.read_bytes(n_bytes, deadline)
        if len(result) < n_bytes:
            raise odrive.protocol.TimeoutException()
        return result


def find_usb_channels(vid_pid_pairs=odrive.util.USB_VID_PID_PAIRS, printer=noprint):
    """
    Scans for compatible USB devices.
    Returns a generator of odrive.protocol.Channel objects.
    """
    for vid_pid_pair in vid_pid_pairs:
        usb_device = usb.core.find(idVendor=vid_pid_pair[0], idProduct=vid_pid_pair[1])
        if usb_device is None:
            continue
        printer("Found ODrive via PyUSB")
        bulk_device = odrive.usbbulk.USBBulkDevice(usb_device, printer)
        printer(bulk_device.info())
        bulk_device.init(printer)
        #Oskar: Here bulk_device is expected to have a write_packet function, but it doesnt, it has read/write_bytes.
        # We can either also use a stream converter here, but I'd prefer to write packets directly.
        yield odrive.protocol.Channel(
                "USB device {}:{}".format(vid_pid_pair[0], vid_pid_pair[1]),
                bulk_device, bulk_device)

def find_serial_channels(printer=noprint):
    """
    Scans for serial devices.
    Returns a generator of odrive.protocol.Channel objects.
    Not every returned object necessarily represents a compatible device.
    """
    return
    # Look for serial device
    # TODO: OS specific heuristic to find serial ports
    for serial_port in filter(re.compile(r'^tty\.usbmodem').search, os.listdir('/dev')):
        serial_port = '/dev/' + serial_port
        # If this is actually a USB device, the baudrate setting has no effect
        try:
            serial_device = SerialDevice(serial_port, 115200)
        except serial.serialutil.SerialException:
            printer("could not open " + serial_port)
            continue
        input = odrive.protocol.PacketFromStreamConverter(serial_device)
        output = odrive.protocol.PacketToStreamConverter(serial_device)
        yield odrive.protocol.Channel(
                "serial port {}@{}".format(serial_port, 115200),
                input, output)


def find_all(printer=noprint):
    """
    Returns a generator with all the connected devices that speak the ODrive protocol
    """
    usb_channels = find_usb_channels(printer=printer)
    serial_channels = find_serial_channels(printer=printer)
    for channel in itertools.chain(usb_channels, serial_channels):
        # TODO: blacklist known bad channels
        printer("Connecting to device on " + channel._name)
        try:
            #Oskar: The fact that the JSON is on endpoint 0 is kind of protocol internal.
            # Instead you can make a function on Channel called get_json.
            json_bytes = channel.remote_endpoint_read_buffer(0)
        except (odrive.protocol.TimeoutException, odrive.protocol.ChannelBrokenException):
            printer("no response - probably incompatible")
            continue
        json_crc16 = odrive.protocol.calc_crc16(odrive.protocol.PROTOCOL_VERSION, json_bytes)
        channel._interface_definition_crc = struct.pack("<H", json_crc16)
        try:
            json_string = json_bytes.decode("ascii")
        except UnicodeDecodeError:
            printer("device responded on endpoint 0 with something that is not ASCII")
            continue
        #printer("JSON: " + json_string)
        try:
            json_data = json.loads(json_string)
        except json.decoder.JSONDecodeError:
            printer("device responded on endpoint 0 with something that is not JSON")
            continue
        yield create_object(json_data, "odrive", channel)


def find_any(printer=noprint):
    """
    Scans for ODrives on all supported interfaces and returns the first device
    that is found. If no device is connected the function blocks.
    """
    # TODO: do device discovery and instantiation in a separate thread and just wait on a semaphore here

    # poll for device
    printer("looking for ODrive...")
    while True:
        dev = next(find_all(printer=printer), None)
        if dev is not None:
            return dev
        printer("no device found")
        time.sleep(1)
