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
import odrive.usbbulk
import re
import serial
import time
import os
import odrive.protocol
import itertools
import struct
import functools

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



def call_remote_function(channel, trigger_id, arg_properties, *args):
    if (len(arg_properties) != len(args)):
        raise TypeError("expected {} arguments but have {}".format(len(arg_properties), len(args)))
    for i in range(len(args)):
        arg_properties[i].fset(None, args[i])
    channel.remote_endpoint_operation(trigger_id, None, True, 0)

def raise_if_undefined(self, name, value):
    if hasattr(self, name):
        object.__setattr__(self, name, value)
    else:
        raise TypeError('Cannot set name %r on object of type %s' % (
                        name, self.__class__.__name__))

def create_property(name, json_data, channel, printer):
    name = name or "[anonymous]"

    type_str = json_data.get("type", None)
    if type_str is None:
        printer("property {} has no specified type".format(name))
        return None

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
        printer("property {} has unsupported type {}".format(name, type_str))
        return None

    id_str = json_data.get("id", None)
    if id_str is None:
        printer("property {} has no specified ID".format(name))
        return None

    access_mode = json_data.get("mode", "rw")
    return SimpleDeviceProperty(channel, id_str, property_type,
                                struct_format,
                                'r' in access_mode,
                                'w' in access_mode)

def create_function(name, json_data, channel, printer):
    id_str = json_data.get("id", None)
    if id_str is None:
        printer("function {} has no specified ID".format(name))
        return None

    inputs = []
    for param in json_data.get("arguments", []):
        param["mode"] = "r"
        inputs.append(create_property(json_data["name"], param, channel, printer))
    return functools.partial(call_remote_function, channel, id_str, inputs)

def create_object(name, json_data, namespace, channel, printer=noprint):
    """
    Creates an object that implements the specified JSON type description by
    communicating with the provided device object
    """

    if not namespace is None:
        namespace = namespace + "." + name
    else:
        namespace = name

    # Build attribute list from JSON
    attributes = {"__setattr__": raise_if_undefined}
    for member in json_data.get("members", []):
        member_name = member.get("name", None)
        if member_name is None:
            printer("ignoring unnamed attribute in {}".format(namespace))
            continue

        type_str = member.get("type", None)
        if type_str is None:
            printer("member {} has no specified type".format(member_name))
            continue

        if type_str == "object":
            attribute = create_object(member_name, member, namespace, channel, printer=printer)
        elif type_str == "function":
            attribute = create_function(member_name, member, channel, printer)
        else:
            attribute = create_property(member_name, member, channel, printer)
        
        if not attribute is None:
            attributes[member_name] = attribute

    # Create a type from the property list and instantiate it
    jit_type = type(namespace, (object,), attributes)
    new_object = jit_type()
    return new_object

class SerialDevice(odrive.protocol.StreamSource, odrive.protocol.StreamSink):
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
        try:
            printer(bulk_device.info())
            bulk_device.init(printer)
        except usb.core.USBError as ex:
            if ex.errno == 13:
                printer("USB device access denied. Did you set up your udev rules correctly?")
                continue
            else:
                raise
        yield odrive.protocol.Channel(
                "USB device bus {} device {}".format(usb_device.bus, usb_device.address),
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
        input_stream = odrive.protocol.PacketFromStreamConverter(serial_device)
        output_stream = odrive.protocol.PacketToStreamConverter(serial_device)
        yield odrive.protocol.Channel(
                "serial port {}@{}".format(serial_port, 115200),
                input_stream, output_stream)


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
        channel._interface_definition_crc = json_crc16
        try:
            json_string = json_bytes.decode("ascii")
        except UnicodeDecodeError:
            printer("device responded on endpoint 0 with something that is not ASCII")
            continue
        printer("JSON: " + json_string)
        try:
            json_data = json.loads(json_string)
        except json.decoder.JSONDecodeError:
            printer("device responded on endpoint 0 with something that is not JSON")
            continue
        json_data = {"name": "odrive", "members": json_data}
        yield create_object("odrive", json_data, None, channel, printer=printer)


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
