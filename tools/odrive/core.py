"""
Provides functions for the discovery of ODrive devices
"""

import sys
import time
import json
import usb.core
import usb.util
import serial
import serial.tools.list_ports
import odrive.util
import odrive.usbbulk_transport
import odrive.serial_transport
import re
import time
import os
import odrive.protocol
import itertools
import struct
import functools

def noprint(x):
  pass


class SimpleDeviceProperty(property):
    """
    Used internally by dynamically created objects to translate
    property assignments and fetches into endpoint operations on the
    object's associated channel
    """
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
        value = self._type(value)
        buffer = struct.pack(self._struct_format, value)
        # TODO: Currenly we wait for an ack here. Settle on the default guarantee.
        self._channel.remote_endpoint_operation(self._id, buffer, True, 0)

def call_remote_function(channel, trigger_id, arg_properties, *args):
    """
    Used internally by the dynamically created objects to translate
    function calls into endpoint operations on the associated channel
    """
    if (len(arg_properties) != len(args)):
        raise TypeError("expected {} arguments but have {}".format(len(arg_properties), len(args)))
    for i in range(len(args)):
        arg_properties[i].fset(None, args[i])
    channel.remote_endpoint_operation(trigger_id, None, True, 0)

def setattr_or_raise_if_undefined(self, name, value):
    """
    If employed as an object's __setattr__ function, this function
    makes sure that an assignment to an undefined attribute doesn't
    create a new attribute but instead raises an exception
    """
    # We can't use hasattr here because internally it fetches the property
    # value, creating unnecessary bus traffic
    if name in dir(self):
        object.__setattr__(self, name, value)
    else:
        raise TypeError('Cannot set name %r on object of type %s' % (
                        name, self.__class__.__name__))

def create_property(name, json_data, channel, printer):
    """
    Dynamically creates a property based on a JSON definition
    """
    name = name or "[anonymous]"

    type_str = json_data.get("type", None)
    if type_str is None:
        printer("property {} has no specified type".format(name))
        return None

    if type_str == "float":
        property_type = float
        struct_format = "<f"
    elif type_str == "bool":
        property_type = bool
        struct_format = "<?"
    elif type_str == "int8":
        property_type = int
        struct_format = "<b"
    elif type_str == "uint8":
        property_type = int
        struct_format = "<B"
    elif type_str == "int16":
        property_type = int
        struct_format = "<h"
    elif type_str == "uint16":
        property_type = int
        struct_format = "<H"
    elif type_str == "int32":
        property_type = int
        struct_format = "<i"
    elif type_str == "uint32":
        property_type = int
        struct_format = "<I"
    else:
        printer("property {} has unsupported type {}".format(name, type_str))
        return None

    id_str = json_data.get("id", None)
    if id_str is None:
        printer("property {} has no specified ID".format(name))
        return None

    access_mode = json_data.get("access", "r")
    return SimpleDeviceProperty(channel, id_str, property_type,
                                struct_format,
                                'r' in access_mode,
                                'w' in access_mode)

def create_function(name, json_data, channel, printer):
    """
    Dynamically creates a function based on a JSON definition
    """
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
    attributes = {"__setattr__": setattr_or_raise_if_undefined}
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


def channel_from_usb_device(usb_device, printer=noprint):
    """
    Inits an ODrive Protocol channel from a PyUSB device object.
    """
    bulk_device = odrive.usbbulk_transport.USBBulkTransport(usb_device, printer)
    printer(bulk_device.info())
    bulk_device.init(printer)
    return odrive.protocol.Channel(
            "USB device bus {} device {}".format(usb_device.bus, usb_device.address),
            bulk_device, bulk_device)

def channel_from_serial_port(port, baud, packet_based, printer=noprint):
    """
    Inits an ODrive Protocol channel from a serial port name and baudrate.
    """
    if packet_based == True:
        # TODO: implement packet based transport over serial
        raise NotImplementedError("not supported yet")
    serial_device = odrive.serial_transport.SerialStreamTransport(port, baud)
    input_stream = odrive.protocol.PacketFromStreamConverter(serial_device)
    output_stream = odrive.protocol.PacketToStreamConverter(serial_device)
    return odrive.protocol.Channel(
            "serial port {}@{}".format(port, baud),
            input_stream, output_stream)

def object_from_channel(channel, printer=noprint):
    """
    Inits an object from a given channel.
    This queries the endpoint 0 on that channel to gain information
    about the interface, which is then used to init the corresponding object.
    """
    printer("Connecting to device on " + channel._name)
    try:
        json_bytes = channel.remote_endpoint_read_buffer(0)
    except (odrive.protocol.TimeoutException, odrive.protocol.ChannelBrokenException):
        raise odrive.protocol.DeviceInitException("no response - probably incompatible")
    json_crc16 = odrive.protocol.calc_crc16(odrive.protocol.PROTOCOL_VERSION, json_bytes)
    channel._interface_definition_crc = json_crc16
    try:
        json_string = json_bytes.decode("ascii")
    except UnicodeDecodeError:
        raise odrive.protocol.DeviceInitException("device responded on endpoint 0 with something that is not ASCII")
    printer("JSON: " + json_string)
    try:
        json_data = json.loads(json_string)
    except json.decoder.JSONDecodeError:
        raise odrive.protocol.DeviceInitException("device responded on endpoint 0 with something that is not JSON")
    json_data = {"name": "odrive", "members": json_data}
    return create_object("odrive", json_data, None, channel, printer=printer)

def find_usb_channels(vid_pid_pairs=odrive.util.USB_VID_PID_PAIRS, printer=noprint):
    """
    Scans for compatible USB devices.
    Returns a generator of odrive.protocol.Channel objects.
    """
    for vid_pid_pair in vid_pid_pairs:
        for usb_device in usb.core.find(idVendor=vid_pid_pair[0], idProduct=vid_pid_pair[1], find_all=True):
            printer("Found ODrive via PyUSB")
            try:
                yield channel_from_usb_device(usb_device, printer)
            except usb.core.USBError as ex:
                if ex.errno == 13:
                    printer("USB device access denied. Did you set up your udev rules correctly?")
                    continue
                raise

def find_dev_serial_ports(search_regex):
    try:
        return ['/dev/' + x for x in filter(re.compile(search_regex).search, os.listdir('/dev'))]
    except FileNotFoundError:
        return []

def find_pyserial_ports():
    return [x.name for x in serial.tools.list_ports.comports()]

def find_serial_channels(printer=noprint):
    """
    Scans for serial ports.
    Returns a generator of odrive.protocol.Channel objects.
    Not every returned object necessarily represents a compatible device.
    """

    # Real serial ports or USB-Serial converters (tested on Linux and Windows)
    real_serial_ports = find_pyserial_ports()

    # Serial devices that are exposed by the platform
    # for the device's USB connection
    linux_usb_serial_ports = find_dev_serial_ports(r'^ttyACM')
    macos_usb_serial_ports = find_dev_serial_ports(r'^tty\.usbmodem')

    for port in real_serial_ports + linux_usb_serial_ports + macos_usb_serial_ports:
        try:
            yield channel_from_serial_port(port, 115200, False, printer)
        except serial.serialutil.SerialException:
            printer("could not open " + port)
            continue


def find_all(consider_usb=True, consider_serial=False, printer=noprint):
    """
    Returns a generator with all the connected devices that speak the ODrive protocol
    """
    channels = iter(())
    if (consider_usb):
        channels = itertools.chain(channels, find_usb_channels(printer=printer))
    if (consider_serial):
        channels = itertools.chain(channels, find_serial_channels(printer=printer))
    for channel in channels:
        # TODO: blacklist known bad channels
        try:
            yield object_from_channel(channel, printer)
        except odrive.protocol.DeviceInitException as ex:
            printer(str(ex))
            continue


def find_any(consider_usb=True, consider_serial=False, printer=noprint):
    """
    Scans for ODrives on all supported interfaces and returns the first device
    that is found. If no device is connected the function blocks.
    """
    # TODO: do device discovery and instantiation in a separate thread and just wait on a semaphore here

    # poll for device
    printer("looking for ODrive...")
    while True:
        dev = next(find_all(consider_usb, consider_serial, printer=printer), None)
        if dev is not None:
            return dev
        printer("no device found")
        time.sleep(1)

def open_serial(port_name, printer=noprint):
    channel = channel_from_serial_port(port_name, 115200, False, printer)
    return object_from_channel(channel, printer)

def open_usb(bus, address, printer=noprint):
    usb_device1 = usb.core.find(bus=1, address=16)
    usb_device = usb.core.find(bus=bus, address=address)
    if usb_device is None:
        raise odrive.protocol.DeviceInitException("No USB device found on bus {} device {}".format(bus, address))
    channel = channel_from_usb_device(usb_device, printer)
    return object_from_channel(channel, printer)
