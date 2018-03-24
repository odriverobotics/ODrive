"""
Provides functions for the discovery of ODrive devices
"""

import sys
import json
import time
import threading
import traceback
import odrive.protocol
import odrive.utils
import odrive.remote_object
import odrive.usbbulk_transport
import odrive.serial_transport
from odrive.utils import Event

channel_types = {
    "usb": odrive.usbbulk_transport.discover_channels,
    "serial": odrive.serial_transport.discover_channels
}

def noprint(text):
    pass

def find_all(path, serial_number,
         did_discover_object_callback,
         cancellation_token, printer=noprint):
    """
    Starts scanning for ODrives that match the specified path spec and calls
    the callback for each ODrive that is found.
    This function is non-blocking.
    """

    def did_discover_channel(channel):
        """
        Inits an object from a given channel and then calls did_discover_object_callback
        with the created object
        This queries the endpoint 0 on that channel to gain information
        about the interface, which is then used to init the corresponding object.
        """
        try:
            printer("Connecting to device on " + channel._name)
            try:
                json_bytes = channel.remote_endpoint_read_buffer(0)
            except (odrive.utils.TimeoutException, odrive.protocol.ChannelBrokenException):
                printer("no response - probably incompatible")
                return
            json_crc16 = odrive.protocol.calc_crc16(odrive.protocol.PROTOCOL_VERSION, json_bytes)
            channel._interface_definition_crc = json_crc16
            try:
                json_string = json_bytes.decode("ascii")
            except UnicodeDecodeError:
                printer("device responded on endpoint 0 with something that is not ASCII")
                return
            printer("JSON: " + json_string)
            try:
                json_data = json.loads(json_string)
            except json.decoder.JSONDecodeError as error:
                printer("device responded on endpoint 0 with something that is not JSON: " + str(error))
                return
            json_data = {"name": "odrive", "members": json_data}
            obj = odrive.remote_object.RemoteObject(json_data, None, channel, printer)
            device_serial_number = format(obj.serial_number, 'x').upper() if hasattr(obj, 'serial_number') else "[unknown serial number]"
            if serial_number != None and device_serial_number != serial_number:
                printer("Ignoring device with serial number {}".format(device_serial_number))
                return
            did_discover_object_callback(obj)
        except Exception:
            printer("Unexpected exception after discovering channel: " + traceback.format_exc())

    # For each connection type, kick off an appropriate discovery loop
    for search_spec in path.split(','):
        prefix = search_spec.split(':')[0]
        the_rest = ':'.join(search_spec.split(':')[1:])
        if prefix in channel_types:
            threading.Thread(target=channel_types[prefix],
                             args=(the_rest, serial_number, did_discover_channel, cancellation_token, printer)).start()
        else:
            raise Exception("Invalid path spec \"{}\"".format(search_spec))


def find_any(path="usb", serial_number=None, cancellation_token=None, timeout=None, printer=noprint):
    """
    Blocks until the first matching ODrive is connected and then returns that device
    """
    result = [ None ]
    done_signal = Event(cancellation_token)
    def did_discover_object(obj):
        result[0] = obj
        done_signal.set()
    find_all(path, serial_number, did_discover_object, done_signal, printer)
    try:
        done_signal.wait(timeout=timeout)
    finally:
        done_signal.set() # terminate find_all
    return result[0]
