"""
Provides functions for the discovery of fibre hubs
"""

import sys
import time
import json
#import usb.core
#import usb.util
#import serial
#import serial.tools.list_ports
import re
import time
import os
import itertools
import struct
import functools

import fibre.protocol
# TODO: refactor code for each transport layer
import fibre.udp_transport
import fibre.tcp_transport
try:
    import fibre.usbbulk_transport
except ModuleNotFoundError:
    def find_usb_channels():
        return []
try:
    import fibre.serial_transport
except ModuleNotFoundError:
    def find_serial_channels():
        return []

def noprint(x):
  pass

def find_all(consider_usb=True, consider_serial=False, printer=noprint, device_stdout=noprint):
    """
    Returns a generator with all the connected devices that speak the Fibre protocol
    """
    channels = iter(())
    if (consider_usb):
        channels = itertools.chain(channels, find_usb_channels(printer=printer, device_stdout=device_stdout))
    if (consider_serial):
        channels = itertools.chain(channels, find_serial_channels(printer=printer, device_stdout=device_stdout))
    for channel in channels:
        # TODO: blacklist known bad channels
        try:
            yield object_from_channel(channel, printer)
        except fibre.protocol.DeviceInitException as ex:
            printer(str(ex))
            continue


def find_any(consider_usb=True, consider_serial=False, printer=noprint, device_stdout=noprint):
    """
    Scans for Fibre Hubs on all supported interfaces and returns the first device
    that is found. If no device is connected the function blocks.
    """
    # TODO: do device discovery and instantiation in a separate thread and just wait on a semaphore here

    # poll for device
    printer("looking for Fibre Hubs...")
    while True:
        dev = next(find_all(consider_usb, consider_serial, printer=printer, device_stdout=device_stdout), None)
        if dev is not None:
            return dev
        printer("no device found")
        time.sleep(1)
