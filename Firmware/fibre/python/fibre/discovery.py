"""
Provides functions for the discovery of Fibre nodes
"""

import sys
import json
import time
import threading
import traceback
import struct
import fibre.protocol
import fibre.utils
import fibre.remote_object
from fibre.utils import Event, Logger
from fibre.protocol import ChannelBrokenException, TimeoutError
import appdirs
import os

# Load all installed transport layers

channel_types = {}

try:
    import fibre.usbbulk_transport
    channel_types['usb'] = fibre.usbbulk_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.serial_transport
    channel_types['serial'] = fibre.serial_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.tcp_transport
    channel_types['tcp'] = fibre.tcp_transport.discover_channels
except ImportError:
    pass

try:
    import fibre.udp_transport
    channel_types['udp'] = fibre.udp_transport.discover_channels
except ImportError:
    pass

def noprint(text):
    pass

def find_all(path, serial_number,
         did_discover_object_callback,
         search_cancellation_token,
         channel_termination_token,
         logger):
    """
    Starts scanning for Fibre nodes that match the specified path spec and calls
    the callback for each Fibre node that is found.
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
            logger.debug("Connecting to device on " + channel._name)

            cache_dir = appdirs.user_cache_dir("odrivetool")
            cache_path = None

            # Fetch the json version tag to check cache (only supported on firmware v0.5 or later)
            try:
                json_version_tag = channel.remote_endpoint_operation(0, struct.pack("<I", 0xffffffff), True, 4)
                json_version_tag = struct.unpack("<I", json_version_tag)[0]

                logger.debug("Device reported JSON version ID: {:08d}".format(json_version_tag))
                cache_path = os.path.join(cache_dir, 'fibre_schema_cache_{:08d}'.format(json_version_tag))
            except:
                logger.debug("Failed to get JSON checksum")

            # Check cache
            json_data = None
            try:
                if not cache_path is None:
                    with open(cache_path, 'rb') as fp:
                        json_crc16 = fibre.protocol.calc_crc16(fibre.protocol.PROTOCOL_VERSION, fp.read())
                        fp.seek(0)
                        json_data = json.load(fp)
            except:
                logger.debug("Failed load JSON cache file {}".format(cache_path))

            # Fallback to loading JSON from device
            if json_data is None:
                # Downloading json data
                logger.info("Downloading json data from ODrive... (this might take a while)")
                json_bytes = channel.remote_endpoint_read_buffer(0)
                try:
                    json_string = json_bytes.decode("ascii")
                except UnicodeDecodeError:
                    logger.debug("Device responded on endpoint 0 with something that is not ASCII")
                    raise UnicodeDecodeError

                json_crc16 = fibre.protocol.calc_crc16(fibre.protocol.PROTOCOL_VERSION, json_bytes)
                json_data = json.loads(json_string)

                # Save JSON to cache
                if not cache_path is None:
                    logger.debug("Creating new JSON cache file {}".format(cache_path))
                    os.makedirs(cache_dir, exist_ok=True)
                    with open(cache_path, 'w+') as json_cache:
                        json_cache.write(json_string)
                    logger.debug("Saved JSON to cache file {}".format(cache_path))

            channel._interface_definition_crc = json_crc16

            logger.debug("JSON: " + str(json_data).replace("{'name'", "\n{'name'"))

            json_data = {"name": "fibre_node", "members": json_data}
            obj = fibre.remote_object.RemoteObject(json_data, None, channel, logger)

            obj.__dict__['_json_data'] = json_data['members']
            obj.__dict__['_json_crc'] = json_crc16

            device_serial_number = fibre.utils.get_serial_number_str(obj)
            if serial_number != None and device_serial_number != serial_number:
                logger.debug("Ignoring device with serial number {}".format(device_serial_number))
                return
            
            did_discover_object_callback(obj)


        except Exception:
            logger.debug("Unexpected exception after discovering channel: " + traceback.format_exc())

    # For each connection type, kick off an appropriate discovery loop
    for search_spec in path.split(','):
        prefix = search_spec.split(':')[0]
        the_rest = ':'.join(search_spec.split(':')[1:])
        if prefix in channel_types:
            t = threading.Thread(target=channel_types[prefix],
                             args=(the_rest, serial_number, did_discover_channel, search_cancellation_token, channel_termination_token, logger))
            t.daemon = True
            t.start()
        else:
            raise Exception("Invalid path spec \"{}\"".format(search_spec))


def find_any(path="usb", serial_number=None,
        search_cancellation_token=None, channel_termination_token=None,
        timeout=None, logger=Logger(verbose=False), find_multiple=False):
    """
    Blocks until the first matching Fibre node is connected and then returns that node
    """
    result = []
    done_signal = Event(search_cancellation_token)
    def did_discover_object(obj):
        result.append(obj)
        if find_multiple:
            if len(result) >= int(find_multiple):
               done_signal.set()
        else:
            done_signal.set()

    find_all(path, serial_number, did_discover_object, done_signal, channel_termination_token, logger)
    try:
        done_signal.wait(timeout=timeout)
    except TimeoutError:
        if not find_multiple:
            return None
    finally:
        done_signal.set() # terminate find_all

    if find_multiple:
        return result
    else:
        return result[0]
