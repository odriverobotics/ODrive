"""
Provides functions for the discovery of ODrive devices
"""

import sys
import json
import odrive
import odrive.mock_device

class DeviceProperty(property):
    def __init__(self, device, id, type, can_read, can_write):
        self._device = device
        self._id = id
        self._type = type
        property.__init__(self,
            self.fget if can_read else None,
            self.fset if can_write else None)

    def fget(self, obj):
        self._device.send("r " + str(self._id) + "\n")
        # TODO: message based receive
        response = self._device.receive_until('\n')
        return self._type(response.strip('\n'))

    def fset(self, obj, value):
        if not isinstance(value, self._type):
            raise TypeError("expected value of type {}".format(self._type.__name__))
        self._device.send("w " + str(self._id) + " " + str(value) + "\n")


def create_object(json_data, namespace, device):
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
            properties[name] = create_object(item["content"], namespace + "." + item["name"], device)
        else:
            if type_str == "float":
                property_type = float
            elif type_str == "int":
                property_type = int
            elif type_str == "bool":
                property_type = bool
            elif type_str == "uint16":
                property_type = int
            else:
                sys.stderr.write("property {} has unsupported type {}".format(name, type_str))
                continue

            id_str = item.get("id", None)
            if id_str is None:
                sys.stderr.write("property {} has specified ID".format(name))
                continue

            access_mode = item.get("mode", "rw")
            properties[name] = DeviceProperty(device, id_str, property_type,
                                              'r' in access_mode,
                                              'w' in access_mode)

    # Create a type from the property list and instantiate it
    jit_type = type(namespace, (object,), properties)
    new_object = jit_type()
    return new_object


def find_any():
    """
    Scans for ODrives on all supported interfaces (currently only USB) and
    returns the first device that is found. If no device is connected the
    function blocks.
    """
    # TODO: do device discovery and instantiation in a separate thread
    # TODO: test with real device
    device = odrive.mock_device.MockDevice()
    #device = odrive.usbbulk.poll_odrive_bulk_device()
    device.send('j\n')
    json_string = device.receive_until('\n')
    #print("have JSON: " + json_string)
    return create_object(json.loads(json_string), "odrive.usb_device", device)
