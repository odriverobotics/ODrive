
import json
import os
import tempfile
import fibre.libfibre
import odrive
from odrive.utils import OperationAbortedException, yes_no_prompt

def obj_to_path(root, obj):
    for k in dir(root):
        v = getattr(root, k)
        if not k.startswith('_') and isinstance(v, fibre.libfibre.RemoteObject):
            if v == obj:
                return k
            subpath = obj_to_path(v, obj)
            if not subpath is None:
                return k + "." + subpath
    return None

def get_dict(root, obj, is_config_object):
    result = {}

    for k in dir(obj):
        v = getattr(obj, k)
        if k.startswith('_') and k.endswith('_property') and is_config_object:
            v = v.read()
            if isinstance(v, fibre.libfibre.RemoteObject):
                v = obj_to_path(root, v)
            result[k[1:-9]] = v
        elif not k.startswith('_') and isinstance(v, fibre.libfibre.RemoteObject):
            sub_dict = get_dict(root, v, (k == 'config') or is_config_object)
            if sub_dict != {}:
                result[k] = sub_dict

    return result

def set_dict(obj, path, config_dict):
    errors = []
    for (k,v) in config_dict.items():
        name = path + ("." if path != "" else "") + k
        if not k in dir(obj):
            errors.append("Could not restore {}: property not found on device".format(name))
            continue
        if isinstance(v, dict):
            errors += set_dict(getattr(obj, k), name, v)
        else:
            try:
                remote_attribute = getattr(obj, '_' + k + '_property')
                #if isinstance(v, str) and isinstance()
                remote_attribute.exchange(v)
            except Exception as ex:
                errors.append("Could not restore {}: {}".format(name, str(ex)))
    return errors

def get_temp_config_filename(device):
    serial_number = odrive.get_serial_number_str_sync(device)
    safe_serial_number = ''.join(filter(str.isalnum, serial_number))
    return os.path.join(tempfile.gettempdir(), 'odrive-config-{}.json'.format(safe_serial_number))

def backup_config(device, filename, logger):
    """
    Exports the configuration of an ODrive to a JSON file.
    If no file name is provided, the file is placed into a
    temporary directory.
    """

    if filename is None:
        filename = get_temp_config_filename(device)

    logger.info("Saving configuration to {}...".format(filename))

    if os.path.exists(filename):
        if not yes_no_prompt("The file {} already exists. Do you want to override it?".format(filename), True):
            raise OperationAbortedException()

    data = get_dict(device, device, False)
    with open(filename, 'w') as file:
        json.dump(data, file)
    logger.info("Configuration saved.")

def restore_config(device, filename, logger):
    """
    Restores the configuration stored in a file 
    """

    if filename is None:
        filename = get_temp_config_filename(device)

    with open(filename) as file:
        data = json.load(file)

    logger.info("Restoring configuration from {}...".format(filename))
    errors = set_dict(device, "", data)

    for error in errors:
        logger.info(error)
    if errors:
        logger.warn("Some of the configuration could not be restored.")
    
    try:
        device.save_configuration()
    except fibre.libfibre.ObjectLostError:
        pass # Saving configuration makes the device reboot
    logger.info("Configuration restored.")
