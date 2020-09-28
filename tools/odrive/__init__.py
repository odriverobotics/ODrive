
import os
import sys

# We want to use the fibre package that is included with the odrive package
# in order to avoid any version mismatch issues,
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), "pyfibre"))

import fibre

# Standard convention is to add a __version__ attribute to the package
from .version import get_version_str
__version__ = get_version_str()
del get_version_str

from .utils import get_serial_number_str

default_search_path = 'usb:idVendor=0x1209,idProduct=0x0D32,bInterfaceClass=0,bInterfaceSubClass=1,bInterfaceProtocol=0'

def find_any(path=default_search_path, serial_number=None,
        search_cancellation_token=None, channel_termination_token=None,
        timeout=None, logger=fibre.Logger(verbose=False), find_multiple=False):
    """
    Blocks until the first matching ODrive object is connected and then returns that object
    """
    result = []
    done_signal = fibre.Event(search_cancellation_token)
    def did_discover_object(obj):
        result.append(obj)
        if find_multiple:
            if len(result) >= int(find_multiple):
               done_signal.set()
        else:
            done_signal.set()

    async def obj_filter(obj):
        return (serial_number is None or
                (await get_serial_number_str(obj)) == serial_number)

    fibre.start_discovery(path, obj_filter, did_discover_object,
            done_signal, channel_termination_token, logger)

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
        return result[0] if len(result) > 0 else None
