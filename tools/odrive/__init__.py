
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

from .utils import get_serial_number_str, get_serial_number_str_sync
import threading

default_search_path = 'usb:idVendor=0x1209,idProduct=0x0D32,bInterfaceClass=0,bInterfaceSubClass=1,bInterfaceProtocol=0'

def find_any(path=default_search_path, serial_number=None,
        search_cancellation_token=None, channel_termination_token=None,
        timeout=None, logger=fibre.Logger(verbose=False)):
    """
    Blocks until the first matching ODrive object is connected and then returns that object
    """

    result = []

    done_signal = fibre.Event(search_cancellation_token)
    channel_termination_token = fibre.Event(channel_termination_token)

    async def discovered_object(obj):
        if not (serial_number is None) and ((await get_serial_number_str(obj)) != serial_number):
            return # ignore this device

        obj._on_lost.add_done_callback(lambda x: channel_termination_token.set())
        result.append(obj)
        done_signal.set()

    def domain_thread():
        with fibre.Domain(path) as domain:
            discovery = domain.run_discovery(discovered_object)
            channel_termination_token.wait()
            discovery.stop()
    
    threading.Thread(target=domain_thread).start()

    try:
        done_signal.wait(timeout=timeout)
    except:
        channel_termination_token.set()
        raise

    return result[0]
