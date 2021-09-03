
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
import time

default_usb_search_path = 'usb:idVendor=0x1209,idProduct=0x0D32,bInterfaceClass=0,bInterfaceSubClass=1,bInterfaceProtocol=0'
default_search_path = default_usb_search_path


_discovery_lock = threading.Lock()
_discovery_started = [False]
_discovery_path = [None]
_discovery_signal = threading.Condition()
_objects = []


def _start_discovery(path):
    _domain_termination_token = fibre.Event()

    async def discovered_object(obj):
        def lost_object(_):
            idx = [i for i, (o, _) in enumerate(_objects) if o == obj][0]
            _objects.pop(idx)

        _objects.append((obj, await get_serial_number_str(obj)))

        obj._on_lost.add_done_callback(lost_object)
        with _discovery_signal:
            _discovery_signal.notify_all()

    def domain_thread():
        with fibre.Domain(path) as domain:
            discovery = domain.run_discovery(discovered_object)
            _domain_termination_token.wait()
            discovery.stop()

    threading.Thread(target=domain_thread, daemon=True).start()


def find_any(path=default_search_path, serial_number=None, cancellation_token=None, timeout=None):
    """
    Blocks until the first matching ODrive object is connected and then returns
    that object.

    If find_any() is called multiple times, the same object may be returned (
    depending on the serial_number argument).

    The first call to find_any() will start a background thread that handles
    the backend. This background thread will keep running until the program is
    terminated.
    
    If you want finer grained control over object discovery
    consider using fibre.Domain directly.
    """
    assert(cancellation_token is None or isinstance(cancellation_token, fibre.Event))

    # Start backend if it's not already started
    with _discovery_lock:
        if not _discovery_started[0]:
            _start_discovery(path)
            _discovery_started[0] = True
            _discovery_path[0] = path
        elif path != _discovery_path[0]:
            raise Exception("Cannot change discovery path between multiple find_any() "
                            "calls: {} != {}. Use fibre.Domain() directly for finer "
                            "grained discovery control.".format(path, _discovery_path))

    cancelled = [False]

    def cancel():
        with _discovery_signal:
            cancelled[0] = True
            _discovery_signal.notify_all()

    try:
        if cancellation_token:
            cancellation_token.subscribe(cancel)

        wait_start = time.monotonic()
        with _discovery_signal:
            while True:
                # If the ODrive was already found, return it now
                for (obj, s) in _objects:
                    if (serial_number is None) or (serial_number == s):
                        return obj
                
                current_timeout = None if timeout is None else min(0, timeout - (time.monotonic() - wait_start))
                _discovery_signal.wait(current_timeout)

                # TODO: it would be more sensible to raise an exception here but
                # DFU implementation assumes that None is returned on cancellation.
                if cancelled[0]:
                    return None

    finally:
        if cancellation_token:
            cancellation_token.unsubscribe(cancel)

