#!/usr/bin/env python3
"""
Liveplotter
"""

import sys
import time
import threading
import platform
import subprocess
import os

try:
    if platform.system() == 'Windows':
        import win32console
        import colorama
        colorama.init()
except ModuleNotFoundError:
    print("Could not init terminal colors")
    pass

data_rate = 100
plot_rate = 10
num_samples = 1000

def start_liveplotter(get_var_callback):
    """
    Starts a liveplotter.
    The variable that is plotted is retrieved from get_var_callback.
    This function returns immediately and the liveplotter quits when
    the user closes it.
    """

    import matplotlib.pyplot as plt

    cancellation_token = threading.Event()

    global vals
    vals = []
    def fetch_data():
        global vals
        while not cancellation_token.is_set():
            try:
                data = get_var_callback()
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            vals.append(data)
            if len(vals) > num_samples:
                vals = vals[-num_samples:]
            time.sleep(1/data_rate)

    # TODO: use animation for better UI performance, see:
    # https://matplotlib.org/examples/animation/simple_anim.html
    def plot_data():
        global vals

        plt.ion()

        # Make sure the script terminates when the user closes the plotter
        def did_close(evt):
            cancellation_token.set()
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', did_close)

        while not cancellation_token.is_set():
            plt.clf()
            plt.plot(vals)
            #time.sleep(1/plot_rate)
            fig.canvas.flush_events()

    threading.Thread(target=fetch_data).start()
    threading.Thread(target=plot_data).start()
    #plot_data()

def print_drv_regs(name, motor):
    """
    Dumps the current gate driver regisers for the specified motor
    """
    fault = motor.gate_driver.drv_fault
    status_reg_1 = motor.gate_driver.status_reg_1
    status_reg_2 = motor.gate_driver.status_reg_2
    ctrl_reg_1 = motor.gate_driver.ctrl_reg_1
    ctrl_reg_2 = motor.gate_driver.ctrl_reg_2
    print(name + ": " + str(fault))
    print("DRV Fault Code: " + str(fault))
    print("Status Reg 1: " + str(status_reg_1) + " (" + format(status_reg_1, '#010b') + ")")
    print("Status Reg 2: " + str(status_reg_2) + " (" + format(status_reg_2, '#010b') + ")")
    print("Control Reg 1: " + str(ctrl_reg_1) + " (" + format(ctrl_reg_1, '#013b') + ")")
    print("Control Reg 2: " + str(ctrl_reg_2) + " (" + format(ctrl_reg_2, '#09b') + ")")

def rate_test(device):
    """
    Tests how many integers per second can be transmitted
    """

    import matplotlib.pyplot as plt
    plt.ion()

    print("reading 10000 values...")
    numFrames = 10000
    vals = []
    for _ in range(numFrames):
        vals.append(device.motor0.loop_counter)

    plt.plot(vals)

    loopsPerFrame = (vals[-1] - vals[0])/numFrames
    loopsPerSec = (168000000/(2*10192))
    FramePerSec = loopsPerSec/loopsPerFrame
    print("Frames per second: " + str(FramePerSec))

def setup_udev_rules(logger):
    if platform.system() != 'Linux':
        logger.error("This command only makes sense on Linux")
    if os.getuid() != 0:
        logger.warn("you should run this as root, otherwise it will probably not work")
    with open('/etc/udev/rules.d/50-odrive.rules', 'w') as file:
        file.write('SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d3[0-9]", MODE="0666"\n')
    subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
    subprocess.run(["udevadm", "trigger"], check=True)
    logger.info('udev rules configured successfully')


## Exceptions ##

class TimeoutException(Exception):
    pass

## Threading utils ##

class Event():
    """
    Alternative to threading.Event(), enhanced by the subscribe() function
    that the original fails to provide.
    @param Trigger: if supplied, the newly created event will be triggered
                    as soon as the trigger event becomes set
    """
    def __init__(self, trigger=None):
        self._evt = threading.Event()
        self._subscribers = []
        self._mutex = threading.Lock()
        if not trigger is None:
            trigger.subscribe(self.set())

    def is_set(self):
        return self._evt.is_set()

    def set(self):
        """
        Sets the event and invokes all subscribers if the event was
        not already set
        """
        self._mutex.acquire()
        try:
            if not self._evt.is_set():
                self._evt.set()
                for s in self._subscribers:
                    s()
        finally:
            self._mutex.release()

    def subscribe(self, handler):
        """
        Invokes the specified handler exactly once as soon as the
        specified event is set. If the event is already set, the
        handler is invoked immediately.
        Returns a function that can be invoked to unsubscribe.
        """
        self._mutex.acquire()
        try:
            self._subscribers.append(handler)
            if self._evt.is_set():
                handler()
        finally:
            self._mutex.release()
        return lambda: self.unsubscribe(handler)
    
    def unsubscribe(self, handler):
        self._mutex.acquire()
        try:
            self._subscribers.pop(self._subscribers.index(handler))
        finally:
            self._mutex.release()

    def wait(self, timeout=None):
        if not self._evt.wait(timeout=timeout):
            raise TimeoutError()

    def trigger_after(self, timeout):
        """
        Triggers the event after the specified timeout.
        This function returns immediately.
        """
        def delayed_trigger():
            if not self.wait(timeout=timeout):
                self.set()
        threading.Thread(target=delayed_trigger, daemon=True).start()

def wait_any(*events, timeout=None):
    """
    Blocks until any of the specified events are triggered.
    Returns the number of the event that was triggerd or raises
    a TimeoutException
    """
    or_event = threading.Event()
    unsubscribe_functions = []
    for event in events:
        unsubscribe_functions.append(event.subscribe(lambda: or_event.set()))
    or_event.wait(timeout=timeout)
    for unsubscribe_function in unsubscribe_functions:
        unsubscribe_function()
    for i in range(len(events)):
        if events[i].is_set():
            return i
    raise TimeoutException()

class Logger():
    """
    Logs messages to stdout
    """

    COLOR_DEFAULT = 0
    COLOR_GREEN = 1
    COLOR_CYAN = 2
    COLOR_YELLOW = 3
    COLOR_RED = 4

    _VT100Colors = {
        COLOR_GREEN: '\x1b[92;1m',
        COLOR_CYAN: '\x1b[96;1m',
        COLOR_YELLOW: '\x1b[93;1m',
        COLOR_RED: '\x1b[91;1m',
        COLOR_DEFAULT: '\x1b[0m'
    }

    _Win32Colors = {
        COLOR_GREEN: 0x0A,
        COLOR_CYAN: 0x0B,
        COLOR_YELLOW: 0x0E,
        COLOR_RED: 0x0C,
        COLOR_DEFAULT: 0x07
    }

    def __init__(self, verbose=True):
        self._prefix = ''
        self._skip_bottom_line = False # If true, messages are printed one line above the cursor
        self._verbose = verbose
        if platform.system() == 'Windows':
            self._stdout_buf = win32console.GetStdHandle(win32console.STD_OUTPUT_HANDLE)

    def indent(self, prefix='  '):
        indented_logger = Logger()
        indented_logger._prefix = self._prefix + prefix
        return indented_logger

    def print_on_second_last_line(self, text, color):
        """
        Prints a text on the second last line.
        This can be used to print a message above the command
        prompt. If the command prompt spans multiple lines
        there will be glitches.
        If the printed text spans multiple lines there will also
        be glitches (though this could be fixed).
        """

        if platform.system() == 'Windows':
            # Windows <10 doesn't understand VT100 escape codes and the colorama
            # also doesn't support the specific escape codes we need so we use the
            # native Win32 API.
            info = self._stdout_buf.GetConsoleScreenBufferInfo()
            cursor_pos = info['CursorPosition']
            scroll_rect=win32console.PySMALL_RECTType(
                Left=0, Top=1,
                Right=info['Window'].Right,
                Bottom=cursor_pos.Y-1)
            scroll_dest = win32console.PyCOORDType(scroll_rect.Left, scroll_rect.Top-1)
            self._stdout_buf.ScrollConsoleScreenBuffer(
                scroll_rect, scroll_rect, scroll_dest, # clipping rect is same as scroll rect
                u' ', Logger._Win32Colors[color]) # fill with empty cells with the desired color attributes
            line_start = win32console.PyCOORDType(0, cursor_pos.Y-1)
            self._stdout_buf.WriteConsoleOutputCharacter(text, line_start)

        else:
            # Assume we're in a terminal that interprets VT100 escape codes.
            # TODO: test on macOS

            # Escape character sequence:
            #   ESC 7: store cursor position
            #   ESC 1A: move cursor up by one
            #   ESC 1S: scroll entire viewport by one
            #   ESC 1L: insert 1 line at cursor position
            #   (print text)
            #   ESC 8: restore old cursor position

            sys.stdout.write('\x1b7\x1b[1A\x1b[1S\x1b[1L')
            sys.stdout.write(Logger._VT100Colors[color] + text + Logger._VT100Colors[Logger.COLOR_DEFAULT])
            sys.stdout.write('\x1b8')
            sys.stdout.flush()

    def print_colored(self, text, color):
        if self._skip_bottom_line:
            self.print_on_second_last_line(text, color)
        else:
            # On Windows, colorama does the job of interpreting the VT100 escape sequences
            sys.stdout.write(Logger._VT100Colors[color] + text + Logger._VT100Colors[Logger.COLOR_DEFAULT] + '\n')
            sys.stdout.flush()

    def debug(self, text):
        if self._verbose:
            self.print_colored(self._prefix + text, Logger.COLOR_DEFAULT)
    def success(self, text):
        self.print_colored(self._prefix + text, Logger.COLOR_GREEN)
    def info(self, text):
        self.print_colored(self._prefix + text, Logger.COLOR_CYAN)
    def warn(self, text):
        self.print_colored(self._prefix + text, Logger.COLOR_YELLOW)
    def error(self, text):
        # TODO: write to stderr
        self.print_colored(self._prefix + text, Logger.COLOR_RED)
