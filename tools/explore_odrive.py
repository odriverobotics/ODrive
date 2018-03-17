#!/usr/bin/env python3
"""
Load an odrive object to play with in the IPython interactive shell.
"""

import argparse
import sys
import platform
import threading
import odrive.discovery
from odrive.utils import start_liveplotter

# Flush stdout by default
import functools
print = functools.partial(print, flush=True)


# some enums described in the README
# TODO: transmit as part of the JSON
MOTOR_TYPE_HIGH_CURRENT = 0
#MOTOR_TYPE_LOW_CURRENT = 1
MOTOR_TYPE_GIMBAL = 2

CTRL_MODE_VOLTAGE_CONTROL = 0,
CTRL_MODE_CURRENT_CONTROL = 1,
CTRL_MODE_VELOCITY_CONTROL = 2,
CTRL_MODE_POSITION_CONTROL = 3


## Parse arguments ##
parser = argparse.ArgumentParser(description='Load an odrive object to play with in the IPython interactive shell.',
                                 formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument("-v", "--verbose", action="store_true",
                    help="print debug information")
parser.add_argument("-p", "--path", metavar="PATH", action="store",
                    help="The path(s) where ODrive(s) should be discovered.\n"
                    "By default the script will connect to any ODrive on USB.\n\n"
                    "To select a specific USB device:\n"
                    "  --path usb:BUS:DEVICE\n"
                    "usbwhere BUS and DEVICE are the bus and device numbers as shown in `lsusb`.\n\n"
                    "To select a specific serial port:\n"
                    "  --path serial:PATH\n"
                    "where PATH is the path of the serial port. For example \"/dev/ttyUSB0\".\n"
                    "You can use `ls /dev/tty*` to find the correct port.\n\n"
                    "You can combine USB and serial specs by separating them with a comma (no space!)\n"
                    "Example:\n"
                    "  --path usb,serial:/dev/ttyUSB0\n"
                    "means \"discover any USB device or a serial device on /dev/ttyUSB0\"")
parser.add_argument("--no-ipython", action="store_true",
                    help="Use the regular Python shell\n"
                    "instead of the IPython shell,\n"
                    "even if IPython is installed\n")
parser.add_argument("-s", "--serial-number", action="store",
                    help="The serial number of the device. If omitted, any device is accepted.\n")
parser.set_defaults(path="usb")
args = parser.parse_args()

if (args.verbose):
  printer = print
else:
  printer = lambda x: None


## Interactive console utils ##

COLOR_RED = '\x1b[91;1m'
COLOR_CYAN = '\x1b[96;1m'
COLOR_RESET = '\x1b[0m'

def print_on_second_last_line(text, **kwargs):
    """
    Prints a text on the second last line.
    This can be used to print a message above the command
    prompt. If the command prompt spans multiple lines,
    there will be glitches.
    """
    # Escape character sequence:
    #   ESC 7: store cursor position
    #   ESC 1A: move cursor up by one
    #   ESC 1S: scroll entire viewport by one
    #   ESC 1L: insert 1 line at cursor position
    #   (print text)
    #   ESC 8: restore old cursor position
    kwargs['end'] = ''
    kwargs['flush'] = True
    print('\x1b7\x1b[1A\x1b[1S\x1b[1L' + text + '\x1b8', **kwargs)

def print_banner():
    print('ODrive control utility v0.4')
    print('Please connect your ODrive.')
    print('Type help() for help.')

def print_help():
    print('')
    if len(discovered_devices) == 0:
        print('Connect your ODrive to {} and power it up.'.format(args.path))
        print('After that, the following message should appear:')
        print('  "Connected to ODrive [serial number] as odrv0"')
        print('')
        print('Once the ODrive is connected, type "odrv0." and press <tab>')
    else:
        print('Type "odrv0." and press <tab>')
    print('This will present you with all the properties that you can reference')
    print('')
    print('For example: "odrv0.motor0.encoder.pll_pos"')
    print('will print the current encoder position on motor 0')
    print('and "odrv0.motor0.pos_setpoint = 10000"')
    print('will send motor0 to 10000')
    print('')

interactive_variables = {}
interactive_variables["help"] = print_help


## Device discovery ##

discovered_devices = []

def did_discover_device(odrive):
    """
    Handles the discovery of new devices by displaying a
    message and making the device available to the interactive
    console
    """
    serial_number = odrive.serial_number if hasattr(odrive, 'serial_number') else "[unknown serial number]"
    if serial_number in discovered_devices:
        verb = "Reconnected"
        index = discovered_devices.index(serial_number)
    else:
        verb = "Connected"
        discovered_devices.append(serial_number)
        index = len(discovered_devices) - 1
    interactive_name = "odrv" + str(index)

    # Subscribe to disappearance of the device
    odrive.__dict__["__sealed__"] = False
    odrive._did_disappear_callback = lambda: did_lose_device(interactive_name)
    odrive.__sealed__ = True

    # Publish new ODrive to interactive console
    interactive_variables[interactive_name] = odrive
    globals()[interactive_name] = odrive # Add to globals so tab complete works
    print_on_second_last_line(COLOR_CYAN + "{} to ODrive {:012X} as {}".format(verb, serial_number, interactive_name) + COLOR_RESET)

def did_lose_device(interactive_name):
    """
    Handles the disappearance of a device by displaying
    a message.
    """
    print_on_second_last_line(COLOR_RED + "Oh no {} disappeared".format(interactive_name) + COLOR_RESET)

# Connect to device
printer("Waiting for device...")
app_shutdown_token = threading.Event()
odrive.discovery.find_all(args.path, args.serial_number,
                 did_discover_device, app_shutdown_token,
                 printer=printer)


## Launch interactive shell ##

# Check if IPython is installed
if args.no_ipython:
    use_ipython = False
else:
    try:
        import IPython
        use_ipython = True
    except:
        print("Warning: you don't have IPython installed.")
        print("If you want to have an improved interactive console with pretty colors,")
        print("you should install IPython\n")
        use_ipython = False

# If IPython is installed, embed IPython shell, otherwise embed regular shell
if use_ipython:
    #interactive_variables["lalala"] = print_help
    help = print_help # Override help function
    console = IPython.terminal.embed.InteractiveShellEmbed(local_ns=interactive_variables, banner1='')
    console.runcode = console.run_code # hack to make IPython look like the regular console
    interact = console
else:

    # Enable tab complete if possible
    try:
        import rlcompleter
        import readline
        readline.parse_and_bind("tab: complete")
    except:
        sudo_prefix = "" if platform.system() == "Windows" else "sudo "
        print("Warning: could not enable tab-complete. User experience will suffer.\n"
            "Run `{}pip install readline` and then restart this script to fix this."
            .format(sudo_prefix))

    import code
    console = code.InteractiveConsole(locals=interactive_variables)
    interact = lambda: console.interact(banner='')

# install hook to hide ChannelBrokenException
console.runcode('import sys')
console.runcode('superexcepthook = sys.excepthook')
console.runcode('def newexcepthook(ex_class,ex,trace):\n'
                '  if ex_class.__module__ + "." + ex_class.__name__ != "odrive.protocol.ChannelBrokenException":\n'
                '    superexcepthook(ex_class,ex,trace)')
console.runcode('sys.excepthook=newexcepthook')


# Launch shell
print_banner()
interact()
app_shutdown_token.set()
