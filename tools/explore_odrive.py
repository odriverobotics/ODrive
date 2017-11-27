#!/usr/bin/env python3
"""
Load an odrive object to play with in the IPython interactive shell.
"""

import odrive.core
import argparse
import sys


# Parse arguments
parser = argparse.ArgumentParser(description='Load an odrive object to play with in the IPython interactive shell.')
parser.add_argument("-v", "--verbose", action="store_true",
                    help="print debug information")
group = parser.add_mutually_exclusive_group()
group.add_argument("-d", "--discover", metavar="CHANNELS", action="store",
                    help="Automatically discover ODrives. Takes a comma-separated list (without spaces) "
                    "to indicate which connection types should be considered. Possible values are "
                    "usb and serial. For example \"--discover usb,serial\" indicates "
                    "that USB and serial ports should be scanned for ODrives. "
                    "If none of the below options are specified, --discover usb is assumed.")
group.add_argument("-u", "--usb", metavar="BUS:DEVICE", action="store",
                    help="Specifies the USB port on which the device is connected. "
                    "For example \"001:014\" means bus 001, device 014. The numbers can be obtained "
                    "using `lsusb`.")
group.add_argument("-s", "--serial", metavar="PORT", action="store",
                    help="Specifies the serial port on which the device is connected. "
                    "For example \"/dev/ttyUSB0\". Use `ls /dev/tty*` to find your port name.")
parser.set_defaults(discover="usb")
args = parser.parse_args()

if (args.verbose):
  printer = print
else:
  printer = lambda x: None


# Connect to device
if not args.usb is None:
  try:
    bus = int(args.usb.split(":")[0])
    address = int(args.usb.split(":")[1])
  except (ValueError, IndexError):
    print("the --usb argument must look something like this: \"001:014\"")
    sys.exit(1)
  try:
    my_odrive = odrive.core.open_usb(bus, address, printer=printer)
  except odrive.protocol.DeviceInitException as ex:
    print(str(ex))
    sys.exit(1)
elif not args.serial is None:
  my_odrive = odrive.core.open_serial(args.serial, printer=printer)
else:
  print("Waiting for device...")
  consider_usb = 'usb' in args.discover.split(',')
  consider_serial = 'serial' in args.discover.split(',')
  my_odrive = odrive.core.find_any(consider_usb, consider_serial, printer=printer)
print("Connected!")


print('')
print('ODRIVE EXPLORER')
print('')
print('You can now type "my_odrive." and press <tab>')
print('This will present you with all the properties that you can reference')
print('')
print('For example: "my_odrive.motor0.encoder.pll_pos"')
print('will print the current encoder position on motor 0')
print('and "my_odrive.motor0.pos_setpoint = 10000"')
print('will send motor0 to 10000')
print('')

try:
  # If this assignment works, we are already in interactive mode.
  # so just drop out of script to existing shell
  interpreter = sys.ps1
except AttributeError:
  # We are not in interactive mode, so let's fire one up
  # Though let's be real, IPython is the way to go
  print('If you want to have an improved interactive console with pretty colors,')
  print('you can run this script in interactive mode with IPython with this command:')
  print('ipython -i explore_odrive.py')
  print('')
  # Enter interactive python shell with tab complete enabled
  import code
  import rlcompleter
  import readline
  readline.parse_and_bind("tab: complete")
  code.interact(local=locals(), banner='')
