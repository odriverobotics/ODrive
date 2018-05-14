#!/usr/bin/env python3
"""
Connect to a fibre-enabled device to play with in the IPython interactive shell.
"""
import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))) + "/python")
import fibre

# Parse arguments
parser = argparse.ArgumentParser(description='Connect to a fibre-enabled device to play with it in the IPython interactive shell.')
parser.add_argument("-v", "--verbose", action="store_true",
                    help="print debug information")
group = parser.add_mutually_exclusive_group()
group.add_argument("-d", "--discover", metavar="CHANNELS", action="store",
                    help="Automatically discover fibre-enabled devices. Takes a comma-separated list (without spaces) "
                    "to indicate which connection types should be considered. Possible values are "
                    "usb and serial. For example \"--discover usb,serial\" indicates "
                    "that USB and serial ports should be scanned for fibre-enabled devices. "
                    "If none of the below options are specified, --discover usb is assumed.")
group.add_argument("-u", "--usb", metavar="BUS:DEVICE", action="store",
                    help="Specifies the USB port on which the device is connected. "
                    "For example \"001:014\" means bus 001, device 014. The numbers can be obtained "
                    "using `lsusb`.")
group.add_argument("-p", "--udp", metavar="ADDR:PORT", action="store",
                    help="Specifies the UDP port on which the device is reachable. ")
group.add_argument("-t", "--tcp", metavar="ADDR:PORT", action="store",
                    help="Specifies the TCP port on which the device is reachable. ")
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
    my_device = fibre.open_usb(bus, address, printer=printer, device_stdout=print)
  except fibre.protocol.DeviceInitException as ex:
    print(str(ex))
    sys.exit(1)
elif not args.serial is None:
  my_device = fibre.open_serial(args.serial, printer=printer, device_stdout=print)
elif not args.udp is None:
  my_device = fibre.open_udp(args.udp, printer=printer, device_stdout=print)
elif not args.tcp is None:
  my_device = fibre.open_tcp(args.tcp, printer=printer, device_stdout=print)
else:
  print("Waiting for device...")
  consider_usb = 'usb' in args.discover.split(',')
  consider_serial = 'serial' in args.discover.split(',')
  my_device = fibre.find_any(consider_usb, consider_serial, printer=printer, device_stdout=print)
print("Connected!")


try:
  # If this assignment works, we are already in interactive mode.
  # so just drop out of script to existing shell
  interpreter = sys.ps1
except AttributeError:
  # We are not in interactive mode, so let's fire one up
  # Though let's be real, IPython is the way to go
  print('If you want to have an improved interactive console with pretty colors,')
  print('you can run this script in interactive mode with IPython with this command:')
  print('ipython -i explore_fibre.py')
  print('')
  # Enter interactive python shell with tab complete enabled
  import code
  import rlcompleter
  import readline
  readline.parse_and_bind("tab: complete")
  code.interact(local=locals(), banner='')

