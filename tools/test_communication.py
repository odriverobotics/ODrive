#!/usr/bin/env python3

import argparse

def parse_args():
  parser = argparse.ArgumentParser(description='Talk to a ODrive board over USB or serial.')
  parser.add_argument("-v", "--verbose", action="store_true",
                      help="print debug information")
  group = parser.add_mutually_exclusive_group()
  group.add_argument("-u", "--usb", action="store",
                      help="Specifies the USB port on which the device is connected. "
                      "For example \"001:014\" means bus 001, device 014. The numbers can be obtained "
                      "using `lsusb`.")
  group.add_argument("-s", "--serial", action="store",
                      help="Specifies the serial port on which the device is connected. "
                      "For example \"/dev/ttyUSB0\". Use `ls /dev/tty*` to find your port name.")
  return parser.parse_args()

if __name__ == '__main__':
  # parse args before other imports
  args = parse_args()

import sys
import time
import threading
import prompt_toolkit
import re
import tempfile
import odrive.core


def noprint(str):
  pass

def print_usage():
  print("ODrive Control Utility")
  print("---------------------------------------------------------------------")
  print("USAGE:")
  print("\tPOSITION_CONTROL:\n\t\tp MOTOR_NUMBER POSITION VELOCITY CURRENT")
  print("\tVELOCITY_CONTROL:\n\t\tv MOTOR_NUMBER VELOCITY CURRENT")
  print("\tCURRENT_CONTROL:\n\t\tc MOTOR_NUMBER CURRENT")
  # TODO: implement the following features:
  #print("\tList parameters:\n\t\tmotor0.[TAB]")
  #print("\tShow parameter:\n\t\tmotor0.pos_setpoint")
  #print("\tChange parameter:\n\t\tmotor0.pos_setpoint = 0")
  # print("\tHALT:\n\t\th")
  print("\tQuit Python Script:\n\t\tq")
  print("---------------------------------------------------------------------")

def command_prompt_loop(device, history):
  """
  Presents the command prompt indefinitely until something goes wrong
  """
  # Load all motors
  motors = []
  if "motor0" in dir(device):
    motors.append(device.motor0)
  if "motor1" in dir(device):
    motors.append(device.motor1)

  print("Connected - have {} {}".format(len(motors), "motor" if len(motors) == 1 else "motors"))

  while True:
    try:
      command = prompt_toolkit.prompt(
        "ODrive> ",
        history=history).strip()
    except EOFError:
      command = "exit"

    if len(command) == 0:
      continue
    elif command.startswith("p "):
      args = command[2:].split()
      try:
        motor = motors[int(args[0])]
        pos = float(args[1])
        vel = float(args[2])
        cur = float(args[3])
      except (ValueError, IndexError):
        print("invalid command format")
        continue
      motor.set_pos_setpoint(pos, vel, cur)
    elif command.startswith("v "):
      args = command[2:].split()
      try:
        motor = motors[int(args[0])]
        vel = float(args[1])
        cur = float(args[2])
      except (ValueError, IndexError):
        print("invalid command format")
        continue
      motor.set_vel_setpoint(vel, cur)
    elif command.startswith("c "):
      args = command[2:].split()
      try:
        motor = motors[int(args[0])]
        cur = float(args[1])
      except (ValueError, IndexError):
        print("invalid command format")
        continue
      motor.set_current_setpoint(cur)
    elif command == "h" or command == '?' or command == 'help':
      print_usage()
    elif command == "q" or command == 'exit':
      sys.exit(0)
    else:
      print("unknown command \"" + command + "\"")

def main(args):
  if (args.verbose):
    printer = print
  else:
    printer = noprint

  history = prompt_toolkit.history.InMemoryHistory()

  print_usage()

  while True:
    # Connect to device
    if not args.usb is None:
      try:
        bus = int(args.usb.split(":")[0])
        address = int(args.usb.split(":")[1])
      except (ValueError, IndexError):
        print("the --usb argument must look something like this: \"001:014\"")
        sys.exit(1)
      try:
        device = odrive.core.open_usb(bus, address, printer=printer)
      except odrive.protocol.DeviceInitException as ex:
        print(str(ex))
        sys.exit(1)
    elif not args.serial is None:
      device = odrive.core.open_serial(args.serial, printer=printer)
    else:
      print("Waiting for device...")
      device = odrive.core.find_any(printer=printer)
      autoconnected = True

    try:
      command_prompt_loop(device, history)
    except odrive.protocol.ChannelBrokenException:
      print("ODrive disconnected")
      if not autoconnected:
        sys.exit(1)
  

if __name__ == "__main__":
   main(args)
