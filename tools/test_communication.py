#!/usr/bin/env python3

import argparse

def parse_args():
  parser = argparse.ArgumentParser(description='Talk to a ODrive board over USB bulk channel.')
  parser.add_argument("-v", "--verbose", action="store_true",
                      help="print debug information")
  parser.add_argument("-d", "--device", action="store",
                      help="Specifies the device to talk to. If this parameter is provided but the device is not available, I will exit immediately."
                      "If not provided I will do my best to find an ODrive on USB and serial ports.")
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
      motor.pos_setpoint = pos
    elif command == "q" or command == 'exit':
      sys.exit()
    else:
      print("unknown command \"" + command + "\"")

def main(args):
  if (args.verbose):
    printer = print
  else:
    printer = noprint

  print("ODrive Control Utility")
  print("---------------------------------------------------------------------")
  print("USAGE:")
  print("\tPOSITION_CONTROL:\n\t\tp MOTOR_NUMBER POSITION VELOCITY CURRENT")
  print("\tVELOCITY_CONTROL:\n\t\tv MOTOR_NUMBER VELOCITY CURRENT")
  print("\tCURRENT_CONTROL:\n\t\tc MOTOR_NUMBER CURRENT")
  print("\tQuit Python Script:\n\t\tq")
  print("---------------------------------------------------------------------")

  history = prompt_toolkit.history.InMemoryHistory()

  while True:
    # Connect to device
    if (args.device is None):
      print("Waiting for device...")
      device = odrive.core.find_any(printer=printer)
    else:
      device = odrive.core.open(args.device, printer=printer)

    try:
      command_prompt_loop(device, history)
    except odrive.protocol.ChannelBrokenException:
      print("ODrive disconnected")
      if not args.device is None:
        sys.exit()
  

if __name__ == "__main__":
   main(args)
