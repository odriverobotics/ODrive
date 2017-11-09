#! /usr/bin/env python3

import argparse

def parse_args():
  parser = argparse.ArgumentParser(description='Talk to a ODrive board over USB bulk channel.')
  return parser.parse_args()

if __name__ == '__main__':
  # parse args before other imports
  args = parse_args()

import sys
import time
import threading
from odrive import usbbulk

running = True
ready   = False

def main(args):
  global running
  print("ODrive USB Bulk Communications")
  print("---------------------------------------------------------------------")
  print("USAGE:")
  print("\tPOSITION_CONTROL:\n\t\tp MOTOR_NUMBER POSITION VELOCITY CURRENT")
  print("\tVELOCITY_CONTROL:\n\t\tv MOTOR_NUMBER VELOCITY CURRENT")
  print("\tCURRENT_CONTROL:\n\t\tc MOTOR_NUMBER CURRENT")
  print("\tHALT:\n\t\th")
  print("\tQuit Python Script:\n\t\tq")
  print("---------------------------------------------------------------------")
  # query device
  dev = usbbulk.poll_odrive_bulk_device(printer=print)
  print (dev.info())
  print (dev.init())
  # thread
  thread = threading.Thread(target=receive_thread, args=[dev])
  thread.start()
  while running:
    time.sleep(0.1)
    try:
      command = input("\r\nEnter ODrive command:\n")
      if 'q' in command:
        running = False
        sys.exit()
      else:
        dev.send(command)
    except:
      running = False

def receive_thread(dev):
  global ready

  while running:
    time.sleep(0.001)
    try:
      message = dev.receive(dev.receive_max())
      message_ascii = bytes(message).decode('ascii')
      print(message_ascii, end='')
      if "ODrive Firmware" in message_ascii:
        ready = True
    except:
      pass

if __name__ == "__main__":
   main(args)
