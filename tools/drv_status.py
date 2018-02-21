#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive.core
import time
import math

# Find a connected ODrive (this will block until you connect one)
my_drive = odrive.core.find_any(consider_usb=True, consider_serial=False, printer=print)

# Print DRV device regs for Motor 0
fault = my_drive.motor0.gate_driver.drv_fault
status_reg_1 = my_drive.motor0.gate_driver.status_reg_1
status_reg_2 = my_drive.motor0.gate_driver.status_reg_2
ctrl_reg_1 = my_drive.motor0.gate_driver.ctrl_reg_1
ctrl_reg_2 = my_drive.motor0.gate_driver.ctrl_reg_2

print("DRV Fault Code: " + str(fault))
print("Status Reg 1: " + str(status_reg_1) + " (" + format(status_reg_1, '#010b') + ")")
print("Status Reg 2: " + str(status_reg_2) + " (" + format(status_reg_2, '#010b') + ")")
print("Control Reg 1: " + str(ctrl_reg_1) + " (" + format(ctrl_reg_1, '#010b') + ")")
print("Control Reg 2: " + str(ctrl_reg_2) + " (" + format(ctrl_reg_2, '#010b') + ")")


