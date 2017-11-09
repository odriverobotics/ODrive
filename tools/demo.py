#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

import odrive.core
import time
import math

# Find a connected ODrive (this will block until you connect one)
odrives = odrive.core.find_all(printer=print)
odrives = list(odrives) #force eval of generator to test finding functions
my_drive = odrives[0]
# my_drive = odrive.core.find_any(printer=print)

# The above call returns a python object with a dynamically generated type. The
# type hierarchy will correspond to the endpoint list in `MotorControl/protocol.cpp`.
# You can also inspect the object using the dir-function:
#print(dir(my_drive))
#print(dir(my_drive.motor0))
# TODO: maybe provide an introspection method that dumps the whole type hierarchy at once

# To read a value, simply read the property
print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

# Or to change a value, just assign to the property
my_drive.motor0.pos_setpoint = 3.14
print("Position setpoint is " + str(my_drive.motor0.pos_setpoint))

# And this is how function calls are done:
my_drive.motor0.set_pos_setpoint(0.0, 0.0, 0.0)

# little sine wave to test
t0 = time.monotonic()
while False:
    setpoint = 10000.0 * math.sin((time.monotonic() - t0)*2)
    print("goto " + str(int(setpoint)))
    my_drive.motor0.set_pos_setpoint(setpoint, 0.0, 0.0)
    time.sleep(0.01)


# Some more things you can try:

# Write to a read-only property:
my_drive.vbus_voltage = 11.0  # fails with `AttributeError: can't set attribute`

# Assign an incompatible value:
# my_drive.motor0.pos_setpoint = "I like trains"  # fails with `TypeError: expected value of type float`
