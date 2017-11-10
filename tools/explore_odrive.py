"""
Load an odrive object to play with in the IPython interactive shell.
"""

import odrive.core
my_odrive = odrive.core.find_any()

print('')
print('ODRIVE EXPLORER')
print('')
print('Run this script with the following command:')
print('ipython -i explore_odrive.py')
print('')
print('You can now type "my_odrive." and press <tab>')
print('This will present you with all the properties that you can reference')
print('')
print('For example: "my_odrive.motor0.encoder.pll_pos"')
print('will print the current encoder position on motor 0')
print('and "my_odrive.motor0.pos_setpoint = 10000"')
print('will send motor0 to 10000')
