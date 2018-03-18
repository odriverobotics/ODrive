#!/usr/bin/env python3
"""
Liveplotter
"""

import time
import threading
import odrive.discovery
from odrive.utils import start_liveplotter

data_rate = 100
plot_rate = 10
num_samples = 1000

my_odrive = odrive.discovery.find_any()

# If you want to plot different values, change them here.
# You can plot any number of values concurrently.
start_liveplotter(lambda: [my_odrive.motor0.encoder.pll_pos,
                           my_odrive.motor1.encoder.pll_pos])

