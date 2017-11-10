#!/usr/bin/env python3
"""
Liveplotter
"""

import time
import odrive.core
import matplotlib.pyplot as plt
import numpy as np
import threading
from IPython import embed

data_rate = 100
plot_rate = 10
num_samples = 1000

myOdrive = odrive.core.find_any()

plt.ion()
global vals
vals = []

def fetch_data():
    global vals
    while True:
        vals.append(myOdrive.motor0.encoder.pll_pos)
        if len(vals) > num_samples:
            vals = vals[-num_samples:]
        time.sleep(1/data_rate)

def plot_data():
    global vals
    while True:
        plt.clf()
        plt.plot(vals)
        plt.pause(1/plot_rate)

fetch_thread = threading.Thread(target=fetch_data, daemon=True)
fetch_thread.start()

plot_data()