#!/usr/bin/env python3
"""
Liveplotter
"""

import time
import odrive.core
import matplotlib.pyplot as plt
import numpy as np
import threading

data_rate = 100
plot_rate = 10
num_samples = 1000

my_odrive = odrive.core.find_any()

plt.ion()
global vals
vals = []

def fetch_data():
    global vals
    while True:
        point = []
        point.append(my_odrive.motor0.timing_log.TIMING_LOG_ADC_CB_M0_I)
        point.append(my_odrive.motor1.timing_log.TIMING_LOG_ADC_CB_M1_I)
        point.append(my_odrive.motor0.timing_log.TIMING_LOG_ADC_CB_M0_DC - 8192)
        point.append(my_odrive.motor1.timing_log.TIMING_LOG_ADC_CB_M1_DC - 8192)
        vals.append(point)
        if len(vals) > num_samples:
            vals = vals[-num_samples:]
        time.sleep(1/data_rate)

def plot_data():
    global vals
    while True:
        plt.clf()
        # for series in vals:
        #     plt.plot(series)
        plt.plot(vals)
        plt.pause(1/plot_rate)

fetch_thread = threading.Thread(target=fetch_data, daemon=True)
fetch_thread.start()

plot_data()
