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

# Make sure the script terminates when the user closes the plotter
cancellation_token = threading.Event()
def handle_close(evt):
    cancellation_token.set()
fig = plt.figure()
fig.canvas.mpl_connect('close_event', handle_close)

def fetch_data():
    global vals
    global cancellation_token
    while not cancellation_token.is_set():
        vals.append(my_odrive.motor0.timing_log.TIMING_LOG_FOC_CURRENT)
        if len(vals) > num_samples:
            vals = vals[-num_samples:]
        time.sleep(1/data_rate)

# TODO: use animation for better UI performance, see:
# https://matplotlib.org/examples/animation/simple_anim.html
def plot_data():
    global vals
    global cancellation_token
    while not cancellation_token.is_set():
        plt.clf()
        plt.plot(vals)
        #time.sleep(1/plot_rate)
        fig.canvas.flush_events()

fetch_thread = threading.Thread(target=fetch_data, daemon=True)
fetch_thread.start()

plot_data()
