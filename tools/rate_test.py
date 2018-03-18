#!/usr/bin/env python3

import time
import odrive.discovery
import matplotlib.pyplot as plt
import numpy as np

# Find a connected ODrive (this will block until you connect one)
print("Waiting for ODrive...")
myOdrive = odrive.discovery.find_any()
print("connected")

plt.ion()

numFrames = 10000
vals = []
for _ in range(numFrames):
    vals.append(myOdrive.motor0.loop_counter)

plt.plot(vals)

loopsPerFrame = (vals[-1] - vals[0])/numFrames
loopsPerSec = (168000000/(2*10192))
FramePerSec = loopsPerSec/loopsPerFrame
print(FramePerSec)