import time
import odrive.core
import matplotlib.pyplot as plt
import numpy as np

myOdrive = odrive.core.find_any()

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