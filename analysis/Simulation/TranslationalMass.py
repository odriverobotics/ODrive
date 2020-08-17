import os
import matplotlib.pyplot as plt
from control.matlab import *
import numpy as np


# Input: Current (A)
# Output: Torque (Nm)
# Params: Kt (Nm/A)
def motor(Kt):
    return tf(Kt, 1)

# Mass-Spring-Damper
# Input: Force
# Output: Position
# Params:   m (kg)
#           b 
#           k (N/m)
def mass(m, b, k):
    A = [[0, 1.], [-k/m, -b/m]]
    B = [[0], [1/m]]
    C = [[1., 0]]
    return ss(A, B, C, 0)

# Input: Torque (Nm)
# Output: Force (N)
# Params: r (m)
def pulley(r):
    return tf(r, 1)

# Make s a transfer function s/1
s = tf('s')
print(s)

# build a new transfer function using our variable s as a handy placeholder
sys = 1 / (s*s + s + 1)
print(sys)

# Hit the system with a step command
yout, T = step(sys)
plt.plot(T, yout)

# convert our continuous time model to discrete time via Tustin at 0.01s timestep
sysd = c2d(tf(sys), 0.01, method='tustin')
print(sysd)

# Hit the discrete system with a step command, and sample it at 0.01 timestep from 0 to 14 seconds
yout, T = step(sysd, np.arange(0, 14, 0.01))
plt.plot(T, yout)
plt.legend(['Continuous', 'Discrete'])

# Build a system based on the series connection of the motor, pulley, and mass "blocks"
sys = series(motor(2.5), pulley(0.015), mass(0.10, .1, .1))
print(tf(sys))

# Step our series system, returning y (outputs) and x (states)
yout, T, xout = step(sys, return_x=True)
plt.figure()
plt.plot(T, yout)
plt.plot(T, xout)
plt.legend(['Displacement', r'$x$', r'$\dot{x}$'])
plt.show()