import os
import matplotlib.pyplot as plt
from control.matlab import *

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

sys = series(motor(2.5), pulley(0.015), mass(0.10, 0, 0))
yout, T, xout = step(sys, return_x=True)
print(yout)
# plt.plot(T, yout)
plt.plot(T, xout)
plt.legend(['Displacement', 'Velocity'])
plt.show()