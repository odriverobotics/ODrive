# Copyright (c) 2018 Paul Guénette
# Copyright (c) 2018 Oskar Weigl

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# This algorithm is based on:
# FIR filter-based online jerk-constrained trajectory generation
# https://www.researchgate.net/profile/Richard_Bearee/publication/304358769_FIR_filter-based_online_jerk-controlled_trajectory_generation/links/5770ccdd08ae10de639c0ff7/FIR-filter-based-online-jerk-controlled-trajectory-generation.pdf

import numpy as np
import math
import matplotlib.pyplot as plt
import random

# Symbol                     Description
# Ta, Tv and Td              Duration of the stages of the AL profile
# Xi and Vi                  Adapted initial conditions for the AL profile
# Xf                         Position set-point
# s                          Direction (sign) of the trajectory
# Vmax, Amax, Dmax and jmax  Kinematic bounds
# Ar, Dr and Vr              Reached values of acceleration and velocity

# Test scales:
pos_range  = 10000.0
Vmax_range = 8000.0
Amax_range = 10000.0
plot_range = 10000.0


def PlanTrap(Xf, Xi, Vi, Vmax, Amax, Dmax):
    dX = Xf - Xi    # Distance to travel
    stop_dist = Vi**2 / (2*Dmax)  # Minimum stopping distance
    dXstop = np.sign(Vi)*stop_dist # Minimum stopping displacement
    s = np.sign(dX - dXstop)   # Sign of coast velocity (if any)
    Ar = s*Amax     # Maximum Acceleration (signed)
    Dr = -s*Dmax    # Maximum Deceleration (signed)
    Vr = s*Vmax     # Maximum Velocity (signed)

    # If we start with a speed faster than cruising, then we need to decel instead of accel
    # aka "double deceleration move" in the paper
    if s*Vi > s*Vr:
        print("Handbrake!")
        Ar = -s*Amax

    # Time to accel/decel to/from Vr (cruise speed)
    Ta = (Vr-Vi)/Ar
    Td = -Vr/Dr

    # Integral of velocity ramps over the full accel and decel times to get
    # minimum displacement required to reach cuising speed
    dXmin = Ta*(Vr+Vi)/2.0 + Td*(Vr)/2.0

    # Are we displacing enough to reach cruising speed?
    if s*dX < s*dXmin:
        print("Short Move:")
        # From paper:
        # Vr = s*math.sqrt((-(Vi**2/Ar)-2*dX)/(1/Dr-1/Ar))
        # Simplified for less divisions:
        Vr = s*math.sqrt((Dr*Vi**2 + 2*Ar*Dr*dX) / (Dr-Ar))
        Ta = max(0, (Vr - Vi)/Ar)
        Td = max(0, -Vr/Dr)
        Tv = 0
    else:
        print("Long move:")
        Tv = (dX - dXmin)/Vr # Coasting time

    Tf = Ta+Tv+Td

    print("Xi: {:.2f}\tXf: {:.2f}\tVi: {:.2f}".format(Xi, Xf, Vi))
    print("Amax: {:.2f}\tVmax: {:.2f}\tDmax: {:.2f}".format(Amax, Vmax, Dmax))
    print("dX: {:.2f}\tdXst: {:.2f}\tdXmin: {:.2f}".format(dX, dXstop, dXmin))
    print("Ar: {:.2f}\tVr: {:.2f}\tDr: {:.2f}".format(Ar, Vr, Dr))
    print("Ta: {:.2f}\tTv: {:.2f}\tTd: {:.2f}".format(Ta, Tv, Td))

    return (Ar, Vr, Dr, Ta, Tv, Td, Tf)

def EvalTrap(Xf, Xi, Vi, Ar, Vr, Dr, Ta, Tv, Td, Tf):
    # Create the time series and preallocate the position, velocity, and acceleration arrays
    t_traj = np.arange(0, Tf+0.1, 1/10000)
    y = [None]*len(t_traj)
    yd = [None]*len(t_traj)
    ydd = [None]*len(t_traj)

    # We only know acceleration (Ar and Dr), so we integrate to create
    # the velocity and position curves
    y_Accel = Xi + Vi*Ta + 0.5*Ar*Ta**2

    for i in range(len(t_traj)):
        t = t_traj[i]
        if t < 0: # Initial conditions
            y[i]   = Xi
            yd[i]  = Vi
            ydd[i] = 0
        elif t < Ta: # Acceleration
            y[i]   = Xi + Vi*t + 0.5*Ar*t**2
            yd[i]  = Vi + Ar*t
            ydd[i] = Ar
        elif t < Ta+Tv: # Coasting
            y[i]   = y_Accel + Vr*(t-Ta)
            yd[i]  = Vr
            ydd[i] = 0
        elif t < Tf: # Deceleration
            td     = t-Tf
            y[i]   = Xf + 0*td + 0.5*Dr*td**2
            yd[i]  = 0 + Dr*td
            ydd[i] = Dr
        elif t >= Tf: # Final condition
            y[i]   = Xf
            yd[i]  = 0
            ydd[i] = 0
        else:
            raise ValueError("t = {} is outside of considered range".format(t))
        
    dy = np.diff(y)
    dy_max = np.max(np.abs(dy))
    dyd = np.diff(yd)
    dyd_max = np.max(np.abs(dyd))
    print("dy_max: {:.2f}\tdyd_max: {:.2f}".format(dy_max, dyd_max))

    error = False
    if dy_max/pos_range > 0.001:
        print("---------- Bad Pos Continuity --------------------")
        error = True
    if dyd_max/Vmax_range > 0.001:
        print("---------- Bad Vel Continuity --------------------")
        error = True
    if abs(Xi-y[0]) > 0.0001:
        print("---------- Bad Initial Position --------------------")
        error = True
    if abs(Xf-y[-1]) > 0.0001:
        print("---------- Bad Final Position --------------------")
        error = True
    if abs(Vi-yd[0]) > 0.0001:
        print("---------- Bad Initial Velocity --------------------")
        error = True
    if abs(yd[-1]) > 0.0001:
        print("---------- Bad Final Velocity --------------------")
        error = True

    if error:
        import ipdb; ipdb.set_trace()

    return (y, yd, ydd, t_traj)

def graphical_test():
    numRows = 3
    numCols = 5
    fig, axes = plt.subplots(numRows, numCols)
    random.seed(3) # Repeatable tests by using specific seed
    for x in range(numRows*numCols):
        rownow = int(x/numCols)
        colnow = x % numCols
        print("row: {}, col: {}".format(rownow, colnow))

        Vmax = random.uniform(0.1*Vmax_range, Vmax_range)
        Amax = random.uniform(0.1*Amax_range, Amax_range)
        Dmax = Amax
        Xf = random.uniform(-pos_range, pos_range)
        Xi = random.uniform(-pos_range, pos_range)
        if random.random() <= 0.5:
            Vi = random.uniform(-Vmax*1.5, Vmax*1.5)
        else:
            Vi = 0

        (Ar, Vr, Dr, Ta, Tv, Td, Tf) = PlanTrap(Xf, Xi, Vi, Vmax, Amax, Dmax)
        (Y, Yd, Ydd, t) = EvalTrap(Xf, Xi, Vi, Ar, Vr, Dr, Ta, Tv, Td, Tf)

        # Plotting
        ax1 = axes[rownow, colnow]
        # Vel limits (draw first for clearer z-order)
        ax1.plot([t[0], t[-1]], [Vmax, Vmax], 'g--')
        ax1.plot([t[0], t[-1]], [-Vmax, -Vmax], 'g--')

        ax1.plot(t, Y) # Pos
        ax1.plot(t, Yd) # Vel
        ax1.plot(0, Xi, 'bo') # Pos Initial
        ax1.plot(0, Vi, 'ro') # Vel Initial
        ## TODO: pull out Ta+Td+Td from planner for correct plot points
        ax1.plot(t[-1]-0.1, Xf, 'b*') # Pos Final
        ax1.plot(t[-1]-0.1, 0, 'r*') # Vel Final

        ax1.set_ylim(-plot_range, plot_range)

        print()

    plt.show()

def large_test():
    random.seed(1) # Repeatable tests by using specific seed
    for x in range(100):
        print("Test {}".format(x))
        Vmax = random.uniform(0.1*Vmax_range, Vmax_range)
        Amax = random.uniform(0.1*Amax_range, Amax_range)
        Dmax = Amax
        Xf = random.uniform(-pos_range, pos_range)
        Xi = random.uniform(-pos_range, pos_range)
        if random.random() <= 0.5:
            Vi = random.uniform(-Vmax*1.5, Vmax*1.5)
        else:
            Vi = 0

        (Ar, Vr, Dr, Ta, Tv, Td, Tf) = PlanTrap(Xf, Xi, Vi, Vmax, Amax, Dmax)
        (Y, Yd, Ydd, t) = EvalTrap(Xf, Xi, Vi, Ar, Vr, Dr, Ta, Tv, Td, Tf)

        print()

if __name__ == '__main__':
    large_test()
    graphical_test()