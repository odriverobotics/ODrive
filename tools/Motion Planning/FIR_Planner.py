import numpy as np
import math
import matplotlib.pyplot as plt
import random

# Symbol                        Description
# Ta, Tv and Td                 Duration of the stages of the AL profile
# q0f , v0f and a0f             Initial conditions of the jerk-limited trajectory
# q0 and v0                     Adapted initial conditions for the AL profile
# qe                            Position set-point
# s                             Direction (sign) of the trajectory
# vmax, amax, dmax and jmax     Kinematic bounds
# vr, ar and dr                 Reached values of velocity and acceleration
# Tj , Tja, Tjv and Tjd         Length of the constant jerk stages (FIR filter time)

def FIR_trapPlan(Xf, Xi, Vi, Ai, Vmax, Amax, Dmax):

    dX = Xf - Xi    # Distance to travel
    s = np.sign(dX)   # Sign

    Ar = s*Amax     # Maximum Acceleration (signed)
    Dr = -s*Dmax    # Maximum Deceleration (signed)
    Vr = s*Vmax     # Maximum Velocity (signed)

    if(s*Vi > s*Vr):
        Ar = -s*Amax
    
    Ta = (Vr - Vi)/Ar   # Acceleration Time
    Td = (-Vr)/Dr   # Deceleration Time

    ## Peak velocity handling
    dXmin = Ta*(Vr + Vi)/2.0 + Td*(Vr)/2.0

    ## Short move handling
    if s*dXmin > s*dX:  
        Vr = s*math.sqrt((-(Vi**2 / Ar)-(2*dX))/(1/Dr - 1/Ar))
        Ta = max(0, (Vr - Vi)/Ar)
        Tv = 0
        Td = max(0, (-Vr)/Dr)
    else:
        Tv = (dX - dXmin)/Vr    # non-short move, coast time at constant v


    ## We've computed Ta, Tv, Td, and Vr.  Time to produce a trajectory
    # Create the time series and preallocate the position, velocity, and acceleration arrays
    t_traj = np.linspace(0, Ta+Tv+Td, 10000)
    y = [None]*len(t_traj)
    yd = [None]*len(t_traj)
    ydd = [None]*len(t_traj)

    # We only know acceleration (Ar and Dr), so we integrate to create
    # the velocity and position curves
    y_Accel = (Ar*Ta*Ta) / 2 + (Vi * Ta) + Xi
    Tav = Ta + Tv

    for i in range(len(t_traj)):
        t = t_traj[i]
        if(t <= 0): # Initial conditions
            y[i] = Xi
            yd[i] = Vi
            ydd[i] = Ai
        elif(t <= Ta):  # Acceleration
            y[i] = (Ar * (t*t)/2) + (Vi * t) + Xi
            yd[i] = (Ar * t) + Vi
            ydd[i] = Ar
        elif(t <= Ta+Tv):   # Coasting
            y[i] = y_Accel + (Vr * (t - Ta))
            yd[i] = Vr
            ydd[i] = 0
        elif(t <= Ta+Tv+Td): # Deceleration
            y[i] = y_Accel + (Vr * (t - Ta)) + Dr*((t - Tav)*(t - Tav))/2
            yd[i] = Vr + Dr*(t - Tav)
            ydd[i] = Dr

    return (y, yd, ydd, t_traj)


#(Y, Yd, Ydd, t) = trapPlan(10, 0, 0, 0, 15.122, 22.022, 22.022)
numRows = 2
numCols = 4
fig, axes = plt.subplots(numRows, numCols, sharey='all')
random.seed()
for x in range(8):

    Vmax = random.uniform(0.1, 20)
    Amax = random.uniform(0.1, 40)

    Xi = random.uniform(-100.0, 100.0)
    Vi = random.uniform(-Vmax, Vmax)
    Xf = random.uniform(-100.0, 100.0)

    (Y, Yd, Ydd, t) = FIR_trapPlan(Xf, Xi, 0, 0, Vmax, Amax, Amax)

    if(abs(Xf-Y[-1]) > 0.0001):
        print("Bad final position: ", Xf, Y[-1], abs(Xf-Y[-1]))
        plt.plot(t, Y)
        plt.plot(t, Yd)
        plt.plot(t, Ydd)
        plt.show()
    
    elif(abs(Yd[-1]) > 0.0001):
        print("Bad final Velocity: ", Yd[-1])
        plt.plot(t, Y)
        plt.plot(t, Yd)
        plt.plot(t, Ydd)
        plt.show()

    else:
        print("Position Error: {:.6f}\tVelocity Error: {:.6f}".format(abs(Xf-Y[-1]),abs(Yd[-1])))

    axes[int(x/numCols), x%numCols].plot(t, Y)
    axes[int(x/numCols), x%numCols].plot(t, Yd)
    axes[int(x/numCols), x%numCols].plot(t, Ydd)
    axes[int(x/numCols), x%numCols].set_title('Xi: {:.3f}  Xf: {:.3f}'.format(Xi, Xf))

plt.show()
