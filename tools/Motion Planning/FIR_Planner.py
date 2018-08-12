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

def FIR_trapPlan(Xf, Xi, Vi, Vmax, Amax, Dmax):

    dX_stop = Vi**2 / (2*Dmax) # Minimum stopping distance
    dX = Xf - Xi    # Distance to travel

    s = np.sign(dX)   # Sign of travel direction

    Ar = s*Amax     # Maximum Acceleration (signed)
    Dr = -s*Dmax    # Maximum Deceleration (signed)
    Vr = s*Vmax     # Maximum Velocity (signed)

    if abs(dX) <= dX_stop:  # Check for an overshoot condition (decelerate only)
        Ta = 0
        Tv = 0
        Vr = Vi
        Dr = -np.sign(Vi)*Dmax
        Td = abs(Vi) / Dmax

        print("Overshoot Move:")
        print("dX: {:.3f}\tdx_Stop: {:.3f}".format(dX, dX_stop))
        print("Xf: {:.3f}\tXi: {:.3f}\tVi: {:.3f}\tVmax: {:.3f}\tAmax: {:.3f}\t".format(Xf, Xi, Vi, Vmax, Amax))
        print("Ta: {:.3f}\tTv: {:.3f}\tTd: {:.3f}".format(Ta, Tv, Td))
        print("Ar: {:.3f}\tDr: {:.3f}\tVr: {:.3f}".format(Ar, Dr, Vr))
        print()        

    else:
    # Correct initial acceleration direction if needed
        if s*Vi > s*Vr:
            Ar = -s*Amax

        Ta = (Vr - Vi)/Ar   # Acceleration Time
        Td = -Vr/Dr   # Deceleration Time

        ## Peak velocity handling
        dXmin = Ta*(Vr + Vi)/2.0 + Td*(Vr)/2.0

        ## Short move handling
        if abs(dX) < abs(dXmin):
            print("Short Move:")
            print("dX: {:.3f}\tdXmin: {:.3f}".format(dX, dXmin))
            print("Xf: {:.3f}\tXi: {:.3f}\tVi: {:.3f}\tVmax: {:.3f}\tAmax: {:.3f}\t".format(Xf, Xi, Vi, Vmax, Amax))
            print("Ta: {:.3f}\tTd: {:.3f}\tVr: {:.3f}".format(Ta, Td, Vr))
            print()

            Vr = s*math.sqrt((-(Vi**2/Ar)-2*dX)/(1/Dr-1/Ar))
            Ta = max(0, (Vr - Vi)/Ar)
            Tv = 0
            Td = max(0, -Vr/Dr)
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
        if(t < 0): # Initial conditions
            y[i] = Xi
            yd[i] = Vi
            ydd[i] = Ar
        elif(t < Ta):  # Acceleration
            y[i] = (Ar * (t*t)/2) + (Vi * t) + Xi
            yd[i] = (Ar * t) + Vi
            ydd[i] = Ar
        elif(t < Ta+Tv):   # Coasting
            y[i] = y_Accel + (Vr * (t - Ta))
            yd[i] = Vr
            ydd[i] = 0
        elif(t <= Ta+Tv+Td): # Deceleration
            y[i] = y_Accel + (Vr * (t - Ta)) + Dr*((t - Tav)*(t - Tav))/2
            yd[i] = Vr + Dr*(t - Tav)
            ydd[i] = Dr

    return (y, yd, ydd, t_traj)


numRows = 2
numCols = 4
fig, axes = plt.subplots(numRows, numCols)
random.seed()
for x in range(numRows*numCols):

    Vmax = random.uniform(0.1, 20)
    Amax = random.uniform(0.1, 4)
    Dmax = Amax

    Xf = random.uniform(-100.0, 100.0)
    Xi = random.uniform(-100.0, 100.0)
    Vi = random.uniform(-Vmax*2, Vmax*2)

    # Vmax = .5
    # Amax = .5
    # Dmax = Amax
    # Xf = 10
    # Xi = -2
    # Vi = 4

    (Y, Yd, Ydd, t) = FIR_trapPlan(Xf, Xi, Vi, Vmax, Amax, Dmax)

    if(abs(Xf-Xi) <= Vi**2 / (2*Dmax)):
        print("Overshoot: ",Xf)
        print("Xf: {:.3f}\tXi: {:.3f}\tVi: {:.3f}\tVmax: {:.3f}\tAmax: {:.3f}\t".format(Xf, Xi, Vi, Vmax, Amax))
        print("Y: {:.3f}\tYd: {:.3f}\tYdd: {:.3f}".format(Y[-1], Yd[-1], Ydd[-1]))
        print()
        (Y2, Yd2, Ydd2, t2) = FIR_trapPlan(Xf, Y[-1], Yd[-1], Vmax, Amax, Dmax)
        Y.extend(Y2)
        Yd.extend(Yd2)
        Ydd.extend(Ydd2)
        t2 = t2 + t[-1]
        t = np.append(t, t2, axis=0)

    if(abs(Xf-Y[-1]) > 0.0001):
        print("Bad Final Position")
        print("Xf: {:.3f}\tXi: {:.3f}\tVi: {:.3f}\tVmax: {:.3f}\tAmax: {:.3f}\t".format(Xf, Xi, Vi, Vmax, Amax))
        print()
        # plt.figure()
        # plt.subplot(2,1,1)
        # plt.plot(t, Y)
        # plt.plot(t, Yd)
        # plt.plot(t[-1], Xf, 'b*')
        # plt.plot(t[-1], 0, 'r*')

        # plt.subplot(2,1,2)
        # plt.plot(t, Ydd)

        # plt.show()
    elif(abs(Yd[-1]) > 0.0001):
        print("Bad Final Velocity")
        print("Xf: {:.3f}\tXi: {:.3f}\tVi: {:.3f}\tVmax: {:.3f}\tAmax: {:.3f}\t".format(Xf, Xi, Vi, Vmax, Amax))
        print()
        # plt.figure()
        # plt.plot(t, Y)
        # plt.plot(t, Yd)
        # # plt.plot(t, Ydd)
        # plt.show()

    ax1 = axes[int(x/numCols), x%numCols]
    ax1.plot(t, Y)
    ax1.plot(t, Yd)
    ax1.plot(t[-1], Xf, 'b*')
    ax1.plot(t[-1], 0, 'r*')
    ax1.set_ylabel("Pos and Velocity")

    ax2 = ax1.twinx()
    ax2.plot(t, Ydd, color='tab:green')
    ax2.tick_params(axis='y', labelcolor='tab:green')
    ax2.set_ylabel("Acceleration")
    dX = abs(Xf - Y[-1])
    dV = abs(0 - Yd[-1])
    axes[int(x/numCols), x%numCols].set_title('Xf: {:.3f}  Xi: {:.3f}\ndX: {:.3f} dV: {:.3f}'.format(Xf, Xi, dX, dV))

plt.show()
