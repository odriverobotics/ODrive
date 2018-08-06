import numpy as np
import math

def trapPlan(Xf, Xi, Vf, Vi, Af, Ai, Vmax, Amax, Dmax, dT=0.001):

    dX = Xf - Xi    # Distance to travel
    s = np.sign(dX)   # Sign 

    Ar = s*Amax     # Maximum Acceleration (signed)
    Dr = -s*Dmax    # Maximum Deceleration (signed)
    Vr = s*Vmax     # Maximum Velocity (signed)

    if(s*Vi > s*Vr):
        Ar = -s*Amax
    
    Ta = (Vr - Vi)/Ar   # Acceleration Time
    Td = (Vf - Vr)/Dr   # Deceleration Time

    

    ## Peak velocity handling
    if Vf == 0:
        dXmin = Ta*(Vr + Vi)/2 + Td*(Vr)/2  # Basic wedge profile
    elif np.sign(Vf) == np.sign(Vr):
        dXmin = Ta*(Vr + Vi)/2 + Td*(Vr - Vf)/2 + Td*Vf   # Wedge profile with an unfinished end
    else:
        dXmin = Ta*(Vr + Vi)/2 + Td*(Vr - s*Vf)/2   # Wedge profile that crosses Y axis on decel

    ## Short move handling
    if s*dXmin > s*dX:  
        Vr = s*math.sqrt(-1*Ar*(Vf*Vf-2*Dr*dX))/math.sqrt(Dr-Ar)    # Modified from paper to handle non-zero Vf
        Ta = max(0, (Vr - Vi)/Ar)
        Tv = 0
        Td = max(0, (Vf - Vr)/Dr)
    else:
        Tv = (dX - dXmin)/Vr    # non-short move, coast time at constant v
    
    ## We've computed Ta, Tv, Td, and Vr.  Time to produce a trajectory
    # Create the time series and preallocate the position, velocity, and acceleration arrays
    t_traj = np.arange(0, Ta+Tv+Td, dT)
    y = [None]*len(t_traj)
    yd = [None]*len(t_traj)
    ydd = [None]*len(t_traj)

    # We only know acceleration (Ar and Dr), so we integrate to create
    # the velocity and position curves
    for i in range(len(t_traj)):
        t = t_traj[i]
        if(t <= 0): # Initial conditions
            y[i] = Xi
            yd[i] = Vi
            ydd[i] = Ai
        elif(t <= Ta):  # Acceleration
            y[i] = y[i-1] + yd[i-1] * dT + (0.5*ydd[i-1]*dT*dT)
            yd[i] = yd[i-1] + ydd[i-1]*dT
            ydd[i] = Ar
        elif(t <= Ta+Tv):   # Coasting
            y[i] = y[i-1] + yd[i-1] * dT
            yd[i] = yd[i-1]
            ydd[i] = 0
        elif(t < Ta+Tv+Td): # Deceleration
            y[i] = y[i-1] + yd[i-1] * dT + (0.5*ydd[i-1]*dT*dT)
            yd[i] = yd[i-1] + ydd[i-1]*dT
            ydd[i] = Dr
        else:   # Final conditions
            y[i] = Xf
            yd[i] = Vf
            ydd[i] = Af
    
    return (y, yd, ydd)


(Y, Yd, Ydd) = trapPlan(10, 0, 0, 0, 0, 0, 10, 20, 20)
print(Y[len(Y)-1])