import numpy as np
import math

def trapPlan(Xf, Xi, Vf, Vi, Af, Ai, Vmax, Amax, Dmax, dT=0.001):

    s = np.sign(Xf - Xi)   # Sign 

    Ar = s*Amax     # Maximum Acceleration (signed)
    Dr = -s*Dmax    # Maximum Deceleration (signed)
    Vr = s*Vmax     # Maximum Velocity (signed)

    if(s*Vi > s*Vr):
        Ar = -s*Amax
    
    Ta = (Vr - Vi)/Ar   # Acceleration Time
    Td = (Vf - Vr)/Dr   # Deceleration Time

    dX = Xf - Xi    # Distance to travel
    if Vf == 0:
        dXmin = Ta*(Vr + Vi)/2 + Td*(Vr)/2
    elif np.sign(Vf) == np.sign(Vr):
        dXmin = Ta*(Vr + Vi)/2 + Td*(Vr-Vf)/2 + Td*Vf
    else:
        dXmin = Ta*(Vr + Vi)/2 + Td*(Vr - s*Vf)/2

    if s*dXmin > s*dX:
        Vr = s*math.sqrt(-1*Ar*(Vf*Vf-2*Dr*dX))/math.sqrt(Dr-Ar)
        Ta = max(0, (Vr - Vi)/Ar)
        Tv = 0
        Td = max(0, (Vf - Vr)/Dr)
    else:
        Tv = (dX - dXmin)/Vr
    
    t_traj = np.arange(0, Ta+Tv+Td, dT)
    y = [None]*len(t_traj)
    yd = [None]*len(t_traj)
    ydd = [None]*len(t_traj)

    for i in range(len(t_traj)):
        t = t_traj[i]
        if(t <= 0):
            y[i] = Xi
            yd[i] = Vi
            ydd[i] = Ai
        elif(t <= Ta):
            y[i] = y[i-1] + yd[i-1] * dT + (0.5*ydd[i-1]*dT*dT)
            yd[i] = yd[i-1] + ydd[i-1]*dT
            ydd[i] = Ar
        elif(t <= Ta+Tv):
            y[i] = y[i-1] + yd[i-1] * dT
            yd[i] = yd[i-1]
            ydd[i] = 0
        elif(t <= Ta+Tv+Td):
            y[i] = y[i-1] + yd[i-1] * dT + (0.5*ydd[i-1]*dT*dT)
            yd[i] = yd[i-1] + ydd[i-1]*dT
            ydd[i] = Dr
    
    return (y, yd, ydd)


(Y, Yd, Ydd) = trapPlan(10, 0, 0, 0, 0, 0, 10, 20, 20)
print(Y[len(Y)-1])