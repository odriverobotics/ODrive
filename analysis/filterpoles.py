
import numpy as np
import sympy as sp
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

do_mass_spring = True
do_PLL = True

bandwidth = 1

pos_ref = 0
vel_ref = 0
init_pos = 1000
init_vel = 0

if do_mass_spring:
    # 2nd order system response with manipulation of velocity only
    # This is similar to a mass/spring/damper system
    # pos_dot = vel
    # vel_dot = Kp * delta_pos + Ki * delta_vel

    Ki = 2.0 * bandwidth
    Kp = 0.25 * Ki**2

    def get_Xdot(t, X):
        pos = X[0]
        vel = X[1]

        pos_err = pos_ref - pos
        vel_err = vel_ref - vel
        
        pos_dot = vel
        vel_dot = Kp * pos_err + Ki * vel_err

        Xdot = [pos_dot, vel_dot]
        return Xdot

    sol = solve_ivp(get_Xdot, (0.0, 10.0), [init_pos, init_vel], t_eval=np.linspace(0, 10, 100))

    plt.plot(np.transpose(sol.t), np.transpose(sol.y[0,:]), label='physical mass pos')
    plt.plot(np.transpose(sol.t), np.transpose(sol.y[1,:]), label='physical mass vel')


if do_PLL:
    # 2nd order system response with a "slipping displacement" term directly on position
    # This formulation is given in the sensorless PLL paper
    # pos_dot = vel + Kp * delta_pos
    # vel_dot = Ki * delta_pos

    Kp = 2.0 * bandwidth
    Ki = 0.25 * Kp**2

    def get_Xdot(t, X):
        pos = X[0]
        vel = X[1]

        pos_err = pos_ref - pos
        vel_err = vel_ref - vel

        pos_dot = vel + Kp * pos_err
        vel_dot = Ki * pos_err

        Xdot = [pos_dot, vel_dot]
        return Xdot

    sol = solve_ivp(get_Xdot, (0.0, 10.0), [init_pos, init_vel], t_eval=np.linspace(0, 10, 100))

    plt.plot(np.transpose(sol.t), np.transpose(sol.y[0,:]), label='PLL pos')
    plt.plot(np.transpose(sol.t), np.transpose(sol.y[1,:]), label='PLL vel')



plt.legend()
plt.show(block=False)