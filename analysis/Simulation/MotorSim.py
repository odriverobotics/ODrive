# this file is for the simulation of a 3-phase synchronous motor

import numpy as np
import scipy as sp
import scipy.signal as signal
import scipy.integrate
import matplotlib.pyplot as plt

def sign(num):
    if num > 0:
        return 1
    elif num < 0:
        return -1
    else:
        return 0

# example params for d5065 motor
# phase_R = 0.039 Ohms
# phase_L = 0.0000157 H
# pole_pairs = 7
# KV = 270

class motor_pmsm_electrical:
    # class for simulating PMSM electrical dynamics in dq reference frame
    # R, L input are phase-neutral, not phase-phase
    def __init__(self, R, L_q, L_d, KV, pole_pairs):
        # KV is published KV
        kt = 8.27/KV
        self.lambda_m = 2*kt/(3*pole_pairs) #speed constant in Vs/rad (electrical rad)
        self.L_q = L_q
        self.L_d = L_d
        self.R = R
        self.pole_pairs = pole_pairs

        self.V_q = 0
        self.V_d = 0
        self.w_e = 0

    def diff_eqs(self, t, y, V_d, V_q, w):
        # this is for solving with solve_ivp or similar
        # t is time, y is vector of state: [I_d, I_q], V_d and V_q are input voltages in dq ref frame, w is electrical freq
        I_d = y[0]
        I_q = y[1]

        # set these equal to the inputs for plotting
        self.V_d = V_d
        self.V_q = V_q
        self.w_e = w

        I_d_dot = V_d / self.L_d - self.R / self.L_d * I_d + w * self.L_q / self.L_d * I_q
        I_q_dot = V_q / self.L_q - self.R / self.L_q * I_q - w * self.L_d / self.L_q * I_d - w * self.lambda_m / self.L_q

        return np.array([I_d_dot, I_q_dot])

# example params for D5065 motor
# J = 1e-4
# b_coulomb = 0.001
# b_viscous = 0.001

class motor_pmsm_mechanical:
    def __init__(self, J, b_coulomb, b_viscous):
        # J is moment of inertia
        # b_coulomb is coulomb friction coefficient
        # b_viscous is viscous friction coefficient
        self.J = J
        self.b_c = b_coulomb
        self.b_v = b_viscous

    def diff_eqs(self, t, y, torque):
        theta = y[0]
        theta_dot = y[1]

        theta_ddot = (1/self.J) * (torque - self.b_v * theta_dot - self.b_c * sign(theta_dot))

        return np.array([theta_dot, theta_ddot])

class motor_pmsm_combined:
    def __init__(self, J, b_coulomb, b_viscous, R, L_q, L_d, KV, pole_pairs):
        # J is moment of inertia
        # b_coulomb is coulomb friction coefficient
        # b_viscous is viscous friction coefficient
        self.J = J
        self.b_c = b_coulomb
        self.b_v = b_viscous

        kt = 8.27/KV
        self.lambda_m = 2*kt/(3*pole_pairs) #speed constant in Vs/rad (electrical rad)
        self.L_q = L_q
        self.L_d = L_d
        self.R = R
        self.pole_pairs = pole_pairs

    def diff_eqs(self, t, y, V_d, V_q):
        # inputs are V_d, V_q
        # state is y, y = [theta, theta_dot, I_d, I_q]

        theta = y[0]
        theta_dot = y[1]
        I_d = y[2]
        I_q = y[3]

        torque = 3*self.pole_pairs/2 * (self.lambda_m * I_q + (self.L_d - self.L_q)*I_d*I_q)

        # theta_dot = theta_dot, no ode here
        theta_ddot = (1/self.J) * (torque - self.b_v * theta_dot - self.b_c * sign(theta_dot))
        I_d_dot = V_d / self.L_d - self.R / self.L_d * I_d + theta_dot*self.pole_pairs * self.L_q / self.L_d * I_q
        I_q_dot = V_q / self.L_q - self.R / self.L_q * I_q - theta_dot*self.pole_pairs * self.L_d / self.L_q * I_d - theta_dot*self.pole_pairs * self.lambda_m / self.L_q

        return np.array([theta_dot, theta_ddot, I_d_dot, I_q_dot])

def inverter(vbus, timings, current):
    # this function should take the relevant inputs and output voltages in dq reference frame.
    pass

class motor:
    def __init__(self, J, b_coulomb, b_viscous, R, L_q, L_d, KV, pole_pairs, dT):
        self.dT = dT
        self.b_coulomb = b_coulomb
        self.KV = KV
        self.pole_pairs = pole_pairs
        kt = 8.27/KV
        self.lambda_m = 2*kt/(3*pole_pairs) #speed constant in Vs/rad (electrical rad)
        self.R = R
        self.L_q = L_q
        self.L_d = L_d
        self.J = J
        # set up SS matrices
        # _m for mechanical, _e for electrical
        self.A_m = np.array([[0,1],[0,-1*b_viscous/J]])
        self.B_m = np.array([[0],[1/J]])
        self.C_m = np.array([[1,0],[0,1]])
        self.D_m = np.array([[0],[0]])

        self.A_e = np.array([[-1*R/L_d, 0],[0, -1*R/L_q]]) # the zero terms get replaced by the theta_dot terms in simulate
        self.B_e = np.array([[1/L_d, 0],[0, 1/L_q]])
        self.C_e = np.array([[1,0],[0,1]])
        self.D_e = np.array([[0,0],[0,0]])

        self.theta = 0      # mechanical!
        self.theta_dot = 0  # mechanical!
        self.I_d = 0
        self.I_q = 0

    def simulate(self, t, u, x0):
        # t is timesteps [t0, t1, ...]
        # u is [T_load, V_d, V_q]
        # x0 is initial states, [theta, theta_dot, I_d, I_q]
        (self.theta, self.theta_dot, self.I_d, self.I_q) = x0
        time = []
        pos = []
        vel = []
        I_d = []
        I_q = []
        for i in range(len(t)):
            out = self.singleStep(u[1],u[2],u[0])
            self.theta = out[0]
            self.theta_dot = out[1]
            self.I_d = out[2]
            self.I_q = out[3]
            time.append(i*self.dT)
            pos.append(self.theta)
            vel.append(self.theta_dot)
            I_d.append(self.I_d)
            I_q.append(self.I_q)

        return [time,pos,vel,I_d,I_q]

    def singleStep(self, V_d, V_q, T_load):
        # create the discretized SS electrical and mechanical models, valid for this time instance
        theta_e = self.theta * self.pole_pairs
        theta_dot_e = self.theta_dot * self.pole_pairs

        V_q_effective = V_q - theta_dot_e * self.lambda_m

        # make new A electrical matrix with the weird coupled theta_dot terms
        A_e = np.add(self.A_e, np.array([[0, theta_dot_e * self.L_q / self.L_d],[-1*theta_dot_e * self.L_d / self.L_q, 0]]))

        # SS_e is a discretized version of the electrical model, only valid for this time step (theta_dot will change)
        SS_e = signal.cont2discrete((A_e, self.B_e, self.C_e, self.D_e), self.dT)
        Ad_e = SS_e[0] # discretized A matrix for electrical system
        Bd_e = SS_e[1]
        Cd_e = SS_e[2]
        Dd_e = SS_e[3]

        # do the same thing for the mechanical model
        Torque = 3*self.pole_pairs/2 * (self.lambda_m * self.I_q + (self.L_d - self.L_q)*self.I_d*self.I_q) - T_load

        if self.theta_dot == 0 and (-1*self.b_coulomb < Torque < self.b_coulomb):
            Torque = 0

        A_m = np.add(self.A_m, np.array([[0,0], [0, -1*self.b_coulomb/self.J*sign(self.theta_dot)]]))

        SS_m = signal.cont2discrete((A_m, self.B_m, self.C_m, self.D_m),self.dT)
        Ad_m = SS_m[0] # discretized A matrix for mechanical system
        Bd_m = SS_m[1]
        Cd_m = SS_m[2]
        Dd_m = SS_m[3]

        # we now have SS models for the electrical and mechanical systems, valid at this specific time step
        # update states, return as output
        input_e = np.array([[V_d],[V_q_effective]])
        (I_d, I_q) = np.add(np.matmul(Ad_e, np.array([[self.I_d],[self.I_q]])), np.matmul(Bd_e, input_e))
        (theta, theta_dot) = np.add(np.matmul(Ad_m, np.array([[self.theta],[self.theta_dot]])), np.matmul(Bd_m, np.array([[Torque]])))

        return (theta[0], theta_dot[0], I_d[0], I_q[0])

if __name__ == "__main__":
    d5065 = motor(J = 1e-4, b_coulomb = 0, b_viscous = 0.01, R = 0.039, L_q = 1.57e-5, L_d = 1.57e-5, KV = 270, pole_pairs = 7, dT = 1/48000)
    d5065_2 = motor_pmsm_combined(J = 1e-4, b_coulomb= 0, b_viscous = 0.01, R=0.039, L_q = 1.57e-5, L_d=1.57e-5, KV=270, pole_pairs = 7)
    x0 = [0,0,0,0]  # initial state of theta, theta_dot, I_d, I_q
    u = [0,0,1]     # input for simulation as [T_load, V_d, V_q]
    t = [i*1/48000 for i in range(12000)] # half second of runtime at Fs=48kHz

    data = d5065.simulate(t=t, u=u, x0=x0)
    sol = scipy.integrate.solve_ivp(d5065_2.diff_eqs, (0,0.25), t_eval=t, args=(0,1), y0=(0,0,0,0))

    pos = data[1]
    vel = data[2]
    I_d = data[3]
    I_q = data[4]

    pos_ivp = sol.y[0]
    vel_ivp = sol.y[1]
    I_d_ivp = sol.y[2]
    I_q_ivp = sol.y[3]

    fig, axs = plt.subplots(4)

    axs[0].plot(t, pos, linestyle=':', label='homebrew')
    axs[0].plot(t, pos_ivp, linestyle=':', label='solve_ivp')
    axs[0].set_title('pos')
    axs[0].set_ylabel('Theta (eRad)')
    axs[0].legend()
    axs[1].plot(t, vel, linestyle=':')
    axs[1].plot(t, vel_ivp, linestyle=':')
    axs[1].set_title('vel')
    axs[1].set_ylabel('Omega (eRad/s)')
    axs[2].plot(t,I_d, linestyle=':')
    axs[2].plot(t,I_d_ivp, linestyle=':')
    axs[2].set_title('I_d')
    axs[2].set_ylabel('Current (A)')
    axs[3].plot(t,I_q, linestyle=':')
    axs[3].plot(t,I_q_ivp, linestyle=':')
    axs[3].set_title('I_q')
    axs[3].set_ylabel('Current (A)')
    axs[3].set_xlabel('time (s)')

    plt.show()