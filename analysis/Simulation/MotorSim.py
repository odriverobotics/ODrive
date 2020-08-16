# this file is for the simulation of a 3-phase synchronous motor

import numpy as np
import scipy as sp

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
        I_d_dot = V_d / self.L_d - self.R / self.L_d * I_d + theta_dot * self.L_q / self.L_d * I_q
        I_q_dot = V_q / self.L_q - self.R / self.L_q * I_q - theta_dot * self.L_d / self.L_q * I_d - theta_dot * self.lambda_m / self.L_q

        return np.array([theta_dot, theta_ddot, I_d_dot, I_q_dot])

def inverter(vbus, timings, current):
    # this function should take the relevant inputs and output voltages in dq reference frame.
    pass