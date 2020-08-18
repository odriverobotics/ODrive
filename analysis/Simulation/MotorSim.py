# this file is for the simulation of a 3-phase synchronous motor

import numpy as np
import scipy as sp
import scipy.signal as signal
import scipy.integrate
import matplotlib.pyplot as plt
import time

def sign(num):
    if num > 0:
        return 1
    elif num < 0:
        return -1
    else:
        return 0

C = np.array([0, 1/5, 3/10, 4/5, 8/9, 1])
A = np.array([
    [0, 0, 0, 0, 0],
    [1/5, 0, 0, 0, 0],
    [3/40, 9/40, 0, 0, 0],
    [44/45, -56/15, 32/9, 0, 0],
    [19372/6561, -25360/2187, 64448/6561, -212/729, 0],
    [9017/3168, -355/33, 46732/5247, 49/176, -5103/18656]
])
B = np.array([35/384, 0, 500/1113, 125/192, -2187/6784, 11/84])

# rk_step from scipy.integrate rk.py
def rk_step(fun, t, y, f, h, A, B, C, K):
    """Perform a single Runge-Kutta step.
    This function computes a prediction of an explicit Runge-Kutta method and
    also estimates the error of a less accurate method.
    Notation for Butcher tableau is as in [1]_.
    Parameters
    ----------
    fun : callable
        Right-hand side of the system.
    t : float
        Current time.
    y : ndarray, shape (n,)
        Current state.
    f : ndarray, shape (n,)
        Current value of the derivative, i.e., ``fun(x, y)``.
    h : float
        Step to use.
    A : ndarray, shape (n_stages, n_stages)
        Coefficients for combining previous RK stages to compute the next
        stage. For explicit methods the coefficients at and above the main
        diagonal are zeros.
    B : ndarray, shape (n_stages,)
        Coefficients for combining RK stages for computing the final
        prediction.
    C : ndarray, shape (n_stages,)
        Coefficients for incrementing time for consecutive RK stages.
        The value for the first stage is always zero.
    K : ndarray, shape (n_stages + 1, n)
        Storage array for putting RK stages here. Stages are stored in rows.
        The last row is a linear combination of the previous rows with
        coefficients
    Returns
    -------
    y_new : ndarray, shape (n,)
        Solution at t + h computed with a higher accuracy.
    f_new : ndarray, shape (n,)
        Derivative ``fun(t + h, y_new)``.
    References
    ----------
    .. [1] E. Hairer, S. P. Norsett G. Wanner, "Solving Ordinary Differential
           Equations I: Nonstiff Problems", Sec. II.4.
    """
    K[0] = f
    for s, (a, c) in enumerate(zip(A[1:], C[1:]), start=1):
        dy = np.dot(K[:s].T, a[:s]) * h
        K[s] = fun(t + c * h, y + dy)

    y_new = y + h * np.dot(K[:-1].T, B)
    f_new = fun(t + h, y_new)

    K[-1] = f_new

    return y_new, f_new


# example params for d5065 motor
# phase_R = 0.039 Ohms
# phase_L = 0.0000157 H
# pole_pairs = 7
# KV = 270
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

def inverter(vbus, timings, current):
    # this function should take the relevant inputs and output voltages in dq reference frame.
    pass

class motor:
    def __init__(self, J, b_coulomb, b_viscous, R, L_q, L_d, KV, pole_pairs, dT):
        self.dT = dT
        self.b_coulomb = b_coulomb
        self.b_viscous = b_viscous
        self.KV = KV
        self.pole_pairs = pole_pairs
        kt = 8.27/KV
        self.lambda_m = 2*kt/(3*pole_pairs) #speed constant in Vs/rad (electrical rad)
        self.R = R
        self.L_q = L_q
        self.L_d = L_d
        self.J = J

        # state variables for motor
        self.theta = 0      # mechanical!
        self.theta_dot = 0  # mechanical!
        self.I_d = 0
        self.I_q = 0

        # K matrix. For integrator?
        # np.empty((self.n_stages + 1, self.n_stages), dtype=self.y.dtype)
        self.K = np.empty((7, 4))

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
            self.single_step_rk(u[2],u[1],u[0])
            time.append(i*self.dT)
            pos.append(self.theta)
            vel.append(self.theta_dot)
            I_d.append(self.I_d)
            I_q.append(self.I_q)

        return [time,pos,vel,I_d,I_q]

    def inputs(self, V_q, V_d, T_load):
        self.V_q = V_q
        self.V_d = V_d
        self.T_load = T_load

    def diff_eqs(self, t, y):
        # inputs are self.V_q, self.V_d, self.T_load
        # state is y, y = [theta, theta_dot, I_d, I_q]
        # set_inputs must be called before this if the inputs have changed.
        theta = y[0]
        theta_dot = y[1]
        I_d = y[2]
        I_q = y[3]

        torque = 3*self.pole_pairs/2 * (self.lambda_m * I_q + (self.L_d - self.L_q)*I_d*I_q) - self.T_load

        if theta_dot == 0 and -1*self.b_coulomb < torque < self.b_coulomb:
            torque = 0

        # theta_dot = theta_dot, no ode here
        theta_ddot = (1/self.J) * (torque - self.b_viscous * theta_dot - self.b_coulomb * sign(theta_dot))
        I_d_dot = self.V_d / self.L_d - self.R / self.L_d * I_d + theta_dot*self.pole_pairs * self.L_q / self.L_d * I_q
        I_q_dot = self.V_q / self.L_q - self.R / self.L_q * I_q - theta_dot*self.pole_pairs * self.L_d / self.L_q * I_d - theta_dot*self.pole_pairs * self.lambda_m / self.L_q

        return np.array([theta_dot, theta_ddot, I_d_dot, I_q_dot])

    def single_step_rk(self, V_q, V_d, T_load):
        # given inputs
        self.inputs(V_q, V_d, T_load)
        x = (d5065.theta, d5065.theta_dot, d5065.I_d, d5065.I_q)
        ((d5065.theta, d5065.theta_dot, d5065.I_d, d5065.I_q), _) = rk_step(d5065.diff_eqs, 0, x, d5065.diff_eqs(0, x), d5065.dT, A, B, C, d5065.K)

if __name__ == "__main__":
    d5065 = motor(J = 1e-4, b_coulomb = 0, b_viscous = 0.01, R = 0.039, L_q = 1.57e-5, L_d = 1.57e-5, KV = 270, pole_pairs = 7, dT = 1/48000)
    x0 = [0,0,0,0]  # initial state of theta, theta_dot, I_d, I_q
    u = [0,0,1]     # input for simulation as [T_load, V_d, V_q]
    t = [i*1/48000 for i in range(12000)] # half second of runtime at Fs=48kHz

    data = d5065.simulate(t=t, u=u, x0=x0)
    dT = 1/48000
    states = []
    pos = []
    vel = []
    I_d = []
    I_q = []
    pos = data[1]
    vel = data[2]
    I_d = data[3]
    I_q = data[4]

    fig, axs = plt.subplots(4)

    axs[0].plot(t, pos)
    axs[0].set_title('pos')
    axs[0].set_ylabel('Theta (eRad)')
    axs[1].plot(t, vel)
    axs[1].set_title('vel')
    axs[1].set_ylabel('Omega (eRad/s)')
    axs[2].plot(t,I_d)
    axs[2].set_title('I_d')
    axs[2].set_ylabel('Current (A)')
    axs[3].plot(t,I_q)
    axs[3].set_title('I_q')
    axs[3].set_ylabel('Current (A)')
    axs[3].set_xlabel('time (s)')

    plt.show()