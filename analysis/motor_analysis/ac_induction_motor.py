
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
from engineering_notation import EngNumber

filename = "oscilloscope.csv"

USE_TEST_DATA = False
PLOT_INITAL = True
DO_FITTING = False
PLOT_PROGRESS = False
REPORT_PROGRESS = True
assumed_rotor_resistance = 1
pole_pairs = 2

class ACMotor():
    """
    Models an induction motor based on Eq 10 in [1].
    [1] https://pdfs.semanticscholar.org/4770/15e472da4c2e05e9ff8c1b921c76a938f786.pdf


    Note: This model refers all rotor quantities to the stator, i.e. the
    quantities are as if the motor had a winding ratio of k = 1.
    """

    # parameters: (name, range)
    parameter_definitions = [
        ('stator_inductance', (0, np.inf), 'H'), # aka l_s, [Henry]
        ('stator_resistance', (0, np.inf), 'ohm'), # aka r_s, [Ohm]
        ('rotor_inductance',  (0, np.inf), 'H'), # aka l_r [Henry]
        # ('rotor_resistance',  (0, np.inf), 'ohm'), # aka r_r [Ohm]
        ('mutual_inductance_factor', (0, 1.0), ''), #[unitless] = l_m**2 / (l_s * l_r)
    ]
    # parameter index lookup
    pl = {r[0]:i for i, r in enumerate(parameter_definitions)}

    # states: (name, initial_value)
    state_definitions = [
        ('stator_current', 0.0), # aka i_s, [A]
        ('rotor_flux', 0.0),     # aka Phi_r, [Wb]
    ] # complex numbers
    # state index lookup
    sl = {r[0]:i for i, r in enumerate(state_definitions)}

    def __init__(self, params):
        self.params = params

        # Assigned in run():
        # self.stator_voltage = None
        # self.omega_stator = None
        # self.omega_rotor = None

    def get_mutual_inductance(self):
        return np.sqrt(
            self.params[ACMotor.pl['mutual_inductance_factor']]
            * self.params[ACMotor.pl['stator_inductance']]
            * self.params[ACMotor.pl['rotor_inductance']]
        )

    def system_function(self, t, y):
        # local shorthand for params
        p = self.params
        pl = ACMotor.pl
        sl = ACMotor.sl

        # rotor_resistance = p[pl['rotor_resistance']] 
        rotor_resistance = assumed_rotor_resistance
        mutual_inductance = self.get_mutual_inductance()

        tau_rotor = p[pl['rotor_inductance']] / rotor_resistance # [s]
        coupling_factor = mutual_inductance / p[pl['rotor_inductance']] # aka k_r [unitless]
        r_sigma = p[pl['stator_resistance']] + coupling_factor**2 * rotor_resistance # [Ohm]
        leakage_factor = 1.0 - mutual_inductance**2 / (p[pl['rotor_inductance']] * p[pl['stator_inductance']]) # aka sigma [unitless]
        tau_stator_prime = leakage_factor * p[pl['stator_inductance']] / r_sigma # [s]

        # [1] Eq 10a
        dstator_current_dt = (
                -1.0j * self.omega_stator * tau_stator_prime * y[sl['stator_current']]
                - coupling_factor / (r_sigma * tau_rotor) * (1.0j*self.omega_rotor * tau_rotor - 1.0) * y[sl['rotor_flux']]
                + 1.0 / r_sigma * self.stator_voltage
                - y[sl['stator_current']]
            ) / tau_stator_prime

        # [1] Eq 10b
        drotor_flux_dt = (
                -1.0j * (self.omega_stator - self.omega_rotor) * tau_rotor * y[sl['rotor_flux']]
                + mutual_inductance * y[sl['stator_current']]
                - y[sl['rotor_flux']]
            ) / tau_rotor

        return [dstator_current_dt, drotor_flux_dt]

    def run(self, time_series, voltage, omega_stator, omega_rotor):
        self.stator_voltage = voltage
        self.omega_stator = omega_stator
        self.omega_rotor = omega_rotor

        y0 = np.array([x[1] for x in ACMotor.state_definitions], dtype=np.complex)

        result = solve_ivp(self.system_function, (time_series[0], time_series[-1]), y0, t_eval=time_series)
        y = result.y

        # compute derived state
        rotor_inductance = self.params[ACMotor.pl['rotor_inductance']]
        rotor_current = (1/rotor_inductance) * (y[1] - self.get_mutual_inductance() * y[0])
        return np.vstack((y, rotor_current))

    def print_parameter_info(self):
        print()
        print('Given parameters:')
        print('pole_pairs = {}'.format(EngNumber(pole_pairs)))
        print('rotor_resistance = {}ohm'.format(EngNumber(assumed_rotor_resistance)))

        print()
        print('Fitted parameters:')
        for i, r in enumerate(ACMotor.parameter_definitions):
            print('{} = {}{}'.format(r[0], EngNumber(self.params[i]), r[2]))

        print()
        print('Derived parameters:')
        mutual_inductance = motor.get_mutual_inductance()
        coupling_factor = mutual_inductance / self.params[ACMotor.pl['rotor_inductance']]
        torque_constant = pole_pairs * coupling_factor * mutual_inductance
        motor_constant = torque_constant / (3.0 * self.params[ACMotor.pl['stator_resistance']])
        print('mutual_inductance = {}H'.format(EngNumber(mutual_inductance)))
        print('coupling_factor = {}'.format(EngNumber(coupling_factor)))
        print('torque_constant = {}Nm/A^2'.format(EngNumber(torque_constant)))
        print('motor_constant = {}Nm/W'.format(EngNumber(motor_constant)))

    def print_run_info(self, y):
        final_stator_current_d = np.real(y[0,-1])
        final_stator_current_q = np.imag(y[0,-1])
        final_rotor_flux_d = np.real(y[1,-1])

        mutual_inductance = self.get_mutual_inductance()
        coupling_factor = mutual_inductance / self.params[ACMotor.pl['rotor_inductance']]
        final_torque_per_q_amp = pole_pairs * coupling_factor * final_rotor_flux_d

        print()
        print('Final values:')
        print('final_rotor_flux_d = {}Wb'.format(EngNumber(final_rotor_flux_d)))
        print('final_stator_current_d = {}A'.format(EngNumber(final_stator_current_d)))
        print('final_stator_current_q = {}A'.format(EngNumber(final_stator_current_q)))
        print('final_torque_per_q_amp = {}Nm/A'.format(EngNumber(final_torque_per_q_amp)))

def plot_data(t, y, ref, title):
    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    ax1b = ax1.twinx()
    ax1.plot(t, ref, label='Measured current')
    ax1.plot(t, np.real(y[0]), label='Stator current (d)')
    ax1.plot(t, np.imag(y[0]), label='Stator current (q)')
    ax2.plot(t, np.real(y[2]), label='Rotor current (d)')
    ax2.plot(t, np.imag(y[2]), label='Rotor current (q)')
    ax1b.plot(t, 1000*np.real(y[1]), 'C3', label='Rotor flux (d)')
    ax1b.plot(t, 1000*np.imag(y[1]), 'C4', label='Rotor flux (q)')
    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('Current [A]')
    ax1b.set_ylabel('Flux [mWb]')
    ax2.set_ylabel('Current [A]')
    plt.title(title)
    fig.legend()
    plt.show()

# load test data
t = np.arange(4096)/8000.0
voltage_step = 1.0
if USE_TEST_DATA:
    with open(filename, 'r') as fp:
        test_response = np.array([float(x) for x in fp.readlines()])
else: test_response = None


inital_parameters = np.zeros(len(ACMotor.parameter_definitions))
inital_parameters[ACMotor.pl['stator_inductance']] = 7.72181086e-04
inital_parameters[ACMotor.pl['stator_resistance']] = 3.06884624e-02
inital_parameters[ACMotor.pl['rotor_inductance']] = assumed_rotor_resistance*6.82013522e-02
# inital_parameters[ACMotor.pl['rotor_resistance']] = 1.0e-0
# inital_parameters[ACMotor.pl['mutual_inductance']] = 2.40e-4
inital_parameters[ACMotor.pl['mutual_inductance_factor']] = 8.68671978e-01

# inital_parameters[ACMotor.pl['stator_resistance']] = 1.298
# inital_parameters[ACMotor.pl['stator_inductance']] = 0.157228647
# inital_parameters[ACMotor.pl['rotor_resistance']] = 0.975052932
# inital_parameters[ACMotor.pl['rotor_inductance']] = 0.16674423623999998
# inital_parameters[ACMotor.pl['mutual_inductance']] = 0.157221177

# Plot initial run
if PLOT_INITAL:
    print()
    print('Initial run:')
    motor = ACMotor(inital_parameters)
    motor.print_parameter_info()

    y = motor.run(
        time_series = t,
        voltage = voltage_step,
        omega_stator = 0,
        omega_rotor= 0)
    motor.print_run_info(y)
    plot_data(t, y, test_response, 'initial')


# Fit to data
def get_residuals(params):
    if REPORT_PROGRESS: print(params)
    motor = ACMotor(params)
    y = motor.run(
        time_series = t,
        voltage = voltage_step,
        omega_stator = 0,
        omega_rotor= 0)
    
    residuals = test_response - np.real(y[0])
    fitness = sum(residuals**2)
    if REPORT_PROGRESS: print(fitness)

    if PLOT_PROGRESS:
        plot_data(t, y, test_response, 'progress')
    
    return residuals

if DO_FITTING:
    print()
    print('Fitting parameters:')
    optiresult = least_squares(get_residuals, inital_parameters,
        bounds=list(zip(*[x[1] for x in ACMotor.parameter_definitions])),
        x_scale='jac',
        diff_step = 1e-2 * np.array([
            7.58192590e-04,
            3.07166671e-02,
            6.85207075e-02,
            # 4.66461518e+00,
            8.67540012e-01])
    )
    print(optiresult.message)

    motor = ACMotor(optiresult.x)
    motor.print_parameter_info()

    y = motor.run(
        time_series = t,
        voltage = voltage_step,
        omega_stator = 0,
        omega_rotor= 0)
    motor.print_run_info(y)
    plot_data(t, y, test_response, 'final')

