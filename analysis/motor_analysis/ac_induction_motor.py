
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
from engineering_notation import EngNumber

filename = "oscilloscope.csv"

PLOT_INITAL = True
PLOT_PROGRESS = False

class ACMotor():
    """
    Models an induction motor based on Eq 10 in [1].
    [1] https://pdfs.semanticscholar.org/4770/15e472da4c2e05e9ff8c1b921c76a938f786.pdf


    Note: This model refers all rotor quantities to the stator, i.e. the
    quantities are as if the motor had a winding ratio of k = 1.
    """

    # parameters: (name, range)
    parameter_definitions = [
        ('stator_inductance', (0, np.inf)), # aka l_s, [Henry]
        ('stator_resistance', (0, np.inf)), # aka r_s, [Ohm]
        ('rotor_inductance',  (0, np.inf)), # aka l_r [Henry]
        # ('rotor_resistance',  (0, np.inf)), # aka r_r [Ohm]
        ('mutual_inductance_factor', (0, 1.0)), #[unitless] = l_m**2 / (l_s * l_r) 
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

    def system_function(self, t, y):
        # local shorthand for params
        p = self.params
        pl = ACMotor.pl
        sl = ACMotor.sl

        # rotor_resistance = p[pl['rotor_resistance']] 
        rotor_resistance = 1.0
        mutual_inductance = np.sqrt(p[pl['mutual_inductance_factor']] * p[pl['stator_inductance']] * p[pl['rotor_inductance']])

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
        return result

def plot_data(t, y, ref, title):
    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()
    ax1.plot(t, ref, label='Measured current')
    ax1.plot(t, y[0], label='Stator current')
    ax2.plot(t, 1000*y[1], 'g', label='Rotor flux')
    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('Current [A]')
    ax2.set_ylabel('Flux [mWb]')
    plt.title(title)
    fig.legend()
    plt.show()


# load test data
t = np.arange(4096)/8000.0
voltage_step = 1.0
with open(filename, 'r') as fp:
    test_response = np.array([float(x) for x in fp.readlines()])


inital_parameters = np.zeros(len(ACMotor.parameter_definitions))
inital_parameters[ACMotor.pl['stator_inductance']] = 6.205440877238289e-05
inital_parameters[ACMotor.pl['stator_resistance']] = 0.031451061367988586
inital_parameters[ACMotor.pl['rotor_inductance']] = 0.25e-0
# inital_parameters[ACMotor.pl['rotor_resistance']] = 1.0e-0
# inital_parameters[ACMotor.pl['mutual_inductance']] = 2.40e-4
inital_parameters[ACMotor.pl['mutual_inductance_factor']] = 0.8

# inital_parameters[ACMotor.pl['stator_resistance']] = 1.298
# inital_parameters[ACMotor.pl['stator_inductance']] = 0.157228647
# inital_parameters[ACMotor.pl['rotor_resistance']] = 0.975052932
# inital_parameters[ACMotor.pl['rotor_inductance']] = 0.16674423623999998
# inital_parameters[ACMotor.pl['mutual_inductance']] = 0.157221177

# Plot initial run
if(PLOT_INITAL):
    motor = ACMotor(inital_parameters)
    result = motor.run(
        time_series = t,
        voltage = voltage_step,
        omega_stator = 0,
        omega_rotor= 0)

    plot_data(t, result.y, test_response, 'initial')


# Fit to data
def get_residuals(params):
    print(params)
    motor = ACMotor(params)
    result = motor.run(
        time_series = t,
        voltage = voltage_step,
        omega_stator = 0,
        omega_rotor= 0)
    
    residuals = test_response - np.real(result.y[0])
    fitness = sum(residuals**2)
    print(fitness)

    if(PLOT_PROGRESS):
        plot_data(t, result.y, test_response, 'progress')
    
    return residuals

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

print()
print('Fitted parameters:')
for i, r in enumerate(ACMotor.parameter_definitions):
    print('{} = {}'.format(r[0], EngNumber(optiresult.x[i])))

print()
print('Derived parameters:')
mutual_inductance = np.sqrt(
    optiresult.x[ACMotor.pl['mutual_inductance_factor']]
    * optiresult.x[ACMotor.pl['stator_inductance']]
    * optiresult.x[ACMotor.pl['rotor_inductance']]
)
coupling_factor = mutual_inductance / optiresult.x[ACMotor.pl['rotor_inductance']]
torque_constant = coupling_factor * mutual_inductance
print('mutual_inductance = {}H'.format(EngNumber(mutual_inductance)))
print('coupling_factor = {}'.format(EngNumber(coupling_factor)))
print('torque_constant = {}Nm/A^2'.format(EngNumber(torque_constant)))

motor = ACMotor(optiresult.x)
result = motor.run(
    time_series = t,
    voltage = voltage_step,
    omega_stator = 0,
    omega_rotor= 0)

final_stator_current = np.real(result.y[0,-1])
final_rotor_flux = np.real(result.y[1,-1])

final_torque_per_amp = coupling_factor * final_rotor_flux

print()
print('Final values:')
print('final_rotor_flux = {}Wb'.format(EngNumber(final_rotor_flux)))
print('final_stator_current = {}A'.format(EngNumber(final_stator_current)))
print('final_torque_per_amp = {}Nm/A'.format(EngNumber(final_torque_per_amp)))

plot_data(t, result.y, test_response, 'final')

