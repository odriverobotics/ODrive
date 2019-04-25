
import numpy as np
from math import pi, sqrt
from scipy.integrate import odeint, complex_ode, solve_ivp
from scipy.optimize import least_squares
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

filenames = [
    "dc_test_blue_motor_2019-04-05.json",
    "blocked_rotor_test_blue_motor_2019-04-05.json"
]

class ACMotor():
    """
    Models an induction motor based on Eq 10 in [1].
    [1] https://pdfs.semanticscholar.org/4770/15e472da4c2e05e9ff8c1b921c76a938f786.pdf


    Note: This model refers all rotor quantities to the stator, i.e. the
    quantities are as if the motor had a winding ratio of k = 1.
    """

    def __init__(self, params):
        self.params = params

        # Assigned in run():
        self.stator_voltage = None
        self.omega_stator = None
        self.omega_rotor = None

    def system_function(self, t, y):
        #rotor_flux, stator_current = y # aka Phi_r, i_s [Wb, A], complex numbers
        rotor_current, stator_current = y

        (
            stator_resistance, # aka r_s, [Ohm]
            residual_stator_inductance, # aka l_s, [Henry]
            rotor_resistance, # aka r_r [Ohm]
            residual_rotor_inductance, # aka l_r [Henry]
            max_magnetization_inductance, # aka l_m [Henry]:
            saturation_current, # [A]:
            saturation_current_slope # [H/A]
        ) = self.params

        current_for_magnetization = np.absolute(stator_current + rotor_current)
        dynamic_magnetization_inductance = max_magnetization_inductance / (1 + np.exp(saturation_current_slope*(current_for_magnetization-saturation_current))) + 0.001
        #import ipdb; ipdb.set_trace()
        stator_inductance = dynamic_magnetization_inductance + residual_stator_inductance
        rotor_inductance = dynamic_magnetization_inductance + residual_rotor_inductance

        leakage_factor = 1 - dynamic_magnetization_inductance**2 / (rotor_inductance * stator_inductance) # aka sigma [unitless]
        coupling_factor = dynamic_magnetization_inductance / rotor_inductance              # aka k_r [unitless]
        r_sigma = stator_resistance + coupling_factor**2 * rotor_resistance   # [Ohm]
        tau_rotor = rotor_inductance / rotor_resistance                            # [s]
        tau_stator_prime = leakage_factor * stator_inductance / r_sigma  # [s]

        if y.size > 2:
            import ipdb; ipdb.set_trace()

        rotor_flux = rotor_current * rotor_inductance

        # [1] Eq 10a
        dstator_current_dt = (
                -1j * self.omega_stator * tau_stator_prime * stator_current
                - coupling_factor / (r_sigma * tau_rotor) * (1j*self.omega_rotor * tau_rotor - 1) * rotor_flux
                + 1 / r_sigma * self.stator_voltage
                - stator_current
            ) / tau_stator_prime

        # [1] Eq 10b
        drotor_flux_dt = (
                -1j * (self.omega_stator - self.omega_rotor) * tau_rotor * rotor_flux
                + dynamic_magnetization_inductance * stator_current
                - rotor_flux
            ) / tau_rotor
        drotor_current_dt = drotor_flux_dt / rotor_inductance

        #return [drotor_flux_dt, dstator_current_dt]
        return [drotor_current_dt, dstator_current_dt]

    def run(self, time_series, voltage, omega_stator, omega_rotor):
        self.stator_voltage = voltage
        self.omega_stator = omega_stator
        self.omega_rotor = omega_rotor

        ys = np.nan * np.ones([2, time_series.size], dtype=np.complex)
        ys[:, 0] = [0.0, 0.0] # initial condition

        if False:
            r = complex_ode(self.system_function)
            r.set_initial_value(ys[:, 0], time_series[0])

            for i, t in enumerate(time_series[1:]):
                if r.successful():
                    ys[:, i + 1] = r.integrate(t)
            return ys
        else:
            result = solve_ivp(self.system_function, (time_series[0], time_series[-1]), ys[:, 0], dense_output=True)
            if not result.success:
                return None
            y_func = interp1d(result.t, result.y)
            return y_func(time_series)

# load test data from disk

realworld_runs = []
for filename in filenames:
    import json
    with open(filename, "r") as fp:
        runs = json.load(fp)
    #realworld_runs += runs[0:2]
    #realworld_runs += runs[2:]
    realworld_runs += runs

for run in realworld_runs:
    num_samples = len(run["timestamps"])
    run["timestamps"] = np.concatenate([np.zeros(1), run["timestamps"]])
    run["omega_stator"] = np.concatenate([np.zeros(1), np.ones(num_samples) * run["omega_stator"]])
    run["omega_rotor"] = np.concatenate([np.zeros(1), np.ones(num_samples) * run["omega_rotor"]])
    run["voltages"] = np.concatenate([np.zeros(1), run["Vq"]]) - 1j * np.concatenate([np.zeros(1), run["Vd"]])
    run["currents"] = np.concatenate([np.zeros(1), run["Iq"]]) - 1j * np.concatenate([np.zeros(1), run["Id"]])



# initial guess
#motor_params = [1.2, 0.14, 3.1, 0.47, 0.25, 3.0]
#motor_params = [1.2922684 , 0.15458073 - 0.29040055, 3.6692156 , 0.56636121 - 0.29040055, 0.29040055, 8.0, 4.0]
#motor_params = [1.2922684 , 0.01, 0.6692156 , 0.01, 0.3, 8.0, 0.5]
motor_params = [1.298, 0.00000747, 0.975052932,  0.00952305924, 0.157221177, 12.7, 0.42]

if len(realworld_runs) > 0 and False:
    def test_params(params):
        motor = ACMotor(params)
        residuals = []
        for run in realworld_runs[:4] + realworld_runs[-7:]:
            selected_values = run["timestamps"] < 2.5
            
            ys = motor.run(
                run["timestamps"][selected_values],
                run["voltages"].mean(), # TODO: support voltage/omega_s/omega_r series, not just one value
                run["omega_stator"].mean(),
                run["omega_rotor"].mean(),
            )

            if not ys is None:            
                deltas = ys[1, :] - run["currents"][selected_values]
                residuals.append(np.real(deltas))
                residuals.append(np.imag(deltas))
            else: # ODE did not converge
                residuals.append([np.infty])

        #print(str(params) + " " + str((np.concatenate(residuals)**2).sum()))
        return np.concatenate(residuals)

    result = least_squares(test_params, motor_params,
        bounds=([0,     0,      0,      0,      0,      0,      0],
                [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]))
    if not result.success:
        print("DID NOT CONVERGE!")
    motor_params = result.x



motor = ACMotor(motor_params)

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()
plt.xlabel('Time (s)')
ax1.set_ylabel('Current [A]')
ax2.set_ylabel('Flux [Wb]')

legends = []
for run in realworld_runs:
    selected_values = run["timestamps"] < 2.1
    
    ts = run["timestamps"][selected_values]
    ys = motor.run(
        ts,
        run["voltages"].mean(), # TODO: support voltage/omega_s/omega_r series, not just one value
        run["omega_stator"].mean(),
        run["omega_rotor"].mean(),
    )

    suffix = " @ $U_s={:.1f}V, \omega_s={:.1f}rad/s$".format(run["voltages"].mean(), run["omega_stator"].mean()).replace("+0.0j", "")

    p = ax1.plot(ts, np.real(ys[1, :]), label = "$I_q$" + suffix)
    p = ax1.plot(ts, np.real(run["currents"][selected_values]), ".", color=p[0].get_color())
    
    #p = ax1.plot(ts, -np.imag(ys[1, :]), label = "$I_d$" + suffix)
    #p = ax1.plot(ts, -np.imag(run["currents"][selected_values]), ".", color=p[0].get_color())

    #ax2.plot(ts, np.absolute(ys[0, :]), label = "$\Phi_r$" + suffix)

fig.legend()
fig.show()


# est. params from using DC dataset:
# [1.23505289, 0.12140287, 2.00802169, 0.55824212, 0.23117988] (old)

# est. params from using blocked rotor dataset:
# [1.56487965, 0.0373343 , 1.95444889, 0.56833555, 0.12249371] (old)

# est. params from using both datasets:
# [1.24881544, 0.14874368, 3.11608887, 0.46870572, 0.25660125] (full dataset)
# [1.30341562, 0.19015128, 2.2655822 , 0.49808841, 0.3001334 ] (4 runs only)
# [1.30344521, 0.19018557, 2.30587854, 0.5071363 , 0.3028763 ]
# [1.3035693 , 0.19003095, 1.34670481, 0.29592189, 0.23126355]
# [1.30347215, 0.19005635, 2.35637425, 0.51777387, 0.30592632]
# [1.30356159, 0.19002855, 2.65765596, 0.58389488, 0.32485096]
#     yes           yes        no           no          no         detectable
