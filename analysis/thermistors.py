#%%
from odrive.utils import calculate_thermistor_coeffs

Rload = 3300 # 2000 for ODrive v4
R_25 = 10000
Beta = 3434
Tmin = 0
Tmax = 140
calculate_thermistor_coeffs(3, Rload, R_25, Beta, Tmin, Tmax, True)
