#%%
import matplotlib.pyplot as plt
import numpy as np

Rload = 3300
R_25 = 10000
T_25 = 25 + 273.15 #Kelvin
Beta = 3434
Tmin = 0
Tmax = 140

temps = np.linspace(Tmin, Tmax, 1000)
tempsK = temps + 273.15

# https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
r_inf = R_25 * np.exp(-Beta/T_25)
R_temps = r_inf * np.exp(Beta/tempsK)
V = Rload / (Rload + R_temps)

fit = np.polyfit(V, temps, 3)
p1 = np.poly1d(fit)
fit_temps = p1(V)

#%%
print(fit)

plt.plot(V, temps, label='actual')
plt.plot(V, fit_temps, label='fit')
plt.xlabel('normalized voltage')
plt.ylabel('Temp [C]')
plt.legend(loc=0)
plt.show()