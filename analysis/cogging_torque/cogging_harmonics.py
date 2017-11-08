
import numpy as np
import matplotlib.pyplot as plt

encoder_cpr = 2400
stator_slots = 12
pole_pairs = 7

N = data.size
fft = np.fft.rfft(data)
freq = np.fft.rfftfreq(N, d=1./encoder_cpr)

harmonics = [0]
harmonics += [(i+1)*stator_slots for i in range(pole_pairs)]
harmonics += [pole_pairs]
harmonics += [(i+1)*2*pole_pairs for i in range(int(stator_slots/4))]


fft_sparse = fft.copy()
indicies = np.arange(fft_sparse.size)
mask = [i not in harmonics for i in indicies]
fft_sparse[mask] = 0.0
interp_data = np.fft.irfft(fft_sparse)

#%%

#plt.figure()
plt.subplot(3, 1, 1)
plt.plot(data, label='raw')
plt.plot(interp_data, label='selected harmonics IFFT')
plt.title('cogging map')
plt.xlabel('counts')
plt.ylabel('A')
plt.legend(loc='best')

#plt.figure()
plt.subplot(3, 1, 2)
plt.stem(freq, np.abs(fft)/N, label='raw')
plt.stem(freq[harmonics], np.abs(fft_sparse[harmonics])/N, markerfmt='ro', label='selected harmonics')
plt.title('cogging map spectrum')
plt.xlabel('cycles/turn')
plt.ylabel('A')
plt.legend(loc='best')

#plt.figure()
plt.subplot(3, 1, 3)
plt.stem(freq, np.abs(fft)/N, label='raw')
plt.stem(freq[harmonics], np.abs(fft_sparse[harmonics])/N, markerfmt='ro', label='selected harmonics')
plt.title('cogging map spectrum')
plt.xlabel('cycles/turn')
plt.ylabel('A')
plt.legend(loc='best')