
from matplotlib import pyplot as plt
import sys

with open(sys.argv[1]) as f:
    data = list(map(float, f))

plt.plot(data)
plt.show()