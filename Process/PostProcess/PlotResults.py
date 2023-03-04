
import numpy as np
import matplotlib.pyplot as plt

r = np.genfromtxt(textFile, delimiter='\t', names=True)
# print(r['POSx'])

fig = plt.figure(figsize=(16, 9), dpi=1920/16)
ax = fig.add_subplot(111)
ax.grid(b='on')
plt.xlabel('time (s)')
plt.ylabel('Altitude (meters)')
ax.plot(r['TIME'],r['POSz'])

plt.show()
textFile.close()