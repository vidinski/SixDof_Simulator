
import numpy as np
import matplotlib.pyplot as plt

textFile = open('../../output001.txt','rb')

r = np.genfromtxt(textFile, delimiter='\t', names=True)

figures = []

###########################################################################################
					#POSITION
###########################################################################################
# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# ax = fig.add_subplot(111)
# ax.grid(b='on')
# plt.xlabel('Time (s)')
# plt.ylabel('Altitude (meters)')
# ax.plot(r['TIME'],r['POSz'])
# figures.append(fig)

fig = plt.figure(figsize=(16, 9), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
ax = fig.add_subplot(111)
ax.grid(b='on')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
ax.plot(r['POSx'],r['POSy'], 'bo')
ax.set_aspect('equal', 'box')
figures.append(fig)

###########################################################################################
					#VELOCITY
###########################################################################################
# saveLn = []
# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# ax = fig.add_subplot(111)
# ax.grid(b='on')
# plt.xlabel('Time (s)')
# plt.ylabel('Velocity (m/s)')
# ln, = ax.plot(r['TIME'],r['VELx'])
# saveLn.append(ln)
# ln, = ax.plot(r['TIME'],r['VELy'])
# saveLn.append(ln)
# ln, = ax.plot(r['TIME'],r['VELz'])
# saveLn.append(ln)
# figures.append(fig)
# plt.legend(('VELx','VELy','VELz'))
# #plt.setp(saveLn[0], linestyle='--')
plt.show()
textFile.close()