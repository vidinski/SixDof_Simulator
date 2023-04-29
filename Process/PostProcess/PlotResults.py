
import numpy as np
import matplotlib.pyplot as plt

textFile = open('../../output001.txt','rb')

r = np.genfromtxt(textFile, delimiter='\t', names=True)

figures = []

ind = np.argwhere(r['C1'])[0]
print('index: ',)
print('time: ',r['TIME'][ind])

###########################################################################################
					#POSITION
###########################################################################################
fig = plt.figure(figsize=(16, 9), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
ax = fig.add_subplot(111)
ax.grid(b='on')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (meters)')
ax.plot(r['TIME'],r['POSz'])
figures.append(fig)

fig = plt.figure(figsize=(16, 9), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
ax = fig.add_subplot(111)
ax.grid(b='on')
plt.xlabel('Rgo (meters)')
plt.ylabel('Altitude (meters)')
ax.plot(r['POSx'],r['POSz'])
ax.plot(8.0, 40.0, '*')
figures.append(fig)
plt.legend(('Trajectory','ZEM Waypoint'))

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

fig = plt.figure(figsize=(16, 9), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
ax = fig.add_subplot(111)
ax.grid(b='on')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
ax.plot(r['POSx'][ind],r['POSy'][ind], 'bo')
# ax.set_aspect('equal', 'box')
figures.append(fig)

###########################################################################################
					#VELOCITY
###########################################################################################
saveLn = []
fig = plt.figure(figsize=(16, 9), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
ax = fig.add_subplot(211)
ax.grid(b='on')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
ln, = ax.plot(r['TIME'],r['VELx'])
saveLn.append(ln)
ln, = ax.plot(r['TIME'],r['VELy'])
saveLn.append(ln)
ln, = ax.plot(r['TIME'],r['VELz'])
saveLn.append(ln)
ax.set_ylim(0.0, 100)
plt.legend(('VELx','VELy','VELz'))
#
ax = fig.add_subplot(212)
ax.grid(b='on')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
ln, = ax.plot(r['TIME'],np.sqrt(r['VELx']**2 + r['VELy']**2 + r['VELz']**2))
saveLn.append(ln)
figures.append(fig)
ax.set_ylim(-60, 60)
# plt.legend(('VELx','VELy','VELz'))

#plt.setp(saveLn[0], linestyle='--')
figures.append(fig)

plt.show()
textFile.close()