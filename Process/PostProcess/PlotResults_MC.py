
import numpy as np
import matplotlib.pyplot as plt
import glob 
from scipy.stats import cumfreq
from mpl_toolkits.axes_grid1 import make_axes_locatable

# outputFilesList = glob.glob("../../output*.txt")
storedOutput = []; 
outputFilesList = glob.glob("Example/Subset/output*.txt")

for fileFor in outputFilesList: 
	textFile = open(fileFor,'rb')
	r = np.genfromtxt(textFile, delimiter='\t', names=True)
	textFile.close()
	storedOutput.append(r)
	print(fileFor)

figures = []

# ind = np.argwhere(storedOutput[1]['C1'])[0]
# # print('index: ',)
# print('time: ',storedOutput[1]['TIME'][ind])

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
finalX = []
finalY = []
finalVX = []
finalVY = []
finalVelZ = []
finalVel = []
finalMiss = []
finalIndex = []

timePOSx = []
timePOSy = []
timePOSz = []
timeVELx = []
timeVELy = [] 
timeVELz = []
timeTIME = [] 

n = 0
## FINAL FOR STATS
for outputData in storedOutput: 	
	if np.size(np.argwhere(outputData['C1'])) > 0 :
		ind = np.argwhere(outputData['C1'])[0] - 10
		finalIndex.append(ind)
		timeTIME.append(outputData['TIME'])
		# timePOSx
		# timePOSy 
		timePOSz.append(outputData['POSz'])
		# timeVELx
		# timeVELy 
		# timeVELz 
		# print(np.size(outputData['TIME']))
		# ln, = ax.plot(outputData['TIME'][0:int(ind)],outputData['POSz'][0:int(ind)])
		# plt.setp(ln, color=(0.75,0.75,0.75))
		finalX.append(outputData['POSx'][ind])
		finalY.append(outputData['POSy'][ind])
		missDist = np.sqrt(outputData['POSx'][ind]**2 + outputData['POSy'][ind]**2)
		finalMiss.append(missDist)
		velFinal = np.sqrt(outputData['VELx'][ind]**2 + outputData['VELy'][ind]**2 + outputData['VELz'][ind]**2)
		finalVel.append(velFinal)
		finalVelZ.append(outputData['VELz'][ind])
		print('z[ind-10] : ', outputData['POSz'][ind])
		n = n +1
print('total runs that made it: ', n)

ax.set_ylim(0, 250)
figures.append(fig)

## Miss Distance X and Y
xs = np.array(finalX) 
ys = np.array(finalY)

fig = plt.figure(figsize=(16,16), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
axscatter = fig.add_subplot(111)

axscatter.grid(b='on')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
ln, = axscatter.plot(xs, ys, 'bo')
axscatter.set_ylim(-75, 75)
axscatter.set_xlim(-75, 75)
circle1 = plt.Circle((0, 0), 25, facecolor='green', alpha = 0.5, edgecolor = 'none', label = 'Requirement Landing Zone')
plt.gca().add_patch(circle1)
axscatter.set_aspect(1.)

figures.append(fig)
divider = make_axes_locatable(axscatter)
axHistx = divider.append_axes("top", 1.2, pad=0.1, sharex=axscatter)
axHisty = divider.append_axes("right", 1.2, pad=0.1, sharey=axscatter)

axHistx.hist(xs, bins = 5, ec = 'blue')
axHisty.hist(ys, bins = 5, orientation="horizontal", ec = 'blue', alpha=0.5)

axHistx.set_title('Miss Distance Bivariate Distribution', fontweight = 'bold')
axHistx.set_ylabel('count')
axHisty.set_xlabel('count')
plt.rcParams["font.weight"] = "bold"

plt.setp(axHistx.get_xticklabels() + axHisty.get_yticklabels(),
         visible=False)

axscatter.legend() 

## Miss Distance, Miss Velocity

res = cumfreq(finalMiss, numbins=25)
x = res.lowerlimit + np.linspace(0, res.binsize*res.cumcount.size,
                                 res.cumcount.size)

fig = plt.figure(figsize=(16, 9), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
ax = fig.add_subplot(111)
ax.grid(b='on')
ax.bar(x, res.cumcount/len(finalMiss), width=res.binsize)
ax.set_title('Cumulative histogram')
ax.set_xlim([x.min(), x.max()])
plt.xlabel('Miss Distance (m)')
plt.ylabel('Probability (%)')

res = cumfreq(finalVelZ, numbins=25)
x = res.lowerlimit + np.linspace(0, res.binsize*res.cumcount.size,
                                 res.cumcount.size)
fig = plt.figure(figsize=(16, 9), dpi=1920/16)
plt.rcParams["font.weight"] = "bold"
plt.rcParams["axes.labelweight"] = "bold"
ax = fig.add_subplot(111)
ax.grid(b='on')
ax.bar(x, res.cumcount/len(finalVelZ), width=res.binsize)
ax.plot([-7.5,-7.5],[0,100], '--k', label = 'Impact Rating') 
ax.set_title('Cumulative histogram')
ax.set_xlim([x.min(), x.max()])
ax.set_ylim([0,1])
plt.xlabel('Final Velocity (m/s)')
plt.ylabel('Probability (%)')
ax.legend()

############## Time Plots #######################
print('size of stored time data: ', timePOSz)

plt.show()




# CDF
# res = cumfreq(finalY, numbins=25)
# x = res.lowerlimit + np.linspace(0, res.binsize*res.cumcount.size,
#                                  res.cumcount.size)

# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# plt.rcParams["font.weight"] = "bold"
# plt.rcParams["axes.labelweight"] = "bold"
# ax = fig.add_subplot(111)
# ax.grid(b='on')
# # ax1.hist(samples, bins=25)
# # ax1.set_title('Histogram')
# ax.bar(x, res.cumcount/len(finalY), width=res.binsize)
# ax.set_title('Cumulative histogram')
# ax.set_xlim([x.min(), x.max()])
# plt.xlabel('Miss Distance Y (m)')
# plt.ylabel('Probability (%)')

#######################################################################################

# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# plt.rcParams["font.weight"] = "bold"
# plt.rcParams["axes.labelweight"] = "bold"
# ax = fig.add_subplot(111)
# ax.grid(b='on')
# plt.xlabel('Rgo (meters)')
# plt.ylabel('Altitude (meters)')
# ax.plot(r['POSx'],r['POSz'])
# ax.plot(8.0, 40.0, '*')
# figures.append(fig)
# plt.legend(('Trajectory','ZEM Waypoint'))

# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# plt.rcParams["font.weight"] = "bold"
# plt.rcParams["axes.labelweight"] = "bold"
# ax = fig.add_subplot(111)
# ax.grid(b='on')
# plt.xlabel('x (m)')
# plt.ylabel('y (m)')
# ax.plot(r['POSx'],r['POSy'], 'bo')
# ax.set_aspect('equal', 'box')
# figures.append(fig)

# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# plt.rcParams["font.weight"] = "bold"
# plt.rcParams["axes.labelweight"] = "bold"
# ax = fig.add_subplot(111)
# ax.grid(b='on')
# plt.xlabel('x (m)')
# plt.ylabel('y (m)')
# ax.plot(r['POSx'][ind],r['POSy'][ind], 'bo')
# # ax.set_aspect('equal', 'box')
# figures.append(fig)

# ###########################################################################################
# 					#VELOCITY
# ###########################################################################################
# saveLn = []
# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# plt.rcParams["font.weight"] = "bold"
# plt.rcParams["axes.labelweight"] = "bold"
# ax = fig.add_subplot(211)
# ax.grid(b='on')
# plt.xlabel('Time (s)')
# plt.ylabel('Velocity (m/s)')
# ln, = ax.plot(r['TIME'],r['VELx'])
# saveLn.append(ln)
# ln, = ax.plot(r['TIME'],r['VELy'])
# saveLn.append(ln)
# ln, = ax.plot(r['TIME'],r['VELz'])
# saveLn.append(ln)
# ax.set_ylim(0.0, 100)
# plt.legend(('VELx','VELy','VELz'))
# #
# ax = fig.add_subplot(212)
# ax.grid(b='on')
# plt.xlabel('Time (s)')
# plt.ylabel('Velocity (m/s)')
# ln, = ax.plot(r['TIME'],np.sqrt(r['VELx']**2 + r['VELy']**2 + r['VELz']**2))
# saveLn.append(ln)
# figures.append(fig)
# ax.set_ylim(-60, 60)
# # plt.legend(('VELx','VELy','VELz'))

# #plt.setp(saveLn[0], linestyle='--')
# figures.append(fig)

# ##
# fig = plt.figure(figsize=(16, 9), dpi=1920/16)
# plt.rcParams["font.weight"] = "bold"
# plt.rcParams["axes.labelweight"] = "bold"
# ax = fig.add_subplot(223)
# ax.grid(b='on')
# plt.xlabel('x (m)')
# plt.ylabel('y (m)')
# ax.plot(finalX, finalY, 'bo')
# ax.set_aspect('equal', 'box')
# figures.append(fig)

# ax = fig.add_subplot(221)
# xs = np.array(finalX); 
# counts, buckets = np.histogram(xs)
# ax.plot(buckets[:-1], counts)

# ax = fig.add_subplot(224)
# ys = np.array(finalY)
# ax.hist(ys, bins = 5, orientation="horizontal", ec = 'blue')
