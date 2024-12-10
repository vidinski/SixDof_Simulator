import numpy as np 
import matplotlib
# matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import scipy.integrate as integrate
from Solver import solver 
from Dynamics import ForcesTypes
#import Dynamics
from Kinematics.body_coordinates import body_coordinates
#import Kinematics.body_coordinates #import body_coordinates as body_coordinates #, BC_constraints, joints
#from body_coordinates import body_coordinates #as BC #, BC_constraints, joints
from mpl_toolkits.mplot3d import Axes3D 
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.animation as animation

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D 
import mpl_toolkits.mplot3d.art3d as art3d
import matplotlib.animation as animation
import argparse
#https://stackoverflow.com/questions/3461869/plot-a-plane-based-on-a-normal-vector-and-a-point-in-matlab-or-matplotlib/23006541

PI = np.pi; cos45 = np.cos(PI/4.)

#global
solver.bodies = []
solver.forces = []
solver.tnb = 1 #total number of bodies
solver.attitude_kp = -3000 #-3000 #-3e+4
solver.attitude_kd = -1000 #-1000  #-1e+3
solver.K_zem = 1.0
solver.K_zev = 2.0
solver.FrcsLim = 100 # force limit on rcs
solver.FEngineLim = 2000 #1200 # force limit on  main engine thruster
solver.g = -3.721; # Mars surface acceleration due to gravity 
solver.trueSpacecraftMass = 100.0 # space craft mass
solver.estSpacecraftMass = 100.0 # space craft mass
solver.LatchTGO = False 
solver.rf = np.matrix([[8.0], [0.0],[40.0]]) #desired position above target landing zone before landing
solver.vf = np.matrix([[0.0], [0.0],[-0.75]]) #final desired velocity
solver.holdAcmdZ = 0.001; 
tf = 20.0 #final time
rn = 1 #run number
tauRCS01  = 0.05 
tauRCS02  = 0.05 
tauRCS03  = 0.05 
tauRCS04  = 0.05
tauRCS05  = 0.05 
tauRCS06  = 0.05  
tauEngine = 0.1 
#initial conditions
pos0 = np.matrix([[100.0],[5.0],[200.0]])
vel0 = np.matrix([-20.0, -2.0, -40.0])
w0 = np.matrix([0.0, 0.0, 0.0])
quat0 = np.matrix([[0.965926],[0.0],[0.258819],[0.0]])

###########################################################################################
					#TAKE INPUTS
###########################################################################################

parser = argparse.ArgumentParser()
parser.add_argument("--finalTime", help=" final time", type=float)
parser.add_argument("--run", help="run number", type=int)
parser.add_argument("--tauRCS01", help=" rcs Time Constant 1", type=float)
parser.add_argument("--tauRCS02", help=" rcs Time Constant 2", type=float)
parser.add_argument("--tauRCS03", help=" rcs Time Constant 3", type=float)
parser.add_argument("--tauRCS04", help=" rcs Time Constant 4", type=float)
parser.add_argument("--tauRCS05", help=" rcs Time Constant 5", type=float)
parser.add_argument("--tauRCS06", help=" rcs Time Constant 6", type=float)
parser.add_argument("--tauEngine", help=" Engine Time Constant", type=float)
parser.add_argument("--trueSpacecraftMass", help=" Spacecraft mass", type=float)
parser.add_argument("--pos0x", help=" Position x", type=float)
parser.add_argument("--pos0y", help=" Position y", type=float)
parser.add_argument("--pos0z", help=" Position z", type=float)
parser.add_argument("--w0x", help=" Angular rate x", type=float)
parser.add_argument("--w0y", help=" Angular rate y", type=float)
parser.add_argument("--w0z", help=" Angular rate z", type=float)
parser.add_argument("--quat00", help=" quaternion[0]", type=float)
parser.add_argument("--quat01", help=" quaternion[1]", type=float)
parser.add_argument("--quat02", help=" quaternion[2]", type=float)
parser.add_argument("--quat03", help=" quaternion[3]", type=float)

args = parser.parse_args()
if args.finalTime:
    tf = args.finalTime
if args.run:
    rn = args.run
if args.tauRCS01:
    tauRCS01 = args.tauRCS01
if args.tauRCS02:
    tauRCS02 = args.tauRCS02
if args.tauRCS03:
    tauRCS03 = args.tauRCS03
if args.tauRCS04:
    tauRCS04 = args.tauRCS04
if args.tauRCS05:
    tauRCS05 = args.tauRCS05
if args.tauRCS06:
    tauRCS06 = args.tauRCS06
if args.tauEngine:
    tauEngine = args.tauEngine
if args.trueSpacecraftMass:
    solver.trueSpacecraftMass = args.trueSpacecraftMass
if args.pos0x:
    pos0[0] = args.pos0x
if args.pos0y:
    pos0[1] = args.pos0y
if args.pos0z:
    pos0[2] = args.pos0z
if args.w0x: 
    w0[0] = args.w0x
if args.w0y: 
    w0[1] = args.w0y
if args.w0z: 
    w0[2] = args.w0z
if args.quat00: 
    quat0[0] = args.quat00
if args.quat01:
    quat0[1] = args.quat01
if args.quat02:
    quat0[2] = args.quat02
if args.quat03:
    quat0[3] = args.quat03

###########################################################################################
					#SETUP VARIABLES
###########################################################################################

dt = 0.1
# go = []
# tspan = [0., 10.0]
tspan = [0., tf]
tmr = np.arange(0.0, tspan[1], dt)

###########################################################################################
					#SETUP BODIES
###########################################################################################

####GROUND#####

body0 = body_coordinates(0, #index
			 np.matrix([[0.0],[0.0],[0.0]]), #center
			 np.matrix([[0.0],[0.0],[0.0]]), #shape
			 np.matrix([[0.0],[0.0],[0.0]]), #joints
			 np.matrix([[0.0],[0.0],[0.0]]), #unit vectors
			 np.matrix([[0.0],[0.0],[0.0],[0.0]]), #quaternion (first element scalar)
			 100000.0, #mass
			 inertia = 100.0*np.eye(3,3)) #inertia xx,yy,zz,xy,xz,yz  
#solver.bodies.append(body0)
solver.bodies.append(body0)
#solver.ground0true = 1		
##_______________________________________________________________________________________##


body1 = body_coordinates(0, #index
			 np.matrix([[0.0],[0.0],[0.0]]), #center
			 np.matrix([[0.5, 0.5,0.0],
                       [1.0,1.0,-0.5],
                       [-0.5,0.5,0.0],
                       [-1.0,1.0,-0.5],
                       [-0.5,-0.5,0.0],
                       [-1.0,-1.0,-0.5],
                       [0.5,-0.5,0.0],
                       [1.0,-1.0,-0.5]]), #shape: each pair of row point to point leg
			 np.matrix([[0.0],[0.0],[0.0]]), #joints
			 np.matrix([[0.0],[1.0],[0.0]]), #unit vectors
			 np.matrix([[1.0],[0.0],[0.0],[0.0]]), #quaternion (first element scalar)
			 solver.trueSpacecraftMass, #mass
			 inertia = np.matrix([[10, 1.2,0.5],
                                 [1.2,19,1.5],
                                 [0.5,1.5,25]]))#100.0*np.eye(3,3)) #inertia xx,yy,zz,xy,xz,yz 


body1.BC_trans(pos0,quat0)
#body1.BC_trans(np.matrix([[50.0],[50.0],[100.0]]),np.matrix([[1.0],[0.0],[0.0],[0.0]]))
solver.bodies.append(body1)

###########################################################################################
					#SETUP FORCES
###########################################################################################
#
index = -1
#
#GRAVITY
index = index + 1
force0 = ForcesTypes.GravityForce(0, #index
                              [[0.0],[0.0],[solver.g*solver.trueSpacecraftMass]]) #force direction, mars gravitational acceleration
solver.forces.append(force0)

#CONTACT
index = index + 1
position_on_body = np.transpose(body1.shape[1,0:3])
u_ground = np.matrix([[0.0],[0.0],[1.0]])
spring_constant = 10000.0
damping = 5000.0
#leg1
force1 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)

solver.forces.append(force1)

#leg2
index = index + 1
position_on_body = np.transpose(body1.shape[3,0:3])
force2 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)
solver.forces.append(force2)

#leg3
index = index + 1
position_on_body = np.transpose(body1.shape[5,0:3])
force3 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)
solver.forces.append(force3)

#leg4
index = index + 1
position_on_body = np.transpose(body1.shape[7,0:3])
force4 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)
solver.forces.append(force4)

# THRUST
#rcs 1
index = index + 1
engine_loc_body = np.matrix([[0.5],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[1.0]])
forceLim = solver.FrcsLim
force5 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tauRCS01, forceLim)
solver.forces.append(force5)

#rcs 2
index = index + 1
engine_loc_body = np.matrix([[0.0],[0.5],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[1.0]])
forceLim = solver.FrcsLim
force6 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tauRCS02, forceLim)
solver.forces.append(force6)

#rcs 3 
index = index + 1
engine_loc_body = np.matrix([[-0.5],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[1.0]])
forceLim = solver.FrcsLim
force7 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tauRCS03, forceLim)
solver.forces.append(force7)

#rcs 4
index = index + 1
engine_loc_body = np.matrix([[0.0],[-0.5],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[1.0]])
forceLim = solver.FrcsLim
force8 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tauRCS04, forceLim)
solver.forces.append(force8)

#rcs 5
index = index + 1
engine_loc_body = np.matrix([[0.5],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[1.0],[0.0]])
forceLim = solver.FrcsLim
force9 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tauRCS05, forceLim)
solver.forces.append(force9)

#rcs 6
index = index + 1
engine_loc_body = np.matrix([[-0.5],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[1.0],[0.0]])
forceLim = solver.FrcsLim
force10 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tauRCS06, forceLim)
solver.forces.append(force10)

#Main engine
index = index + 1
engine_loc_body =  np.matrix([[0.0],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[1.0]])
forceLim = solver.FEngineLim
force11 = ForcesTypes.PropulsionForce(index, engine_loc_body, u_thrust, tauEngine,forceLim)
solver.forces.append(force11)

###########################################################################################
					#SETUP JOINTS
###########################################################################################

### Not yet implemented

###########################################################################################
			#SETUP EQUATIONS OF MOTION AND SOLVE 
###########################################################################################

#states: [x, y, z, p0, p1, p2, p3]

x0 = np.concatenate((np.transpose(solver.bodies[1].xyz_global_center), np.transpose(solver.bodies[1].p)), axis = 1)

#concatenate [x, y, z, p0, p1, p2, p3] with [xd, yd ,zd, w1, w2, w3]
x0 = np.concatenate((x0,vel0), axis = 1)
#x0 = np.concatenate((x0,np.zeros([1,3])), axis = 1)
x0 = np.concatenate((x0,w0), axis = 1)
#add subystems initial conditions: 
x0 = np.concatenate((x0,np.zeros([1,7])), axis = 1)
#x0 = np.concatenate((x0,[[0.0]]), axis = 1)

###### DATA LOGING #########
logHead = "TIME\tPOSx\tPOSy\tPOSz\tVELx\tVELy\tVELz\tACCx\tACCy\tACCz"
logHead = logHead + "\tW1\tW2\tW3\tP0\tP1\tP2\tP3\tT1\tT2\tT3\tT4\tT5"
logHead = logHead + "\tT6\tT7\tC1\tC2\tC3\tC4\tACMD1\tACMD2\tACMD3"
# 
logHead = logHead + "\tPCMD0\tPCMD1\tPCMD2\tPCMD3\tZEM1\tZEM2\tZEM3"
logHead = logHead + "\tZEV1\tZEV2\tZEV3\tTGO"
logHead = logHead + '\n'
# solver.textFile = open('test.txt','w')
fileName = 'output00' + str(rn)+ '.txt'
solver.textFile = open(fileName,'w')
solver.textFile.write(logHead)
##### END DATA LOGING #####

x0 = np.array(x0[0]) 
x = integrate.solve_ivp(solver.solveSys, tspan, x0[0], method='RK45',t_eval = tmr)
solver.textFile.close()

###########################################################################################
					#ANIMATION
###########################################################################################

# Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

N = 4 # Meshsize
fps = 100 # frame per sec
frn = np.arange(1, len(np.transpose(x.y))) # frame number length of the animation
xx, yy = np.meshgrid((-.5,.5), (-.5,.5))
z = np.zeros((2,2))
y = np.zeros((N, N))

xxx, yyy = np.meshgrid((0.0,0.0), (0.0,0.0))
zz = np.zeros((2,2))

# plot 
plt.style.use('dark_background')
time_template = 'time = %.1fs'

fig = plt.figure(figsize=(16, 9), dpi=1920/16)
ax = fig.add_subplot(121, projection='3d')
ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(-2, 2)
ax.xaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
ax.yaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
ax.zaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
ax.grid(b='on')
time_text = ax.text(0.05, 0.9, 0.9, '', transform=ax.transAxes)
plt.xlabel('X')
plt.ylabel('Y')
#ax.set_box_aspect([1,1,1])

#fig2 = plt.figure()
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_xlim3d(-2, 2)
ax2.set_ylim3d(-2, 2)
ax2.set_zlim3d(-2, 2)
ax2.xaxis.set_pane_color((0.1, 0.0, 0.0, 1.0))
ax2.yaxis.set_pane_color((0.1, 0.0, 0.0, 1.0))
ax2.zaxis.set_pane_color((0.1, 0.0, 0.0, 1.0))
ax2.grid(b='on')
time_text2 = ax2.text(0.05, 0.9, 0.9, '', transform=ax.transAxes)
plt.xlabel('X')
plt.ylabel('Y')

def getPlaneData(frame_number,xx,yy,z):
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    posi = np.matrix([[0.0],[0.0],[0.0]])
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    #body graphics  
    for j in range( 0,np.size(xx,0)):
          for jj in range( 0,np.size(xx,1)):
              posi[0,0] = xx[j,jj]
              posi[1,0] = yy[j,jj]
              posi[2,0] = z[j, jj]
              Rposi = np.matmul(body1.A, posi)
              xxx[j,jj] = Rposi[0,0]  + body1.xyz_global_center[0,0] 
              yyy[j,jj] = Rposi[1,0]  + body1.xyz_global_center[1,0]
              zz [j, jj] = Rposi[2,0] + body1.xyz_global_center[2,0]

    xn = xxx
    yn = yyy
    zn = zz
    
    return xn, yn, zn

def getPointData(frame_number):
    #force1.Update() 
    fp  = np.matrix([[0.0], [0.0],[0.0]]) 
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    # time_text = ax.text(0.05 + body1.xyz_global_center[0], 
    #                     0.9 + body1.xyz_global_center[1], 
    #                     0.9 + body1.xyz_global_center[2], 
    #                     '', transform=ax.transAxes)

    return fp

def getLegData(frame_number):
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    posi = np.matrix([[0.0],[0.0],[0.0]])
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    ld = np.matmul(body1.A, np.transpose(body1.shape)) + body1.xyz_global_center
    return ld

def getThrustData(frame_number):
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    n = 13
    td = []
    for i in range(5,12):
        thrustn = x_anim[n,0]

        solver.forces[i].UpdatePropulsion(body1, thrustn)

        v_thrustn = -1/solver.forces[i].forceLim*solver.forces[i].force_mag*solver.forces[i].u_propulsion
        #
        td0n = np.matrix([[0.0, 0.0,0.0],
                        [0.0,0.0,0.0]])
        td0n[0,0:3] = np.transpose(np.matmul(body1.A,solver.forces[i].position_on_body) + body1.xyz_global_center )
        td0n[1,0:3] = np.transpose(np.matmul(body1.A,solver.forces[i].position_on_body + v_thrustn) + body1.xyz_global_center)

        td.append(td0n)
        n = n +1
    return td

def getTrajData():
    pass 

def animate(frame_number,y,plot):
    ax.clear()
    ax2.clear()
    

    xn, yn, zn = getPlaneData(frame_number,xx,yy,z)
    fp = getPointData(frame_number)
    ld = getLegData(frame_number)
    #td1, td2, td3, td4, td5 = getThrustData(frame_number,force5,force6,force7,force8, force9)
    td = getThrustData(frame_number)
    td1 = td[0]; td2 = td[1]; td3 = td[2]; td4 = td[3]; td5 = td[4]; td6 = td[5]; td7 = td[6]
    plot[0][0] = ax.plot_surface(xn, yn, zn, color=(0.9,0.9,0.9))
    plot[1][0]= ax.plot_surface(xxg, yyg, zg, color='red', alpha = 0.0)
    plot[2][0] = ax.scatter(fp[0,0],
                            fp[1,0],
                            fp[2,0],
                            marker = "o",
                            c = "white",
                            s = 200)
    plot[3][0] = ax.plot3D([0.0,0.0],[0.0, 0.0],[0.0,0.0],'orange')
    plot[4][0] = ax.plot3D([ld[0,0],ld[0,1]], [ld[1,0],ld[1,1]],[ld[2,0],ld[2,1]],color=(0.9,0.9,0.9),linewidth=3)
    plot[5][0] = ax.plot3D([ld[0,2],ld[0,3]], [ld[1,2],ld[1,3]],[ld[2,2],ld[2,3]],color=(0.9,0.9,0.9),linewidth=3)
    plot[6][0] = ax.plot3D([ld[0,4],ld[0,5]], [ld[1,4],ld[1,5]],[ld[2,4],ld[2,5]],color=(0.9,0.9,0.9),linewidth=3)
    plot[7][0] = ax.plot3D([ld[0,6],ld[0,7]], [ld[1,6],ld[1,7]],[ld[2,6],ld[2,7]],color=(0.9,0.9,0.9),linewidth=3)
    plot[8][0] = ax.plot3D([td1[0,0],td1[1,0]], [td1[0,1],td1[1,1]],[td1[0,2],td1[1,2]],'cyan')
    plot[9][0] = ax.plot3D([td2[0,0],td2[1,0]], [td2[0,1],td2[1,1]],[td2[0,2],td2[1,2]],'cyan')
    plot[10][0] = ax.plot3D([td3[0,0],td3[1,0]], [td3[0,1],td3[1,1]],[td3[0,2],td3[1,2]],'cyan')
    plot[11][0] = ax.plot3D([td4[0,0],td4[1,0]], [td4[0,1],td4[1,1]],[td4[0,2],td4[1,2]],'cyan')
    plot[12][0] = ax.plot3D([td5[0,0],td5[1,0]], [td5[0,1],td5[1,1]],[td5[0,2],td5[1,2]],'cyan')
    plot[13][0] = ax.plot3D([td6[0,0],td6[1,0]], [td6[0,1],td6[1,1]],[td6[0,2],td6[1,2]],'cyan')
    plot[14][0] = ax.plot3D([td7[0,0],td7[1,0]], [td7[0,1],td7[1,1]],[td7[0,2],td7[1,2]],'orange')
    #trajectory plot
    ax2.plot3D(x.y[0,:], x.y[1,:],x.y[2,:], color=(0.5,0.5,0.9))
    ax2.scatter(x.y[0,frame_number], x.y[1,frame_number],x.y[2,frame_number], color='red', s=100)

    #axes stuff 
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim3d(x.y[0,frame_number]-2, x.y[0,frame_number] + 2)
    ax.set_ylim3d(x.y[1,frame_number]-2, x.y[1,frame_number] + 2)
    ax.set_zlim3d(x.y[2,frame_number]-2, x.y[2,frame_number] + 2)
    ax2.set_xlim3d(-100.0,100.0)
    ax2.set_ylim3d(-100.0,100.0)
    ax2.set_zlim3d(0.0, 200)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')

    time_text2 = ax2.text(250, 150, 150, '', transform=ax.transAxes)
    time_text2.set_text(time_template % (frame_number*dt))
    time_text2.color = 'white'
    

###########################################################################################
                #DEFINE GEOMETRIC FEATURES
###########################################################################################

plot = []

#main body surface
plot0 = [ax.plot_surface(xx, yy, z, color=(0.9,0.9,0.9))]
plot.append(plot0)

# ground
lsp = np.linspace(-2.0, 2.0, num=2)
xxg, yyg = np.meshgrid(lsp, lsp)
zg = np.zeros((2,2))
plot1 = [ax.plot_surface(xxg, yyg, zg, color='red', alpha = 0.01)]
plot.append(plot1)

#origin
plot2 = [ax.scatter(0.0,0.0,0.0,
                    marker = "o", 
                    c = "red",
                    s = 200)] 
                 #markeredgecolor = "red",
                 #markerfacecolor = "red")]
plot.append(plot2)

# thrust vector 
forcevec = [0.0,0.0,1.0]; 
plot3 = [ax.plot3D([0.0,0.0],[0.0, 0.0],[0.0,1.0],'orange')]
plot.append(plot3); 

#legs
ld = np.transpose(body1.shape)
# leg1
plot4 = [ax.plot3D([ld[0,0],ld[0,1]], [ld[1,0],ld[1,1]],[ld[2,0],ld[2,1]])]
plot.append(plot4); 
# leg2
plot5 = [ax.plot3D([ld[0,2],ld[0,3]], [ld[1,2],ld[1,3]],[ld[2,2],ld[2,3]])]
plot.append(plot5); 
# leg3
plot6 = [ax.plot3D([ld[0,4],ld[0,5]], [ld[1,4],ld[1,5]],[ld[2,4],ld[2,5]])]
plot.append(plot6);
# leg4
plot7 = [ax.plot3D([ld[0,6],ld[0,7]], [ld[1,6],ld[1,7]],[ld[2,6],ld[2,7]])]
plot.append(plot7);

# thrust 1
#td1,td2,td3,td4,td5 = getThrustData(0,force5,force6,force7,force8,force9)
td = getThrustData(0)
td1 = td[0]; td2 = td[1]; td3 = td[2]; td4 = td[3]; td5 = td[4]; td6 = td[5]; td7 = td[6]
plot8 = [ax.plot3D([td1[0,0],td1[1,0]], [td1[0,1],td1[1,1]],[td1[0,2],td1[1,2]],'cyan')]
plot.append(plot8)

plot9 = [ax.plot3D([td2[0,0],td2[1,0]], [td2[0,1],td2[1,1]],[td2[0,2],td2[1,2]],'cyan')]
plot.append(plot9)

plot10 = [ax.plot3D([td3[0,0],td3[1,0]], [td3[0,1],td3[1,1]],[td3[0,2],td3[1,2]],'cyan')]
plot.append(plot10)

plot11 = [ax.plot3D([td4[0,0],td4[1,0]], [td4[0,1],td4[1,1]],[td4[0,2],td4[1,2]],'cyan')]
plot.append(plot11)

plot12 = [ax.plot3D([td5[0,0],td5[1,0]], [td5[0,1],td5[1,1]],[td5[0,2],td5[1,2]],'cyan')]
plot.append(plot12)

plot13 = [ax.plot3D([td6[0,0],td6[1,0]], [td6[0,1],td6[1,1]],[td6[0,2],td6[1,2]],'cyan')]
plot.append(plot12)

plot13 = [ax.plot3D([td7[0,0],td7[1,0]], [td7[0,1],td7[1,1]],[td7[0,2],td7[1,2]],'orange')]
plot.append(plot12)

plt.rcParams["figure.figsize"] = [14.00, 7.0]
ani = animation.FuncAnimation(fig, animate, frn, fargs=(y,plot), interval = 1000/10) #interval=1000/fps)

plt.show()

# Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

# ani.save('test.mp4', writer=writer)




