import numpy as np 
#import matplotlib
#matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import scipy.integrate as integrate
from Solver import solver 
import Dynamics
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
#https://stackoverflow.com/questions/3461869/plot-a-plane-based-on-a-normal-vector-and-a-point-in-matlab-or-matplotlib/23006541

PI = np.pi; cos45 = np.cos(PI/4.)
solver.bodies = []
go = []
#Solver.solver.tnb = 1
solver.tnb = 1
dt = 0.1

tspan = [0., 5.0]
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
			 np.matrix([[0.0],[0.0],[30.0]]), #center
			 np.matrix([[0.0],[0.0],[30.0]]), #shape
			 np.matrix([[0.0],[0.0],[30.0]]), #joints
			 np.matrix([[0.0],[1.0],[0.0]]), #unit vectors
			 np.matrix([[1.0],[0.0],[0.0],[0.0]]), #quaternion (first element scalar)
			 10.0, #mass
			 inertia = 100.0*np.eye(3,3)) #inertia xx,yy,zz,xy,xz,yz 

#solver.bodies.append(body1)
#print()
body1.BC_trans(np.matrix([[0.0],[0.0],[0.0]]),np.matrix([[1.0],[0.0],[0.0],[0.0]]))
solver.bodies.append(body1)
#print(body1.Mstar)

###########################################################################################
					#SETUP ANIMATION WINDOW
###########################################################################################



###########################################################################################
					#SETUP JOINTS
###########################################################################################



###########################################################################################
			#SETUP EQUATIONS OF MOTION AND SOLVE 
###########################################################################################
#print(np.transpose(solver.bodies[1].xy_global_center))
#print(np.transpose(solver.bodies[1].p))

#states: [x, y, z, p0, p1, p2, p3]

#x0 = np.concatenate((np.transpose(solver.bodies[1].xy_global_center), 
                     #np.transpose(solver.bodies[1].eu_angles)), 
                     #axis = 1)

#x0 = np.concatenate((x0,np.zeros([1,6*solver.tnb])), axis = 1)
#x0 = np.array(x0) 

x0 = np.concatenate((np.transpose(solver.bodies[1].xyz_global_center), np.transpose(solver.bodies[1].p)), axis = 1)

#concatenate [x, y, z, p0, p1, p2, p3] with [xd, yd ,zd, w1, w2, w3]
#x0 = np.concatenate((x0,np.zeros([1,6*solver.tnb])), axis = 1)
x0 = np.concatenate((x0,np.zeros([1,6])), axis = 1)
x0 = np.array(x0) 
#print(solver.bodies[1].p)
x = integrate.solve_ivp(solver.solveSys, tspan, x0[0], method='RK45',t_eval = tmr)



###########################################################################################
					#ANIMATION
###########################################################################################
# Set up formatting for the movie files
Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

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


def getDataFromSim(frame_number,xx,yy,z,body1):
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    posi = np.matrix([[0.0],[0.0],[0.0]])
    #print(x_anim)
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    
    for j in range( 0,np.size(xx,0)):
          for jj in range( 0,np.size(xx,1)):
              posi[0,0] = xx[j,jj]
              posi[1,0] = yy[j,jj]
              posi[2,0] = z[j, jj]
              Rposi = np.matmul(body1.A, posi)
              xxx[j,jj] = Rposi[0,0]
              yyy[j,jj] = Rposi[1,0]
              zz [j, jj] = Rposi[2,0]

    xn = xxx
    yn = yyy
    zn = zz
    #xn = np.cos(0.05*frame_number)*xx-np.sin(0.05*frame_number)*yy
    #yn = np.sin(0.05*frame_number)*xx+np.cos(0.05*frame_number)*yy
    #zn = z

    
    #transGraphics = 
    return xn, yn, zn


def animate(frame_number,y,plot):
    time_text.set_text(time_template % (frame_number*dt))
    plot[0][0].remove()
    #plot[1][0].remove()
    xn, yn, zn = getDataFromSim(frame_number,xx,yy,z,body1)
    #xn = np.cos(0.05*frame_number)*xx-np.sin(0.05*frame_number)*yy
    #yn = np.sin(0.05*frame_number)*xx+np.cos(0.05*frame_number)*yy
    #zn = z
    plot[0][0]= ax.plot_surface(xn, yn, zn, color='magenta')



fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(-2, 2)
ax.set_ylim3d(-2, 2)
ax.set_zlim3d(-2, 2)
ax.w_xaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
ax.w_yaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
ax.w_zaxis.set_pane_color((0.0, 0.0, 0.0, 1.0))
ax.grid(b='on')
time_text = ax.text(0.05, 0.9, 0.9, '', transform=ax.transAxes)
plt.xlabel('X')
plt.ylabel('Y')
#plt.zlabel('Z')


plot = []
plot1 = [ax.plot_surface(xx, yy, z, color='magenta')]; 
plot.append(plot1)
ani = animation.FuncAnimation(fig, animate, frn, fargs=(y,plot), interval = 1) #interval=1000/fps)

plt.show()

"""
# Set up formatting for the movie files
#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

#ani.save('test.mp4', writer=writer)

############################################################################################
fps = 10 # frame per sec
frn = 50 # frame number of the animation
plt.style.use('dark_background')

# crnr = solver.bodies[1].center + np.matmul(solver.bodies[1].A,solver.bodies[1].shape

xx, yy = np.meshgrid((-.5,.5), (-.5,.5)) #Need to plug in body coordinates in here for shape
z = np.zeros((2,2))

print(xx)
print(yy)
"""


