import numpy as np 
# import matplotlib
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
#https://stackoverflow.com/questions/3461869/plot-a-plane-based-on-a-normal-vector-and-a-point-in-matlab-or-matplotlib/23006541

PI = np.pi; cos45 = np.cos(PI/4.)
solver.bodies = []
solver.forces = []
go = []
#Solver.solver.tnb = 1
solver.tnb = 1
dt = 0.1

tspan = [0., 10.0]
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
			 10.0, #mass
			 inertia = 100.0*np.eye(3,3)) #inertia xx,yy,zz,xy,xz,yz 

vel0 = np.matrix([-10.0, 0.0, -10.0])
body1.BC_trans(np.matrix([[100.0],[0.0],[200.0]]),np.matrix([[0.965926],[0.0],[0.258819],[0.0]]))
solver.bodies.append(body1)

###########################################################################################
					#SETUP FORCES
###########################################################################################
#
index = -1
#
index = index + 1
force0 = ForcesTypes.GravityForce(0, #index
                              [[0.0],[0.0],[-9.81]]) #force direction
solver.forces.append(force0)


index = index + 1
position_on_body = np.transpose(body1.shape[1,0:3])
u_ground = np.matrix([[0.0],[0.0],[1.0]])
spring_constant = 1000.0
damping = 500.0

force1 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)

solver.forces.append(force1)
#
index = index + 1
position_on_body = np.transpose(body1.shape[3,0:3])
force2 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)
solver.forces.append(force2)

#
index = index + 1
position_on_body = np.transpose(body1.shape[5,0:3])
force3 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)
solver.forces.append(force3)

#
index = index + 1
position_on_body = np.transpose(body1.shape[7,0:3])
force4 =  ForcesTypes.ContactForce(index, 
                                    position_on_body,
                                    u_ground, 
                                    spring_constant, 
                                    damping)
solver.forces.append(force4)

#
index = index + 1
engine1_loc_body = np.matrix([[0.5],[0.375],[0.0]])
u_thrust1 = np.matrix([[0.0],[-0.25],[-0.25]])
tau_thrust = 0.25
force5 = ForcesTypes.PropulsionForce(index, engine1_loc_body,u_thrust1,tau_thrust)
solver.forces.append(force5)


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
x0 = np.concatenate((x0,np.zeros([1,3])), axis = 1)
#add subystems initial conditions: 
x0 = np.concatenate((x0,np.zeros([1,1])), axis = 1)
#x0 = np.concatenate((x0,[[0.0]]), axis = 1)

x0 = np.array(x0[0]) 
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
#print(matplotlib.__version__)
plt.style.use('dark_background')
time_template = 'time = %.1fs'

fig = plt.figure(figsize=(16, 9), dpi=1920/16)
ax = fig.add_subplot(121, projection='3d')
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
#ax.set_box_aspect([1,1,1])

#fig2 = plt.figure()
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_xlim3d(-2, 2)
ax2.set_ylim3d(-2, 2)
ax2.set_zlim3d(-2, 2)
ax2.w_xaxis.set_pane_color((0.1, 0.0, 0.0, 1.0))
ax2.w_yaxis.set_pane_color((0.1, 0.0, 0.0, 1.0))
ax2.w_zaxis.set_pane_color((0.1, 0.0, 0.0, 1.0))
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
              xxx[j,jj] = Rposi[0,0]  #+body1.xyz_global_center[0,0] 
              yyy[j,jj] = Rposi[1,0]  #+body1.xyz_global_center[1,0]
              zz [j, jj] = Rposi[2,0] #+body1.xyz_global_center[2,0]

    xn = xxx
    yn = yyy
    zn = zz
    
    return xn, yn, zn

def getPointData(frame_number):
    #force1.Update() 
    fp  = np.matrix([[0.0], [0.0],[0.0]]) 
    return fp

def getLegData(frame_number):
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    posi = np.matrix([[0.0],[0.0],[0.0]])
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    ld = np.matmul(body1.A, np.transpose(body1.shape)) #+ body1.xyz_global_center
    return ld

def getThrustData(frame_number, force):
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    thrust = x_anim[13,0]
    force.UpdatePropulsion(body1, thrust)
    v_thrust = -1/5*force.force_mag*force.u_propulsion
    td = np.matrix([[0.0, 0.0,0.0],
          [0.0,0.0,0.0]])
    td[0,0:3] = np.transpose(np.matmul(body1.A,force.position_on_body)) #+ body1.xyz_global_center
    td[1,0:3] = np.transpose(np.matmul(body1.A,force.position_on_body+v_thrust))
    return td

def getTrajData():
    pass 

def animate(frame_number,y,plot):
    ax.clear()
    ax2.clear()
    time_text = ax.text(0.05, 0.9, 0.9, '', transform=ax.transAxes)
    time_text.set_text(time_template % (frame_number*dt))
    time_text.color = 'white'
    xn, yn, zn = getPlaneData(frame_number,xx,yy,z)
    fp = getPointData(frame_number)
    ld = getLegData(frame_number)
    td = getThrustData(frame_number,force5)
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
    plot[8][0] = ax.plot3D([td[0,0],td[1,0]], [td[0,1],td[1,1]],[td[0,2],td[1,2]],'orange')
    ax2.plot3D(x.y[0,:], x.y[1,:],x.y[2,:], color=(0.5,0.5,0.9))
    ax2.scatter(x.y[0,frame_number], x.y[1,frame_number],x.y[2,frame_number], color='red', s=100)
    #axes stuff 
    ax.set_xlim3d(-2, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax2.set_xlim3d(-200, 200)
    ax2.set_ylim3d(-200, 200)
    ax2.set_zlim3d(0.0, 400)
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    

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
td = getThrustData(0, force5)
plot8 = [ax.plot3D([td[0,0],td[1,0]], [td[0,1],td[1,1]],[td[0,2],td[1,2]],'orange')]
plot.append(plot8)

plt.rcParams["figure.figsize"] = [14.00, 7.0]
ani = animation.FuncAnimation(fig, animate, frn, fargs=(y,plot), interval = 1000/10) #interval=1000/fps)

#plt.show()

# Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

# ani.save('test.mp4', writer=writer)




