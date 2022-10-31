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
solver.tnb = 1
solver.attitude_kp = -1.500 #what did I do, this shouldn't be negative
solver.attitude_kd = -0.500 #what did I do, this shouldn't be negative
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
			 100.0, #mass
			 inertia = np.matrix([[10, 1.2,0.5],
                                 [1.2,19,1.5],
                                 [0.5,1.5,25]]))#100.0*np.eye(3,3)) #inertia xx,yy,zz,xy,xz,yz 

vel0 = np.matrix([-10.0, 0.0, -10.0])
w0 = np.matrix([1.0, 10.0, 10.0])
# body1.BC_trans(np.matrix([[100.0],[0.0],[200.0]]),np.matrix([[0.965926],[0.0],[0.258819],[0.0]]))
body1.BC_trans(np.matrix([[100.0],[0.0],[200.0]]),np.matrix([[1.0],[0.0],[0.0],[0.0]]))
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
                              [[0.0],[0.0],[-3.721]]) #force direction, mars gravitational acceleration
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
u_thrust = np.matrix([[0.0],[0.0],[-1.0]])
tau_thrust = 0.01
force5 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tau_thrust)
solver.forces.append(force5)

#rcs 2
index = index + 1
engine_loc_body = np.matrix([[0.0],[0.5],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[-1.0]])
force6 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tau_thrust)
solver.forces.append(force6)

#rcs 3 
index = index + 1
engine_loc_body = np.matrix([[-0.5],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[-1.0]])
force7 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tau_thrust)
solver.forces.append(force7)

#rcs 4
index = index + 1
engine_loc_body = np.matrix([[0.0],[-0.5],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[-1.0]])
force8 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tau_thrust)
solver.forces.append(force8)

#rcs 5
index = index + 1
engine_loc_body = np.matrix([[-0.5],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[-1.0],[0.0]])
force9 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tau_thrust)
solver.forces.append(force9)

#rcs 6
index = index + 1
engine_loc_body = np.matrix([[0.5],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[1.0],[0.0]])
force10 = ForcesTypes.PropulsionForce(index, engine_loc_body,u_thrust,tau_thrust)
solver.forces.append(force10)

#Main engine
index = index + 1
engine_loc_body =  np.matrix([[0.0],[0.0],[0.0]])
u_thrust = np.matrix([[0.0],[0.0],[-1.0]])
tau_thrust = 0.1
force11 = ForcesTypes.PropulsionForce(index, engine_loc_body, u_thrust, tau_thrust)
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

def getThrustData(frame_number):
    x_anim =  np.transpose(np.matrix(x.y[:,frame_number]))
    body1.BC_trans(x_anim[0:3,0],
                    x_anim[3:7,0])
    n = 13
    td = []
    for i in range(5,10):
        thrustn = x_anim[n,0]
        # thrust2 = x_anim[14,0]
        # thrust3 = x_anim[15,0]
        # thrust4 = x_anim[16,0]
        # thrust5 = x_anim[17,0]

        solver.forces[i].UpdatePropulsion(body1, thrustn)
        # force02.UpdatePropulsion(body1, thrust2)
        # force03.UpdatePropulsion(body1, thrust3)
        # force04.UpdatePropulsion(body1, thrust4)
        # force05.UpdatePropulsion(body1, thrust5)

        v_thrustn = -1/50*solver.forces[i].force_mag*solver.forces[i].u_propulsion
        # v_thrust2 = -1/50*force02.force_mag*force02.u_propulsion
        # v_thrust3 = -1/50*force03.force_mag*force03.u_propulsion
        # v_thrust4 = -1/50*force04.force_mag*force04.u_propulsion
        # v_thrust5 = -1/50*force05.force_mag*force05.u_propulsion
        #
        td0n = np.matrix([[0.0, 0.0,0.0],
                        [0.0,0.0,0.0]])
        td0n[0,0:3] = np.transpose(np.matmul(body1.A,solver.forces[i].position_on_body)) #+ body1.xyz_global_center
        td0n[1,0:3] = np.transpose(np.matmul(body1.A,solver.forces[i].position_on_body+v_thrustn))
        #
        # td02 = np.matrix([[0.0, 0.0,0.0],
        #                 [0.0,0.0,0.0]])
        # td02[0,0:3] = np.transpose(np.matmul(body1.A,force02.position_on_body)) #+ body1.xyz_global_center
        # td02[1,0:3] = np.transpose(np.matmul(body1.A,force02.position_on_body+v_thrust2))
        # #
        # td03 = np.matrix([[0.0, 0.0,0.0],
        #                   [0.0,0.0,0.0]])
        # td03[0,0:3] = np.transpose(np.matmul(body1.A,force03.position_on_body)) #+ body1.xyz_global_center
        # td03[1,0:3] = np.transpose(np.matmul(body1.A,force03.position_on_body+v_thrust3))
        # #
        # td04 = np.matrix([[0.0, 0.0,0.0],
        #                   [0.0,0.0,0.0]])
        # td04[0,0:3] = np.transpose(np.matmul(body1.A,force04.position_on_body)) #+ body1.xyz_global_center
        # td04[1,0:3] = np.transpose(np.matmul(body1.A,force04.position_on_body+v_thrust4))
        # #
        # td05 = np.matrix([[0.0, 0.0,0.0],
        #                   [0.0,0.0,0.0]])
        # td05[0,0:3] = np.transpose(np.matmul(body1.A,force05.position_on_body)) #+ body1.xyz_global_center
        # td05[1,0:3] = np.transpose(np.matmul(body1.A,force05.position_on_body+v_thrust5))
        td.append(td0n)
        n = n +1
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
    #td1, td2, td3, td4, td5 = getThrustData(frame_number,force5,force6,force7,force8, force9)
    td = getThrustData(frame_number)
    td1 = td[0]; td2 = td[1]; td3 = td[2]; td4 = td[3]; td5 = td[4]; 
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
    plot[12][0] = ax.plot3D([td5[0,0],td5[1,0]], [td5[0,1],td5[1,1]],[td5[0,2],td5[1,2]],'orange')
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
#td1,td2,td3,td4,td5 = getThrustData(0,force5,force6,force7,force8,force9)
td = getThrustData(0)
td1 = td[0]; td2 = td[1]; td3 = td[2]; td4 = td[3]; td5 = td[4]; 
plot8 = [ax.plot3D([td1[0,0],td1[1,0]], [td1[0,1],td1[1,1]],[td1[0,2],td1[1,2]],'cyan')]
plot.append(plot8)

plot9 = [ax.plot3D([td2[0,0],td2[1,0]], [td2[0,1],td2[1,1]],[td2[0,2],td2[1,2]],'cyan')]
plot.append(plot9)

plot10 = [ax.plot3D([td3[0,0],td3[1,0]], [td3[0,1],td3[1,1]],[td3[0,2],td3[1,2]],'cyan')]
plot.append(plot10)

plot11 = [ax.plot3D([td4[0,0],td4[1,0]], [td4[0,1],td4[1,1]],[td4[0,2],td4[1,2]],'cyan')]
plot.append(plot11)

plot12 = [ax.plot3D([td5[0,0],td5[1,0]], [td5[0,1],td5[1,1]],[td5[0,2],td5[1,2]],'orange')]
plot.append(plot12)

plt.rcParams["figure.figsize"] = [14.00, 7.0]
ani = animation.FuncAnimation(fig, animate, frn, fargs=(y,plot), interval = 1000/10) #interval=1000/fps)

plt.show()

# Set up formatting for the movie files
# Writer = animation.writers['ffmpeg']
# writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

# ani.save('test.mp4', writer=writer)




