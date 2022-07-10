import numpy as np
#from scipy.integrate import odeint
#from body_coordinates import body_coordinates #, BC_constraints, joints
from Kinematics.body_coordinates import body_coordinates #, skewsym
from Kinematics.kinematics import skewsym
#import frconbod

global bodies
global forces
#global K
global tnb
#global joint_list
#global ground0true

g = 9.81
PI = np.pi

def solveSys(t,x):
    #_______________________________________________________________________
    x = np.matrix(x)
    #need to figure out a better way to write this out with subsystem states
    #q = np.zeros([tnb,12]); 
    #q = np.matrix(q)   
    #bodies[0].BC_trans(np.matrix([[0.0],[0.0],[0.0]]),np.matrix([[1.0],[0.0],[0.0],[0.0]]))
    #for i in range (1,len(bodies)):
        #j = i-1
        #state = [x, y, z, p0, p1, p2, p3, xd, yd, zd, w1, w2, w3] 
	#state derivative = [xdd, ydd, zdd, alpha1, alpha2, alpha3] 
	#xfromintegrator = [x, y, z, p0, p1, p2, p3, xd, yd ,zd, w1, w2, w3]
        #q[j,:] = np.concatenate((x[0,6*j:6*j+6],x[0,6*(tnb+j):6*(tnb+j)+6]), axis = 1) #normal
	#q[j,:] = np.concatenate((x[0,7*j:7*j+7],x[0,7*(tnb+j):7*(tnb+j)+7]), axis = 1)
        #bodies[i].BC_trans(np.transpose(q[j,0:3]), np.transpose(q[j,3:7]))

    s =  np.transpose(x[0,0:3])
    p =  np.transpose(x[0,3:7])
    sd = np.transpose(x[0,7:10])
    w =  np.transpose(x[0,10:13])   

    #_______________________________________________________________________

    #Joint Constraints Jacobian: 
    #_______________________________________________________________________

   #_______________________________________________________________________
    #solver for acclerations
    #_______________________________________________________________________
    F = np.matrix([[0.0],[0.0],[0.0]]) #forces[0].force_direction + forces[1].force_global #np.matrix([[1.0],[0.0],[0.0]])
    T = np.matrix([[0.0],[0.0],[0.0]]) #forces[1].torque_body
    bodies[1].BC_trans(s,p)
    #forces[1].Update(t,bodies[1].A, bodies[1].xyz_global_center)
    # forces[1].UpdateBasic(bodies[1])
    
    for frc in forces:
        F = F+frc.force_global
        T = T+frc.torque_body

    #_______________________________________________________________________
    #Solve for X DOT: 
    #_______________________________________________________________________
    #velocities:   
    #_______________________________________________________________________
    sd = np.matmul(np.eye(3),sd)
    #_______________________________________________________________________
    #quaternions https://ahrs.readthedocs.io/en/latest/filters/angular.html
    #_______________________________________________________________________     
    #pdot = 0.5*[wtild, w; 0, -wT]*q #quaternion
    #pd = np.matmul(np.eye(4),p)  
    wmt = np.matrix(np.transpose(w))
    wtild = skewsym(w) 
    wm = np.matrix(w)
    #pM1 = np.concatenate((wtild, wm), axis = 1)
    pM2 = np.concatenate((wm, wtild), axis = 1)
    pM1 = np.concatenate(([[0]], -wmt), axis = 1)
    pM = np.concatenate((pM1,pM2), axis = 0)
    pM = 0.5*pM
    pd = np.matmul(pM,p)  
    #_______________________________________________________________________
    #translational accelerations
    #_______________________________________________________________________ 
    sdd = np.matmul(bodies[1].Minv,F) 
    #_______________________________________________________________________
    #rotational acclerations
    #_______________________________________________________________________
    Iw = np.matmul(bodies[1].inertia,np.matrix(w))
    wIw = np.matmul(wtild, Iw)
    alpha = np.matmul(bodies[1].Jinv,(T-wIw))

    #package them up
    qdoubledot = np.concatenate((np.transpose(sdd),np.transpose(alpha)), axis=1)
    qdot = np.concatenate((np.transpose(sd),np.transpose(pd)), axis=1)
    xdot = np.concatenate((qdot,qdoubledot),axis = 1 )

    #_______________________________________________________________________	
    #qdoubledotaccel = np.transpose(qdoubledotaccel)
    #qdoubledotalpha = np.transpose(qdoubledotalpha)
    #qdoubledot = np.concatenate((qdoubledotaccel,qdoubledotalpha), axis=1)
    #extract velocities: 
    #xyth_vel = x[0,tnb*9:tnb*18]
    #xyth_vel = x[0,tnb*6:tnb*12]
    #qdoubledot = np.transpose(qdoubledot)
    #xdot = np.concatenate((xyth_vel,qdoubledot),axis = 1 )  
    #_______________________________________________________________________ 


    xdot = np.array(xdot)
    #print(forces[1].force_global)
    print(s[0,0])
    return xdot[0]

def forces(x,t,bodies):
    return F
  



