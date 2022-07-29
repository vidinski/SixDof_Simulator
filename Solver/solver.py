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
    F = np.matrix([[0.0],[0.0],[0.0]]) 
    T = np.matrix([[0.0],[0.0],[0.0]])
    bodies[1].BC_trans(s,p)
    
    for i in range(1,5):
        forces[i].UpdateContact(bodies[1], sd, w)
    
    #F = forces[0].force_global + forces[1].force_global + forces[2].force_global + forces[3].force_global + forces[4].force_global
    #T = forces[0].torque_body + forces[1].torque_body + forces[2].torque_body + forces[3].torque_body + forces[4].torque_body
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

    xdot = np.array(xdot)
    return xdot[0]
  



