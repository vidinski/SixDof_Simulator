import numpy as np
#from scipy.integrate import odeint
#from body_coordinates import body_coordinates #, BC_constraints, joints
from Kinematics.body_coordinates import body_coordinates #, skewsym
from Kinematics.kinematics import skewsym
import Dynamics.Controllers as cntrl
import DataLogging.printResults as LogData

global bodies
global forces
global tnb
global attitude_kp
global attitude_kd
global K_zem
global K_zev
global ly
global lx
global FrcsLim
global FEngineLim
global g 
global spacecraftMass
global textFile
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
    thrust = np.transpose(x[0,13:20])
    
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

    # get controller values: 
    Fguide, p_cmd = cntrl.ZEM_ZEV_Controller(bodies[1], s, sd, t) 
    wr = np.matrix([[0.0],[0.0],[0.0]]);  
    Tcontrol = cntrl.AttitudeController(wr,p_cmd,w,p,
                            bodies[1].inertia, 
                            bodies[1].A,
                            attitude_kp, 
                            attitude_kd)  

    Ft = cntrl.RCSMix(Tcontrol)

    #debug 
    #Ft = np.matrix([[0.0],[0.0], [0.0],[0.0], [0.0],[0.0]])
    #T = Tcontrol
    #debug

    nn = 0
    for i in range(5,12):
        forces[i].UpdatePropulsion(bodies[1], thrust[nn,0])
        nn = nn + 1
    
    for frc in forces:
        F = F+frc.force_global
        T = T+frc.torque_body

    #print('Tcntl: ',Tcontrol[1,0], 'Tact: ', T[1,0])
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
    #_______________________________________________________________________
    #thrust force lag
    #_______________________________________________________________________
    thrust_dot1 = np.matrix([[1/forces[5].tau*(-thrust[0,0] + Ft[0,0])]])
    thrust_dot2 = np.matrix([[1/forces[6].tau*(-thrust[1,0] + Ft[1,0])]])    
    thrust_dot3 = np.matrix([[1/forces[7].tau*(-thrust[2,0] + Ft[2,0])]])    
    thrust_dot4 = np.matrix([[1/forces[8].tau*(-thrust[3,0] + Ft[3,0])]])    
    thrust_dot5 = np.matrix([[1/forces[9].tau*(-thrust[4,0] + Ft[4,0])]]) 
    thrust_dot6 = np.matrix([[1/forces[10].tau*(-thrust[5,0] + Ft[5,0])]]) 
    thrust_dot7 = np.matrix([[1/forces[11].tau*(-thrust[6,0] + Fguide)]]) 
    #package them up
    qdoubledot = np.concatenate((np.transpose(sdd),np.transpose(alpha)), axis=1)
    qdot = np.concatenate((np.transpose(sd),np.transpose(pd)), axis=1)
    xdot = np.concatenate((qdot,qdoubledot),axis = 1 ) 
    #add subsystems: 
    xdot = np.concatenate((xdot,thrust_dot1), axis = 1)
    xdot = np.concatenate((xdot,thrust_dot2), axis = 1)
    xdot = np.concatenate((xdot,thrust_dot3), axis = 1)
    xdot = np.concatenate((xdot,thrust_dot4), axis = 1)
    xdot = np.concatenate((xdot,thrust_dot5), axis = 1)
    xdot = np.concatenate((xdot,thrust_dot6), axis = 1)
    xdot = np.concatenate((xdot,thrust_dot7), axis = 1)
    #send array back to solver
    xdot = np.array(xdot)
    #print(thrust_cmd)
    print("acceleration: ",sdd)
    print("time: ",t, " sec")

    #log data
    #pos
    textFile.write(str(t))
    textFile.write('\t')
    LogData.writeToFile(np.transpose(s))
    textFile.write('\t')
    #vel
    LogData.writeToFile(np.transpose(sd))
    textFile.write('\n')

    return xdot[0]
  



