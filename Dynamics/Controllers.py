import numpy as np 
from Kinematics.kinematics import skewsym
from Solver import solver 

def AttitudeController(wr,pr,w,p,J,A,kp,kd): 
    # Partial Lyapunov Strictification: Smooth Angular 
    # Velocity Observers for Attitude Tracking Control
    #wr -> desired velocity inertial frame
    #qr -> desired orientation quaternion
    p_error = pr - p
    w_error = wr - w
    wrB = np.matmul(np.transpose(A),wr)

    Tcontrol = (-kp*p_error[1:5]-kd*w_error+np.matmul(np.matmul(A,J),wr)
                +np.matmul(
                    np.matmul(skewsym(wrB),J),wrB   
                         ))                     
    #u = -kp*q_error - kv*w_error + J*R(q_error)*w_r + w_rB'*J*w_rB
    #Tcontrol = np.matrix([[0.0], [0.0],[0.0]]) 
    return Tcontrol

def ZEM_ZEV_Controller(body, s, sd): 
    # Applications of Generalized Zero-Effort-Miss/Zero-Effort-Velocity
    # Feedback Guidance Algorithm
    rf = np.matrix([[0.0], [0.0],[0.0]]) #final desired position
    vf = np.matrix([[0.0], [0.0],[0.0]]) #final desired velocity
    z_0 = s[2]
    zd_0 = sd[2]
    tgo = (zd_0 - np.sqrt(zd_0**2 - 4*0.5*solver.g*z_0))/(2*0.5*solver.g)
    gravVec = np.matrix([[0.0], [0.0],[solver.g]])
    ZEM = rf - (s + sd*tgo + 0.5*gravVec*tgo**2)
    ZEV = vf - (sd + gravVec*tgo)
    acmd = ZEM*6.0/tgo**2-ZEV*2.0/tgo

    Fguide = solver.spacecraftMass*np.linalg.norm(acmd)
    #p_cmd = np.matrix([[1.0],[0.0], [0.0],[0.0]])
    ay = np.matmul(np.matrix([[0.0, 1.0, 0.0]]),acmd)
    az = np.matmul(np.matrix([[0.0, 0.0, 1.0]]),acmd)
    ayz = np.matrix([[0.0],[ay], [az]])
    angle1 = np.arctan2(ay,az)
    angle2 = np.arctan2(acmd[0],np.linalg.norm(ayz))
    print(np.cos(angle1/2))
    p_cmd = np.matrix([[0.965926],[0.0],[0.258819],[0.0]])
    return Fguide, p_cmd

def RCSMix(Tcontrol): 
    flim = solver.FrcsLim; 

    rcsMix = 0.5*np.eye(3,3)

    rcsMixInv = np.linalg.inv(rcsMix) 
    F123 = np.matmul(rcsMixInv,Tcontrol)
    Frcs = np.zeros((6,1))
    i = 0;    
    for f in F123:
        if np.abs(f) > flim:
            F123[i] = np.sign(f)*flim
        i = i+1

    Frcs[0] = -0.5*F123[1] #about y
    Frcs[1] = 0.5*F123[0]
    Frcs[2] = 0.5*F123[1] #about y
    Frcs[3] = -0.5*F123[0]
    Frcs[4] =  0.5*F123[2]
    Frcs[5] = -0.5*F123[2]
    #print(Frcs)
    #Frcs = np.matrix([[0.0],[0.0], [0.0],[0.0]])
    #print(Frcs)
    return Frcs