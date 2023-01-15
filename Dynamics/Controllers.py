import numpy as np 
from Kinematics.kinematics import skewsym
from Kinematics.kinematics import A2p
from Solver import solver 
from scipy.optimize import fsolve

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

def ZEM_ZEV_Controller(body, s, sd, time): 
    # Applications of Generalized Zero-Effort-Miss/Zero-Effort-Velocity
    # Feedback Guidance Algorithm
    rf = np.matrix([[0.0], [0.0],[0.0]]) #final desired position
    vf = np.matrix([[0.0], [0.0],[0.0]]) #final desired velocity
    z_0 = s[2]
    zd_0 = sd[2]
    # tgo = (-zd_0 + np.sqrt(zd_0**2 - 4*0.5*solver.g*z_0))/(2*0.5*solver.g)
    # if tgo<0.0: 
    #     tgo = (-zd_0 - np.sqrt(zd_0**2 - 4*0.5*solver.g*z_0))/(2*0.5*solver.g)
    # elif abs(tgo) < 0.1:
    #     tgo = 0.1

    gravVec = np.matrix([[0.0], [0.0],[solver.g]])
    # c4 = np.matmul(np.transpose(gravVec),gravVec) 
    # c3 = 0.0
    # c2 = -4.0*np.matmul(np.transpose(sd), sd)
    # c1 = -24.0*np.matmul(np.transpose(s), sd)  
    # c0 = -36.0*np.matmul(np.transpose(s),s)
    # print('c0:', c0)
    # f = lambda t: c4[0,0]*t**4 + c2[0,0]*t**2 + c1[0,0]*t + c0[0,0]
    # tgo2 = fsolve(f, [-100.0, -50.0, 50.0, 100.0]) 
    # tgo = 0.0
    # for tgos in tgo2: 
    #     if tgos>0.0: 
    #         tgo = tgos
    #         break
    # print('tgo2: ', tgo2)
 
    # tmax = -3*z_0/zd_0
    # print('tmax:', tmax)
    # if abs(tgo) > tmax:
    #     tgo = tmax
    tfinal = 15.0
    tgo = tfinal - time
    if tgo < 0.1 :
        tgo = 0.1

    ZEM = rf - (s + sd*tgo + 0.5*gravVec*tgo**2)
    ZEV = vf - (sd + gravVec*tgo)
    acmd = ZEM*6.0/tgo**2 + ZEV*2.0/tgo
    #acmd = ZEV*2.0/tgo
    # print('acmd: ', acmd)
    # print("ZEV: ", ZEV)
    # print('tgo: ', tgo)
    Fguide = solver.spacecraftMass*np.linalg.norm(acmd)
    
    # if (np.matmul([[0.0, 0.0, 1.0]], acmd) < abs(solver.g)): #or (zd_0 > -0.3):
    #     Fguide = -solver.spacecraftMass*solver.g; 
    if(Fguide > solver.FEngineLim): 
        Fguide = solver.FEngineLim
    elif(z_0 < 0.2): 
        Fguide = 0.0
    else:
        pass 
    print('Fguide: ', Fguide)
    print('vel z: ', zd_0)
    ax = np.matmul(np.matrix([[1.0, 0.0, 0.0]]),acmd)
    az = np.matmul(np.matrix([[0.0, 0.0, 1.0]]),acmd)
    axz = np.matrix([[ax],[az], [0.0]])
    angle1 = np.arctan2(ax,az)
    angle2 = np.arctan2(acmd[1],np.linalg.norm(axz))
    
    Ry = np.matrix([[np.cos(angle1), 0.0, np.sin(angle1)],
                    [0.0, 1.0, 0.0],
                    [-np.sin(angle1), 0.0, np.cos(angle1)]])
    Rx = np.matrix([[1.0, 0.0, 0.0],
                    [0.0, np.cos(angle2), -np.sin(angle2)],
                    [0.0, np.sin(angle2),  np.cos(angle2)]])
    R = np.matmul(Ry,Rx)
    print('angle1: ',angle1)
    print('angle2: ',angle2)
    print('R: ', R)
    p_cmd = A2p(np.transpose(R))
    # print('p_cmd', p_cmd)
    # p_cmd = np.matrix([[0.965926],[0.0],[0.258819],[0.0]])
    p_cmd = np.matrix([[1.0],[0.0],[0.0],[0.0]])
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