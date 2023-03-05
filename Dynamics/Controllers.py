import numpy as np 
from Kinematics.kinematics import skewsym
from Kinematics.kinematics import EulerAng2p
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
                    np.matmul(skewsym(wrB),J),wrB))                     
    #u = -kp*q_error - kv*w_error + J*R(q_error)*w_r + w_rB'*J*w_rB
    #Tcontrol = np.matrix([[0.0], [0.0],[0.0]]) 
    return Tcontrol

def ZEM_ZEV_Controller(body, s, sd, time): 
    # Applications of Generalized Zero-Effort-Miss/Zero-Effort-Velocity
    # Feedback Guidance Algorithm
    
    gravVec = np.matrix([[0.0], [0.0],[solver.g]])

    print('pos: ', s)
    z_0 = s[2]
    zd_0 = sd[2]
    ZEM = np.matrix([[0.0], [0.0],[0.0]])

    tgo = tgoEstimator(time, z_0, zd_0)


    if z_0 > solver.rf[2] + 10 :
        ZEM = solver.rf - (s + sd*tgo); ZEM[2] = 0.0; 
        holdZcmd = np.matrix([[0.0], [0.0],[solver.holdAcmdZ]])
        # acmd = ZEM*6.0/tgo**2 + holdZcmd 
        acmd = ZEM*6.0/tgo**2 - gravVec
        print('ZEM: ', ZEM)
    else: 
        ZEV = solver.vf - sd
        acmd = solver.K_zev * ZEV*2.0/tgo
        solver.holdAcmdZ = acmd[2,0]; 
    
    print('acmd: ', acmd)
    # print("ZEV: ", ZEV)
    print('tgo: ', tgo)
    Fguide = solver.spacecraftMass*np.linalg.norm(acmd)
    Fguide, acmd = limitEngine(Fguide, acmd,z_0)

    # print('Fguide: ', Fguide)
    # print('vel z: ', zd_0)
    ax = np.matmul(np.matrix([[1.0, 0.0, 0.0]]),acmd)
    ay = np.matmul(np.matrix([[0.0, 1.0, 0.0]]),acmd)
    az = np.matmul(np.matrix([[0.0, 0.0, 1.0]]),acmd)
    #axz = np.matrix([[ax],[az], [0.0]])
    axz = ax + az
    axy = ax+ay
    azy = ay+az
    azy_mag = np.linalg.norm(azy)
    axy_mag = np.linalg.norm(axy)
    axz_mag = np.linalg.norm(axz)
    anglez = 0.0
    angley = 0.5*np.pi - np.arctan2(acmd[2],acmd[0])
    anglex = -0.5*np.pi + np.arctan2(axz_mag,acmd[1])
    # print('angley: ',angley)
    # print('anglex: ',anglex)
    p_cmd = EulerAng2p(anglex, angley, anglez)
    return Fguide, p_cmd #, ZEM, ZEV, acmd, tgo

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

    Frcs[0] = -0.5*F123[1] 
    Frcs[1] = 0.5*F123[0]
    Frcs[2] = 0.5*F123[1] 
    Frcs[3] = -0.5*F123[0]
    Frcs[4] =  0.5*F123[2]
    Frcs[5] = -0.5*F123[2]

    return Frcs

def tgoEstimator(time, z_0, zd_0): 
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

    if z_0 < solver.rf[2] :
        tgo = (-zd_0 + np.sqrt(zd_0**2 - 4*0.5*solver.g*z_0))/(2*0.5*solver.g)
        if tgo<0.0: 
            tgo = (-zd_0 - np.sqrt(zd_0**2 - 4*0.5*solver.g*z_0))/(2*0.5*solver.g)
        elif abs(tgo) < 0.1:
            tgo = 0.1
    else: 
        zf = z_0 - solver.rf[2]; 
        tgo = (-zd_0 + np.sqrt(zd_0**2 - 4*0.5*solver.g*zf))/(2*0.5*solver.g)
        if tgo<0.0: 
            tgo = (-zd_0 - np.sqrt(zd_0**2 - 4*0.5*solver.g*zf))/(2*0.5*solver.g)
        elif abs(tgo) < 0.1:
            tgo = 0.1

    # tfinal = 15.0
    # tgo = tfinal - time
    # if tgo < 0.1 :
    #     tgo = 0.1
    return tgo

def limitEngine(Fguide, acmd,z_0): 
        gravVec = np.matrix([[0.0], [0.0],[solver.g]])
        # if (np.linalg.norm(acmd) < abs(solver.g)): #or (zd_0 > -0.3):
        if (np.linalg.norm(acmd) < 0.0):
            # Fguide = -solver.spacecraftMass*solver.g
            # acmd = -gravVec
            acmd = np.matrix([[0.0],[0.0],[1.0]])
            pass
        if(np.linalg.norm(Fguide) > solver.FEngineLim): 
            Fguide = solver.FEngineLim
            acmd = Fguide/solver.spacecraftMass*1/np.linalg.norm(acmd)*acmd
        if(z_0 < 1.0): 
            Fguide = 0.0
            acmd = np.matrix([[0.0],[0.0],[1.0]])
        else:
            pass
        
        return Fguide, acmd

