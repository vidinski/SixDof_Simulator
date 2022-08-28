import numpy as np 
from Kinematics.kinematics import skewsym

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
    #tgo = tf-t
    #tgo = (-zd_0 + sqrt(zd_0^2 - 4*0.5*g*z_0))/(2*0.5*g)
    #sign of g is embedded in the variable g
    Fguide = 0.0 #np.matrix([[0.0]]),#np.matrix([[0.0], [0.0],[0.0]]) 
    p_cmd = np.matrix([[1.0],[0.0], [0.0],[0.0]])
    return Fguide, p_cmd

def EngineMix(Fguide, Tcontrol): 

    #This might be wrong
    Mix = np.matrix([[0.353553,  0.707,  0.943,  0.707],
                     [0.353553,  0.707, -0.943, -0.707], 
                     [0.353553, -0.707, -0.943,  0.707],
                     [0.353553, -0.707, 0.943,  -0.707]])
    
    Fcmd = np.concatenate((np.matrix(Fguide), Tcontrol), axis = 0)
    print(Fcmd)
    #Ft = np.matmul(Mix, Fcmd)
    Ft = np.matrix([[0.0],[0.0], [0.0],[0.0]])
    return Ft