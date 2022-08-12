import numpy as np 

def AttitudeController(wr,qr,w,q, J, kp, kd): 
    #wr -> desired velocity inertial frame
    #qr -> desired orientation quaternion
    q_error = qr - q
    w_error = wr - w
    
    #u = -kp*q_error - kv*w_error + J*R(q_error)*w_r + w_rB'*J*w_rB
    Tcontrol = np.matrix([[0.0], [0.0],[0.0]]) 
    return Tcontrol

def ZEM_ZEV_Controller(time, body, vel): 
    #tgo = tf-t
    #tgo = (-zd_0 + sqrt(zd_0^2 - 4*0.5*g*z_0))/(2*0.5*g)
    # sign of g is embedded in the variable g
    #
    pass
