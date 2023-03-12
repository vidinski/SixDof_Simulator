import numpy as np 

def skewsym(vec):
        #make skew symmetric matrix from a 3D vector
        skewmat = np.matrix([[0.0,-vec[2,0],vec[1,0]],[vec[2,0],0.0,-vec[0,0]],[-vec[1,0],vec[0,0],0.0]])
        return skewmat

def p2A(p):
        e = p[1:4,0]
        e0 = p[0,0]
        etild = skewsym(e)
        G1 = -e
        G2 = -etild+e0*np.eye(3)
        L1 = -e
        L2 = etild+e0*np.eye(3)
        G = np.concatenate((G1,G2),axis=1)
        L = np.concatenate((L1,L2),axis=1)
        A = np.transpose(np.matmul(G,np.transpose(L)))
        return A #, G, L

def EulerAng2p(anglex, angley, anglez):
        p = np.matrix([[1.0],[0.0],[0.0],[0.0]])
        p0 = np.cos(anglex/2.0)*np.cos(angley/2.0)*np.cos(anglez/2.0) + np.sin(anglex/2.0)*np.sin(angley/2.0)*np.sin(anglez/2.0)
        p1 = np.sin(anglex/2.0)*np.cos(angley/2.0)*np.cos(anglez/2.0) + np.cos(anglex/2.0)*np.sin(angley/2.0)*np.sin(anglez/2.0)
        p2 = np.cos(anglex/2.0)*np.sin(angley/2.0)*np.cos(anglez/2.0) + np.sin(anglex/2.0)*np.cos(angley/2.0)*np.sin(anglez/2.0)
        p3 = np.cos(anglex/2.0)*np.cos(angley/2.0)*np.sin(anglez/2.0) + np.sin(anglex/2.0)*np.sin(angley/2.0)*np.cos(anglez/2.0)
        p[0] = p0
        p[1] = p1
        p[2] = p2
        p[3] = p3

        return p

# def pointThisWay(vec): 
#         eulerAng = 0.0; 
#         return eulerAng
