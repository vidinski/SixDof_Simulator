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

