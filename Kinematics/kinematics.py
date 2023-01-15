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

def A2p(A):
        # if (1 + A[0,0] - A[1,1] - A[2,2]) <= 0.0: 
        #         p0 = 1.0
        # else: 
        #         p0 = np.sqrt(1/4*(1 + A[0,0] - A[1,1] - A[2,2]))
        if 0.0 < (1 + A[0,0] - A[1,1] - A[2,2]): 
                p0 = np.sqrt(1/4*(1 + A[0,0] - A[1,1] - A[2,2]))
        else:
                p0 = 1.0

        # p1 = np.sqrt(1/4*(1 - A[0,0] + A[1,1] - A[2,2]))
        # p2 = np.sqrt(1/4*(1 - A[0,0] - A[1,1] + A[2,2]))
        # p3 = np.sqrt(1/4*(1 + A[0,0] + A[1,1] + A[2,2]))
        # print(np.size(p0))
        # print(np.matrix([[p0],[p0],[p0],[p0]]))
        p = np.matrix([[1.0],[0.0],[0.0],[0.0]])
        p[0] = p0
        # if p0 < -0.1 or p0 >-0.01:
        #         p[0] = 1.0
        #         p[1] = 0.0
        #         p[2] = 0.0
        #         p[3] = 0.0
        # else:  
        #         p[1] = (A[0,1] + A[1,0])*1/(4*p0)
        #         p[2] = (A[2,0] + A[0,2])*1/(4*p0)
        #         p[3] = (A[1,2] - A[2,1])*1/(4*p0)
        
        p[1] = (A[0,1] + A[1,0])*1/(4*p0)
        p[2] = (A[2,0] + A[0,2])*1/(4*p0)
        p[3] = (A[1,2] - A[2,1])*1/(4*p0)


        print('p ', p)

        return p