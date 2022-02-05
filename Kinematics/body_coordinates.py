import numpy as np 


class body_coordinates():
    def __init__(self, 	index = 0, 
                       	xyz_global_center = np.matrix([[0.0],[0.0],[0.0]]), 
	          	xyz_local_shape =np.matrix([0.0,0.0,0.0]), 
			xyz_local_joints = np.matrix([[0.0],[0.0],[0.0]]), 
			xyz_local_unit = np.matrix([[0.0],[0.0],[0.0]]), 
			p = np.matrix([[0.0],[0.0],[0.0],[0.0]]),
                        #p = np.matrix([[1.0],[0.0],[0.0],[0.0]]), 
			mass = 0.0, 
		 	inertia = np.eye(3,3)): 
        "shape points and local joint points are defined with respect to the center of mass" 
        self.index =  index
        self.xyz_global_center = xyz_global_center
        self.p = p 
        #self.p = np.matrix([[1.0],[0.0],[0.0],[0.0]])
        self.xyz_local_shape = xyz_local_shape
        self.xyz_local_joints = xyz_local_joints
        self.xyz_local_unit = xyz_local_unit
        self.mass = np.diag([mass,mass,mass])
        self.inertia = inertia 
        self.Jinv = np.linalg.inv(self.inertia)
        self.Minv = np.linalg.inv(self.mass)

    def BC_trans(self, new_position, new_p):
        self.p = new_p
        self.xyz_global_center = new_position
        self.A = p2A(self.p)
        #self.xyz_global_shape = self.xyz_global_center + np.matmul(self.A,self.xyz_local_shape)
        #self.xyz_global_unit = np.matmul(self.A, self.xyz_local_unit)
        #self.xyz_global_joints = np.matmul(self.A,self.xyz_local_joints)  
    
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
        A = np.matmul(G,np.transpose(L))
        return A #, G, L 


def skewsym(vec): 
	# make skew symmetric matrix from a 3D vector
	skewmat = np.matrix([[0.0,-vec[2,0],vec[1,0]],[vec[2,0],0.0,-vec[0,0]],[-vec[1,0],vec[0,0],0.0]])
	return skewmat
# p = [e0, e]; e0 = cos(phi/2); e = sin(phi/2)*u_rot 
# G = [-e, -etild+e0*I]
# L = [-e,  etild+e0*I]
# A = G*L'
