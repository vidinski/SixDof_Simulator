import numpy as np 
import Kinematics.kinematics as kine

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
        self.A = kine.p2A(self.p)
        #self.xyz_global_shape = self.xyz_global_center + np.matmul(self.A,self.xyz_local_shape)
        #self.xyz_global_unit = np.matmul(self.A, self.xyz_local_unit)
        #self.xyz_global_joints = np.matmul(self.A,self.xyz_local_joints)  
