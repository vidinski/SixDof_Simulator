import numpy as np


class GravityForce(): 
	def __init__(self, index = 0, 
                           force_direction = np.matrix([[0.0],[0.0],[0.0]])):
            self.force_direction = force_direction


class AppliedForce(): 
        def __init__(self, index = 0, 
                           time = 0.0):
            self.t = time 
            self.position_body = np.matrix([[self.t],
                                            [self.t],
                                            [0.0]])
            self.dir_body = np.matrix([[0.0], [0.0],[-1.0*self.t]])
            self.force_mag = self.t
            self.torque_body = np.matmul(skewsym(self.position_body),force_mag*dir_body)
                                      

