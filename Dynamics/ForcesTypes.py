import numpy as np
import Kinematics.kinematics as kine

class GravityForce(): 
	def __init__(self, index = 0, 
                           force_direction = np.matrix([[0.0],[0.0],[0.0]])):
            self.force_direction = force_direction


class AppliedForce(): 
        def __init__(self, index = 0, 
                           time = 0.0):
            self.t = time 
            self.position_body = np.matrix([[0.0],
                                            [0.0],
                                            [0.0]])
            self.force_mag = 0.0
            self.dir_body = np.matrix([[0.0], [0.0],[-1.0]])
            self.force_body =  np.matrix([[0.0], [0.0],[0.0]])
            self.torque_body =  np.matrix([[0.0], [0.0],[0.0]])
            self.force_global = np.matrix([[0.0], [0.0],[0.0]])
                                      
        def Update(self, new_time, A):
            self.t = new_time
            self.force_mag = self.t
            self.force_body = self.force_mag*self.dir_body
            self.torque_body = np.matmul(kine.skewsym(self.position_body),self.force_mag*self.dir_body)
            self.force_global = np.matmul(A, self.force_body)
