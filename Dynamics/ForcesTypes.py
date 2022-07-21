import numpy as np
import Kinematics.kinematics as kine
from Kinematics.body_coordinates import body_coordinates

class ForceBase(): 
    def __init__(self, index = 0, position_on_body = np.matrix([[0.0], [0.0],[0.0]])):
        self.position_on_body = position_on_body #wrt body frame
        self.force_body = np.matrix([[0.0], [0.0],[0.0]]) 
        self.torque_body = np.matrix([[0.0], [0.0],[0.0]])
        self.force_global = np.matrix([[0.0], [0.0],[0.0]])
    def UpdateBasic(self, body):
        self.torque_body = np.matmul(kine.skewsym(self.position_on_body),self.force_body)
        self.force_global = np.matmul(body.A, self.force_body)

class GravityForce(ForceBase): 
	def __init__(self, index = 0, 
                       force_direction = np.matrix([[0.0],[0.0],[0.0]])):             
            self.force_global = force_direction
            self.torque_body = np.matrix([[0.0], [0.0],[0.0]])

class ContactForce(ForceBase): 
        def __init__ (self,index, position_on_body,u_ground, spring_constant, damping):
            super().__init__(index, position_on_body)
            self.u_ground = u_ground 
            self.spring_constant = spring_constant
            self.damping = damping
        def UpdateContact(self, body, vel_body_global,w):
            self.vel_body_global = vel_body_global
            self.cg_global = body.xyz_global_center
            self.w = w   
            self.A = body.A   
            self.loc_body_global = np.matmul(self.A,self.position_on_body)+self.cg_global
            self.vel_pos_global = np.matmul(self.A,np.matmul(kine.skewsym(w),self.position_on_body)) + self.vel_body_global 
            #Project position vector onto ground plane z 
            self.disp = np.matmul(np.transpose(self.loc_body_global), self.u_ground)   
            self.dispdot = np.matmul(np.transpose(self.vel_pos_global), self.u_ground)          
            if(self.disp < 0): 
                self.force_mag = -self.spring_constant*self.disp - self.damping*self.dispdot
                self.force_global = np.matmul(self.u_ground,self.force_mag)
                self.force_body = np.matmul(np.transpose(self.A), self.force_global)
                self.torque_body =  np.matmul(kine.skewsym(self.position_on_body),self.force_body)            
                print(self.force_body)
            else: 
                self.force_global = np.matrix([[0.0], [0.0],[0.0]]) 
                self.force_body = np.matrix([[0.0], [0.0],[0.0]])
                self.torque_body = np.matrix([[0.0], [0.0],[0.0]])
            self.UpdateBasic(body)

# class AppliedForce(): 
#         def __init__(self, index = 0, 
#                            time = 0.0):
#             self.t = time 
#             self.position_body = np.matrix([[0.0], [0.0],[0.0]])
#             self.force_mag = 0.0
#             self.dir_body = np.matrix([[0.0], [0.0],[-1.0]])
#             self.force_body =  np.matrix([[0.0], [0.0],[0.0]])
#             self.torque_body =  np.matrix([[0.0], [0.0],[0.0]])
#             self.force_global = np.matrix([[0.0], [0.0],[0.0]])
                                      
#         def Update(self, new_time, A, pos):
#             self.t = new_time
#             self.position_body = np.matrix([[0.1*self.t], [0.1*self.t],[0.01]])
#             self.force_mag = -10.0*self.t
#             self.force_body = self.force_mag*self.dir_body
#             self.torque_body = np.matmul(kine.skewsym(self.position_body),self.force_mag*self.dir_body)
#             self.force_global = np.matmul(A, self.force_body)
#             self.position_global = np.matmul(A,self.position_body)+pos

# class SpringForce(): 
#         def __init__(self, index, position_body, loc_ground, spring_constant, damping):
#             self.index 
#             self.loc_ground = loc_ground
#             self.position_body = position_body
#             self.spring_constant = spring_constant
#             self.damping = damping
#             self.loc_body_global = np.matrix([[0.0], [0.0],[0.0]])
#             self.loc_ground = np.matrix([[0.0], [0.0],[0.0]])
#             self.force_body =  np.matrix([[0.0], [0.0],[0.0]])
#             self.torque_body =  np.matrix([[0.0], [0.0],[0.0]])
#             self.force_global = np.matrix([[0.0], [0.0],[0.0]])
#         def Update(self, cg_global, A):
#             self.cg_global = cg_global
#             self.loc_body_global = np.matmul(A,position_body)+cg_global
#             #self.force_global = 
#             self.force_body = np.matmul(np.transpose(A), self.force_body)
#             self.torque_body =  np.matmul(kine.skewsym(position_body),force_body) 

# class ContactForce(): 
#         def __init__(self, index, position_body, u_ground, spring_constant, damping):
#             self.index 
#             self.position_body = position_body
#             self.u_ground = u_ground 
#             self.spring_constant = spring_constant
#             self.damping = damping  
#             self.loc_body_global = np.matrix([[0.0], [0.0],[0.0]])
#             self.loc_ground = np.matrix([[0.0], [0.0],[0.0]])
#             self.force_body =  np.matrix([[0.0], [0.0],[0.0]])
#             self.torque_body =  np.matrix([[0.0], [0.0],[0.0]])
#             self.force_global = np.matrix([[0.0], [0.0],[0.0]])
#         def Update(self, cg_global, vel_body_global,A, w,):
#             self.vel_body_global = vel_body_global
#             self.cg_global = cg_global
#             self.loc_ground = loc_ground
#             self.w = w      
#             self.loc_body_global = np.matmul(A,position_body)+cg_global
#             self.vel_pos_global = np.matmul(A,np.matmul(kine.skewsym(w),position_body)) + self.vel_body_global 
#             #Project position vector onto ground plane z 
#             self.disp = np.matmul(np.transpose(self.loc_body_global), self.u_ground)   
#             self.dispdot = np.matmul(np.transpose(self.vel_pos_global), self.u_ground)          
#             if(self.disp < 0): 
#                 self.force_global = -self.spring_constant*self.disp - self.damping*self.dispdot
#                 self.force_body = np.matmul(np.transpose(A), self.force_body)
#                 self.torque_body =  np.matmul(kine.skewsym(position_body),force_body)            
#             else: 
#                 self.force_global = np.matrix([[0.0], [0.0],[0.0]]) 
#                 self.force_body = np.matrix([[0.0], [0.0],[0.0]])
#                 self.torque_body = np.matrix([[0.0], [0.0],[0.0]])
