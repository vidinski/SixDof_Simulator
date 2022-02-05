###########################################################################################
					#SETUP BODIES
###########################################################################################

####GROUND#####
body0 = body_coordinates(0, #index
			 np.matrix([[0.0],[0.0],[0.0]]), #center
			 np.matrix([[0.0],[0.0],[0.0]]), #shape
			 np.matrix([[0.0],[0.0],[0.0]]), #joints
			 np.matrix([[0.0],[0.0],[0.0]]), #unit vectors
			 np.matrix([[0.0],[0.0],[0.0]]), #euler angle transformation 123
			 100000.0, #mass
			 inertia = np.eye(3,3)) #inertia xx,yy,zz,xy,xz,yz  
solver.bodies.append(body0)
solver.ground0true = 1		
##_______________________________________________________________________________________##

body1 = body_coordinates(1, #index
			 np.matrix([[0.0],[0.0],[0.0]]), #center
			 np.matrix([[0.0],[0.0],[0.0]]), #shape
			 np.matrix([[0.0],[0.0],[0.0]]), #joints
			 np.matrix([[0.0],[0.0],[0.0]]), #unit vectors
			 np.matrix([[0.0],[0.0],[0.0]]), #euler angle transformation 123
 			 np.matrix([[1.0],[0.0],[0.0],[0.0]]), #quaternion
			 1.0, #mass
			 np.matrix([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])) #inertia
