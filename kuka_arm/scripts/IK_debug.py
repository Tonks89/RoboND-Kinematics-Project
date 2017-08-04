from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
        
    ########################################################################################
    ## 

    ## Insert IK code here!
    

    # IK code starts here
	 

    # Define DH param symbols
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0, a1, a2, a3, a4, a5, a6')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1, d2, d3, d4, d5, d6, d7')


    # Joint angle symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1, q2, q3, q4, q5, q6, q7')

    
    # Modified DH params
    DH_params = {alpha0:0, 	a0:0,	     d1:0.75,
		 alpha1:-pi/2,  a1:0.35,     d2:0,      q2:q2-pi/2,
		 alpha2:0,	a2:1.25,     d3:0,
		 alpha3:-pi/2,  a3:-0.054,   d4:1.5,
		 alpha4:pi/2,   a4:0,	     d5:0,
	         alpha5:-pi/2,  a5:0,        d6:0,
		 alpha6:0,	a6:0,        d7:0.303,  q7:0} 



    # Define Modified DH Transformation matrix
         
    def Trans_mat(alphaj, aj, di, qi):
         Tj_i = Matrix([ [       cos(qi),            -sin(qi),                0,                  aj],
			 [sin(qi)*cos(alphaj),  cos(qi)*cos(alphaj),    -sin(alphaj),    -sin(alphaj)*di],  
			 [sin(qi)*sin(alphaj),  cos(qi)*sin(alphaj),    cos(alphaj),     cos(alphaj)*di],
			 [         0,                   0,                   0,                  1]])
         return Tj_i

    
    # Create individual transformation matrices

    # - individual transform matrices about each joint using the DH table 
    T0_1 = Trans_mat(alpha0, a0, d1, q1)
    T0_1 = T0_1.subs(DH_params)

    T1_2 = Trans_mat(alpha1, a1, d2, q2)
    T1_2 = T1_2.subs(DH_params)

    T2_3 = Trans_mat(alpha2, a2, d3, q3)
    T2_3 = T2_3.subs(DH_params)

    T3_4 = Trans_mat(alpha3, a3, d4, q4)
    T3_4 = T3_4.subs(DH_params)

    T4_5 = Trans_mat(alpha4, a4, d5, q5)
    T4_5 = T4_5.subs(DH_params)

    T5_6 = Trans_mat(alpha5, a5, d6, q6)
    T5_6 = T5_6.subs(DH_params)

    T6_7 = Trans_mat(alpha6, a6, d7, q7)
    T6_7 = T6_7.subs(DH_params)

    T_FKM = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
	    
    # Extract end-effector position and orientation from request
    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z
            
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])

	    
    # Gripper w.r.t to base (R0_7 or R0_EE)

    def rotx(angle):

	 T_rotx = Matrix([ [1,        0,          0  ],
                           [0,   cos(angle), -sin(angle)],
	                   [0,   sin(angle),  cos(angle)]])
	 return T_rotx


    def rotz(angle):
	 T_rotz = Matrix([ [cos(angle),  -sin(angle),  0],
			   [sin(angle),   cos(angle),  0],
			   [   0,              0,      1]])
	 return T_rotz


    def roty(angle):
	 T_roty = Matrix([ [cos(angle),   0,  sin(angle)],
		           [     0,       1,      0     ],
			   [-sin(angle),  0,  cos(angle)]])
	 return T_roty



    R0_EE = rotz(yaw) * roty(pitch) * rotx(roll) 		      


    #  Apply correction (urdf frame is different)
    R_corr = rotz(pi) * roty(-pi/2)
    R0_EE = R0_EE * R_corr   # before i had transp., whis supposedly gives same result **!**



    # Calculate joint angles using Geometric IK method
    
    # ................PART 1. Inverse position problem
    nx = R0_EE[0,2]
    ny = R0_EE[1,2]
    nz = R0_EE[2,2]
    wx = px - (d6 + d7) * nx
    wy = py - (d6 + d7) * ny
    wz = pz - (d6 + d7) * nz

    wx = wx.subs(DH_params)
    wy = wy.subs(DH_params)
    wz = wz.subs(DH_params)

    # Joint 1
    theta1 = atan2(wy,wx) 

    # Joint 2
    a = sqrt(d4**2 + a3**2) # virtual link (03 -> WC)
    a = a.subs(DH_params)
    
    s1 = wz - d1.subs(DH_params) 
    s2 = sqrt(wx**2 + wy**2) - a1.subs(DH_params) # right aux. triangle
    b = sqrt(s2**2 + s1**2)

    c = a2.subs(DH_params) # distance joint 2 -> 3

    alp1 = atan2(s1,s2) # aux. right triangle
    A = acos((b**2 + c**2 - a**2)/(2*b*c)) # aux triangle
    B = acos((c**2 + a**2 - b**2)/(2*a*c))
    C = acos((a**2 + b**2 - c**2)/(2*a*b))

    theta2 = pi/2 - A - alp1
    
    # Joint 3
    g = 0.96
    gamma = atan2(a3,g)  
    gamma = gamma.subs(DH_params) # aux triangle 1  

    e = 0.54
    f = sqrt(g**2 + a3**2)
    f = f.subs(DH_params)
    epsil = acos((-e**2 + a**2 + f**2)/(2*a*f)) # aux triangle 2
  
    beta = abs(gamma) - abs(epsil) # aux triangle 3
    
    theta3 = pi/2 - B - beta


    # Joints 1,2,3
    q_123 = {q1: theta1,  q2: theta2, q3: theta3} 

    # ................PART 2. Inverse orientation problem

    T0_3 = T0_1.subs(q_123)  *  T1_2.subs(q_123)  *  T2_3.subs(q_123)
    R0_3 = T0_3[0:3,0:3]
    R3_0 = R0_3.T


    R3_6 = R3_0 * R0_EE 


    theta4 = atan2(R3_6[2,2],-R3_6[0,2])
    theta5 = atan2(sqrt((R3_6[0,2])**2 + (R3_6[2,2])**2), R3_6[1,2])
    theta6 = atan2(-R3_6[1,1],R3_6[1,0])


    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!

    q_all = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6}
    T_FKnum = T_FKM.subs(q_all)

    #  Apply correction (urdf frame is different)
    #T0_7corr = T0_7 * rotz(pi) * roty(-pi/2)

    edf_x = T_FKnum[0,3]
    edf_y = T_FKnum[1,3]
    edf_z = T_FKnum[2,3]

    print("Desired Orientation")
    pprint(R0_EE)
    print("Resulting Orientation (when using IKM results)")
    pprint(T_FKnum)
    print(theta1, theta2, theta3, theta4, theta5, theta6)
    print(type(theta4))
    ## End your code input for forward kinematics here!

    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx,wy,wz] # <--- Load your calculated WC values in this array
    your_ee = [edf_x,edf_y,edf_z] # <--- Load your calculated end effector value from your forward kinematics

    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

test_code(test_cases[test_case_number])
