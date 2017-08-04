#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import * # not sure if I have to import from sympy.matrices import Matrix
import numpy as np
from numpy import array
#from numpy import arctan2



def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # ------------ FORWARD KINEMATICS CODE --------------
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

        # Pure rotation matrices for future computations
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


        # Initialize service response
        joint_trajectory_list = []
 
        theta4_prev = 0
        theta6_prev = 0
	theta5_prev = 0

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	 
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
 	    R0_EE = rotz(yaw) * roty(pitch) * rotx(roll) 		      


            #  Apply correction (urdf frame is different)
            R_corr = rotz(pi) * roty(-pi/2)
            R0_EE = R0_EE * R_corr   # before i had transp., whis supposedly gives same result **!**



            # Calculate joint angles using Geometric IK method
    
            d_6 = DH_params[d6]
            d_7 = DH_params[d7]
            d_4 = DH_params[d4]
            a_3 = DH_params[a3]
            a_1 = DH_params[a1]
            d_1 = DH_params[d1]
            a_2 = DH_params[a2]
            g = 0.96
            e = 0.54

    	    # --------- 1. Inverse position problem -----------
    	    nx = R0_EE[0,2]
            ny = R0_EE[1,2]
            nz = R0_EE[2,2]
            wx = px - (d_6 + d_7) * nx
            wy = py - (d_6 + d_7) * ny
            wz = pz - (d_6 + d_7) * nz


            # Joint 1
            theta1 = (atan2(wy,wx)).evalf()

            # Joint 2
            a = sqrt(d_4**2 + a_3**2) # virtual link (03 -> WC)
  
            s1 = wz - d_1 
            s2 = sqrt(wx**2 + wy**2) - a_1 # right aux. triangle
            b = sqrt(s2**2 + s1**2)

            c = a_2 # distance joint 2 -> 3

            alp1 = atan2(s1,s2) # aux. right triangle
            A = acos((b**2 + c**2 - a**2)/(2*b*c)) # aux triangle
            B = acos((c**2 + a**2 - b**2)/(2*a*c))
            C = acos((a**2 + b**2 - c**2)/(2*a*b))

            theta2 = (pi/2 - A - alp1).evalf()
    
            # Joint 3
            gamma = atan2(a_3,g)  # aux triangle 1  

            f = sqrt(g**2 + a_3**2)
            epsil = acos((-e**2 + a**2 + f**2)/(2*a*f)) # aux triangle 2
  
            beta = abs(gamma) - abs(epsil) # aux triangle 3
    
            theta3 = (pi/2 - B - beta).evalf()


            # Joints 1,2,3
            q_123 = {q1: theta1,  q2: theta2, q3: theta3} 


            # ----------- 2. Inverse orientation problem -------------------
            T0_3 = T0_1.evalf(subs = q_123)  *  T1_2.evalf(subs = q_123)  *  T2_3.evalf(subs = q_123)
            R0_3 = T0_3[0:3,0:3]
            R3_0 = R0_3.T


            R3_6 = R3_0 * R0_EE 


            theta4 = (atan2(R3_6[2,2],-R3_6[0,2])).evalf()
            theta5 = (atan2(sqrt((R3_6[0,2])**2 + (R3_6[2,2])**2), R3_6[1,2])).evalf()
            theta6 = (atan2(-R3_6[1,1],R3_6[1,0])).evalf()

            # -------------------- Special cases --------------------------
            if sin(theta5) < 0:
             	theta4 = (atan2(-R3_6[2,2],R3_6[0,2])).evalf()
             	theta6 = (atan2(R3_6[1,1],-R3_6[1,0])).evalf()
                print("-------------> S5  0 ")
            if (theta5 == 0): 
               	theta4 = theta4_prev # keep q4 current value
                theta46 = atan2(-R3_6[0,1], -R3_6[2,1]).evalf()
                theta6 = theta46 - theta4
                print("-------------> A wrist singularity reached!")
                
	    """
            while(theta4 > pi):
		theta4 = theta4 - 2*pi
                print("theta4 > pi")
            while(theta4 < -pi):
		theta4 = theta4 + 2*pi
		print("theta4 < -pi")

            while(theta5 > pi):
		theta5 = theta5 - 2*pi
		print("theta5 > pi")
            while(theta5 < -pi):
		theta5 = theta5 + 2*pi
                print("theta5 < -pi")

            while(theta6 > pi):
		theta6 = theta6 - 2*pi
                print("theta6 > pi")
            while(theta6 < -pi):
		theta6 = theta6 + 2*pi
		print("theta6 < -pi")"""
            


            delta_4 = theta4 - theta4_prev # compute displacement to new angle
	    delta_5 = theta5 - theta5_prev
            delta_6 = theta6 - theta6_prev
            
            while delta_4 > pi:                          # check if displacement to large
		theta4 = theta4_prev + (delta_4  - 2*pi) # if so, compute shorter displacement to same point
                delta_4 = theta4 - theta4_prev           # check if new diff is small enough
                print("delta_4 > pi")
            while(delta_4  < -pi):
		theta4 = theta4_prev + (delta_4  + 2*pi)
                delta_4 = theta4 - theta4_prev # 
	        print("delta_4 < -pi")

            while(delta_5 > pi):
		theta5 = theta5_prev + (delta_5 - 2*pi)
                delta_5 = theta5 - theta5_prev #
                print("delta_5 > pi")
            while(delta_5 < -pi):
		theta5 = theta5_prev + (delta_5 + 2*pi)
                delta_5 = theta5 - theta5_prev #
                print("delta_5 < -pi")

            while(delta_6 > pi):
		theta6 = theta6_prev + (delta_6 - 2*pi)
                delta_6 = theta6 - theta6_prev #
		print("delta_6 > pi")
            while(delta_6 < -pi):
		theta6 = theta6_prev + (delta_6 + 2*pi)
                delta_6 = theta6 - theta6_prev #
		print("delta_6 < -pi")

            theta4_prev = theta4
	    theta5_prev = theta5
            theta6_prev = theta6 
     	


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

            #print(".............. All joints...................") 
            #print(joint_trajectory_point.positions)


        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
