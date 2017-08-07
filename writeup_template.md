# Project: Kinematics Pick & Place

---

[//]: # (Image References)

[pr_image1]: ./misc/pr_image1.png
[pr_image0]: ./misc/pr_image0.png
[pr_image2]: ./misc/pr_image2.png
[pr_image3]: ./misc/pr_image3.png
[nav_image0]: ./misc/nav_image.png
[pr_video]: .output/mydata_mapping_final

## Introduction
The objective of this project was to write an Inverse Kinematics solver for the KUKA KR210 robot. This solver is responsible for computing the joint angles corresponding to a desired end-effector or gripper trajectory. The algorithm was tested on pick and place operations, consisting of collecting objects from different locations on a shelf and depositing them on a bin.

(image or GIF)
---

## Kinematic Analysis

### PART 1. Robot geometry and DH parameters

The following scheme shows the Kuka Kuka KR210 and its geometric parameters.

(picture)

In this figure, each joint is assigned an origin (**Oi**) and frame (**Zi** and **Xi**). The position and orientation of each joint from the base to the end-effector is described by a series of geometric parameters (**alpha**, **a**, **d**, **theta**), but only **a** and **d** are depicted here, where:

* The angle **alpha** corresponds to the angle between **Zi-i** and **Zi** along **Xi-1**.

* The distance **a** correponds to the distance between **Zi-1** and **Zi** along **Xi-1**.

* The distance **d** corresponds to the distance between **Xi-1** and **Xi** along **Zi**.

* The angle **theta** (or in the case of revolute joints: joint variable) represents the angle between **Xi-1** and **Xi** along **Zi**.

Such geometric description can also be summed up in a DH parameter table as follows:

i | \alpha_{i-1} | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | d1 | q1
2 | - pi/2 | a1| 0 | -pi/2 + q2
3 | 0 | a2 | 0 | q3
4 |  - pi/2 | a3 | d4 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
EE | 0 | 0 | d7 | 0

The actual numerical values for these paramaters can be extracted from the robots URDF file.
This file contains the distances and orientations between joint frames, however, one must be careful when extracting this information, since the the URDF frames do not correspond to our DH convention.

i | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | - pi/2 | 0.35| 0 | -pi/2 + q2
3 | 0 | 1.25 | 0 | q3
4 |  - pi/2 | -0.054 | 1.5 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
EE | 0 | 0 | 0.303 | 0

where:

* d1 was obtained by adding the distance between joint 0 -> 1 and joint 1 -> 2 along Z in the URDF.
* a1 corresponds to the distance between joint 1 -> 2 along X in the URDF.
* a2 corresponds to the distance between joint 2 -> 3 along Z in the URDF.
* a3 corresponds to the distance between joint 3 -> 4 along Z in the URDF.
* d4 was obtained by adding the distance between joint 3 -> 4 and joint 4 -> 5 along X in the URDF.
* d7 was obtained by adding the distance between joint 5 -> 6 and joint 6 -> 7 (or gripper joint) along X in the URDF

### PART 2. Forward Kinematic Model (FKM)
The forward kinematic model consists in expressing the position and orientation (or pose) of the robot's end-effector as function of its joint variables.
To do this, tranformation matrices compositions (or multiplications) are used. Transformation matrices specify the pose of a joint frame with respect to its predecessor. By multiplying such matrices from the base frame to the end-effector we can obtain its pose expressed in the base frame.


#### a) General transformation matrix between two frames
The general form of these matrices is the following:

![](https://latex.codecogs.com/gif.latex?_%7Bi%7D%5E%7Bi-1%7D%5Ctextrm%7BT%7D%3D%5Cbegin%7Bbmatrix%7D%20cos%28%5Ctheta_i%29%20%26%20-sin%28%5Ctheta_i%29%20%26%200%20%26%20a_%7Bi-1%7D%5C%5C%20sin%28%5Ctheta_i%29cos%28%5Calpha_%7Bi-1%7D%29%20%26%20cos%28%5Ctheta_i%29cos%28%5Calpha_%7Bi-1%7D%29%20%26%20-sin%28%5Calpha_%7Bi-1%7D%29%20%26%20-sin%28%5Calpha_%7Bi-1%7D%29d_%7Bi%7D%20%5C%5C%20sin%28%5Ctheta_i%29sin%28%5Calpha_%7Bi-1%7D%29%20%26%20cos%28%5Ctheta_i%29sin%28%5Calpha_%7Bi-1%7D%29%20%26%20cos%28%5Calpha_%7Bi-1%7D%29%20%26%20cos%28%5Calpha_%7Bi-1%7D%29d_%7Bi%7D%20%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)
  
#### b) Transformation matrices for Kuka KR210
To calculate these matrices for the Kuka KR210 the following function was implemented:

``` python

    	# Define Modified DH Transformation matrix         
    	def Trans_mat(alphaj, aj, di, qi):
             Tj_i = Matrix([ [       cos(qi),            -sin(qi),                0,                 aj],
			 [sin(qi)*cos(alphaj),  cos(qi)*cos(alphaj),   -sin(alphaj),    -sin(alphaj)*di],  
			 [sin(qi)*sin(alphaj),  cos(qi)*sin(alphaj),    cos(alphaj),     cos(alphaj)*di],
			 [         0,                   0,                   0,                  1]])
             return Tj_i
```


Then this function was called several times to construct the tranformation matrix between the joint frame **Ri-1** to **Ri** using the appropriate DH parameters (parameters of row **i** in the DH parameter table). Such matrices are featured below:

TO_1:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20cos%28q_1%29%20%26%20-sin%28q_1%29%20%26%200%20%26%200%5C%5C%20sin%28q_1%29%20%26%20cos%28q_1%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%201%20%26%200.75%5C%5C%200%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)

T1_2:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20sin%28q_%7B2%7D%29%26%20cos%28q_%7B2%7D%29%20%26%200%20%26%200.35%20%5C%5C%200%20%26%200%20%26%201%20%26%200%20%5C%5C%20cos%28q_%7B2%7D%29%20%26%20-sin%28q_%7B2%7D%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)

T2_3:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20cos%28q_%7B3%7D%29%20%26%20-sin%28q_%7B3%7D%29%20%26%200%20%26%201.25%5C%5C%20sin%28q_%7B3%7D%29%20%26%20cos%28q_%7B3%7D%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%201%20%26%200%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)

T3_4:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20cos%28q_%7B4%7D%29%20%26%20-sin%28q_%7B4%7D%29%20%26%200%20%26%20-0.054%5C%5C%200%20%26%200%20%26%201%20%26%201.5%5C%5C%20-sin%28q_%7B4%7D%29%20%26%20-cos%28q_%7B4%7D%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)

T4_5:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20cos%28q_%7B5%7D%29%20%26%20-sin%28q_%7B5%7D%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%20-1%20%26%200%5C%5C%20sin%28q_%7B5%7D%29%20%26%20cos%28q_%7B5%7D%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)

T5_6:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20cos%28q_%7B6%7D%29%20%26%20-sin%28q_%7B6%7D%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%201%20%26%200%5C%5C%20-sin%28q_%7B6%7D%29%20%26%20-cos%28q_%7B6%7D%29%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)

T6_7:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%200%20%26%200%5C%5C%200%20%26%201%20%26%200%20%26%200%5C%5C%200%20%26%200%20%26%201%20%26%200.303%5C%5C%200%20%26%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D)
  
#### c) FKM model
Once these matrices were obtained they were multiplied to obtain the pose of the end-effector's frame expressed in the base frame.

``` python

T_FKM = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7

```

#### d) Homogenous transformation base and gripper
This final matrix can also be obtained using solely the end-effectors pose (for instance, when provided by a simulator).
In this case, we would be given a quaternion representation of the orientation, which would have to be transformed into euler angles, and finally into a homogenous transformation as follows:

``` python

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
R0_EE = R0_EE * R_corr   

```
   
Note that the angles returned by the function **euler_from_quaternion** are extrinsic euler angles corresponding to an XYZ convention. Thus, the composition of matrices for such convention corresponds to rotation around Z (yaw), followed by a rotation around Y (pitch), and a rotation about X (roll).



### PART 3. Inverse Kinematic Model (IKM)
The inverse kinematic model consists in computing the joint positions given the position of the end-effector. This part of the analysis is the most important for this project because it will allow us to compute the trajectories in the joint that correspond to the task space trajectories given by the motion planner.

Since the robot has a spherical wrist the IK problem can be divided into two parts: The inverse p0osition kinematics problem and the inverse orientation kinematics problem. The former is used to compute the first three joint values (which determine the position of the end-effector), while the latter is used to compute the last three joint values (which determine the orientation of the end-effector.

#### a) Inverse Position Kinematics
The first three joint variables (theta1, theta2 and theta3) were computed as follows:

First, the wrist center position was computed, given the end-effector pose:



Where n represents the z-axis of matrix R0_EE and d_6, d_7 are DH parameters.


Then, the angles were determined using this information along side the robot's geometric parameters and trigonometric identities:

* Theta1 computation (drawings, code formulas)

  ![](https://latex.codecogs.com/gif.latex?theta1%3D%20%5Ctan%20%5E%7B-1%7D%28wy/wx%29)


* Theta2 computation

  ![](https://latex.codecogs.com/gif.latex?theta2%20%3D%20pi/2%20-%20A%20-%20alp1)

  First, the sides of the auxiliary triangle 1 and 2 are computed:

  ![](https://latex.codecogs.com/gif.latex?a%20%3D%20%5Csqrt%7Bd4%5E%7B2%7D%20&plus;%20a3%5E%7B2%7D%7D)

  ![](https://latex.codecogs.com/gif.latex?s1%20%3D%20wz%20-%20d1)

  ![](https://latex.codecogs.com/gif.latex?s2%20%3D%20%5Csqrt%7Bwx%5E2%20&plus;%20wy%5E2%20%7D%20-%20a1)

  ![](https://latex.codecogs.com/gif.latex?b%20%3D%20%5Csqrt%7Bs2%5E2%20&plus;%20s1%5E2%7D)

  ![](https://latex.codecogs.com/gif.latex?c%20%3D%20a2)

  Then, the angles of interest are computed

  ![](https://latex.codecogs.com/gif.latex?alp1%20%3D%20%5Ctan%5E%7B-1%7D%28s1/s2%29)

  ![](https://latex.codecogs.com/gif.latex?A%20%3D%20%5Ccos%5E%7B-1%7D%28%28b%5E2%20&plus;%20c%5E2%20-%20a%5E2%29/%282bc%29%29)

  ![](https://latex.codecogs.com/gif.latex?B%20%3D%20%5Ccos%5E%7B-1%7D%28%28c%5E2%20&plus;%20a%5E2%20-%20b%5E2%29/%282ac%29%29)

  ![](https://latex.codecogs.com/gif.latex?C%20%3D%20%5Ccos%5E%7B-1%7D%28%28a%5E2%20&plus;%20b%5E2%20-%20c%5E2%29/%282ab%29%29)

* Theta3 computation




#### b) Inverse Orientation Kinematics
The final three angles were computed by substituting the first three angles and the known end-effector pose in the following equation, and the solving form matrix R36. A matrix function of angles theta4, theta5 and theta 6 only.
(code equation R0gripper)
(code equation R36)


Equating the unknown terms on the RHS (right hand side) to the known terms on the LHS (left hand side) resulted in the following equations: 

* Theta4 computation




* Theta5 computation





* Theta6 computation

---
## Project Implementation

The previous analysis was implemented as a python script: **IK_server.py**. A script which calculates the joint trajectories corresponding to desired end-effector poses. The code is divided into thwo main sections: forward kinematics and inverse kinematics.

* The forward kinematics section contains the implementation of PART 1 (Robot geometry and DH parameters) and PART 2 (Forward Kinematic Model (FKM)) of the previous analysis.
* The inverse kinematics section contains the implementation of PART 3 (Inverse Kinematic Model (IKM)) of the previous analysis. Additionally, this part of the code also checks for the following special cases:

    1. If a wrist singularity occured (when theta5 = 0, and theta4 and 6 become collinear): When this happens the values of joint 4 and 6 cannot be determined separately, only their sum (theta46). Thus, theta4 is assigned its previous value and theta6 is set to theta46 minus the  current value of theta4. 
    2. If the sine of theta5 is positive or negative, as this influences the computation of theta4 and theta6.
    3. If the angular displacement from one joint position to the next is too large (greater or less than +/- 180 degrees):  When this happens the shortest displacement to the desired joint position is computed and added to the current joint position.

(Code add)

To validate the results of this project the robot was tested in 10 pick and place operations (spawn location 1-9, # was repeated). The results show that the robot is able to successfully pick and place the objects *#/10* times while following the desired end-effector trajectories.
Click here, to see an example of a pick and place operation.

However, there are a number of improvements that I still wish to implement:

* Reduce unecessary end-effector rotations: Although I check for large and uncessary angular displacements, the robot still exhibits some unecessary rotations, especially in the beginning. I will keep checking and improving this part of the code.
* In 2/10 occasions the orientation of the robot did not reach the desired end-effector pose at the dropping site. I still need to determine the cause of this. I noticed in both occasions my code for checking large angular displacements was executed, so I'll try to determine if the error happens here or somewhere else.

Other improvements I would like to make:
* Optimize the code to make it faster
* Compute the various IKM solutions for each end-effector position and select the best one (to stay within the robot's reachable workspace, avoid singularities etc.)




