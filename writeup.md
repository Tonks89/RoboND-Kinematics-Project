# Project: Kinematics Pick & Place

---

[//]: # (Image References)

[image0]: ./misc_images/kuka_pickplace.png
[image1]: ./misc_images/kuka_geometry_DH.png
[image2]: ./misc_images/Theta1.png
[image3]: ./misc_images/Theta2.png
[image4]: ./misc_images/Theta3.png

## Introduction
The objective of this project was to write an Inverse Kinematics solver for the KUKA KR210 robot. This solver is responsible for computing the joint angles corresponding to a desired end-effector or gripper trajectory. The algorithm was tested on pick and place operations, consisting of collecting objects from different locations on a shelf and depositing them on a bin.

![alt text][image0]

---

## Kinematic Analysis

### PART 1. Robot geometry and DH parameters

The following scheme shows the Kuka Kuka KR210 and its geometric parameters.

![alt text][image1]

In this figure, each joint is assigned an origin (**Oi**) and frame (**Zi** and **Xi**). The position and orientation of each joint from the base to the end-effector is described by a series of geometric parameters (**alpha**, **a**, **d**, **theta**), but only **a** and **d** are depicted here:

* The angle **alpha** corresponds to the angle between **Zi-i** and **Zi** along **Xi-1**.

* The distance **a** correponds to the distance between **Zi-1** and **Zi** along **Xi-1**.

* The distance **d** corresponds to the distance between **Xi-1** and **Xi** along **Zi**.

* The angle **theta** (or in the case of revolute joints: joint variable) represents the angle between **Xi-1** and **Xi** along **Zi**.

Such geometric description can also be summed up in a DH parameter table as follows:

i | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | d1 | q1
2 | - pi/2 | a1| 0 | -pi/2 + q2
3 | 0 | a2 | 0 | q3
4 |  - pi/2 | a3 | d4 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
EE | 0 | 0 | d7 | 0

The actual numerical values for these paramaters can be extracted from the robot's URDF file.
This file contains the distances and orientations between joint frames, however, one must be careful when extracting this information since the the URDF frames do not correspond to our DH convention.

i | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | - pi/2 | 0.35| 0 | -pi/2 + q2
3 | 0 | 1.25 | 0 | q3
4 |  - pi/2 | -0.054 | 1.5 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
EE | 0 | 0 | 0.303 | 0

Where:

* d1 was obtained by adding the distance between joint 0 -> 1 and joint 1 -> 2 along **Z** in the URDF.
* a1 corresponds to the distance between joint 1 -> 2 along **X** in the URDF.
* a2 corresponds to the distance between joint 2 -> 3 along **Z** in the URDF.
* a3 corresponds to the distance between joint 3 -> 4 along **Z** in the URDF.
* d4 was obtained by adding the distance between joint 3 -> 4 and joint 4 -> 5 along **X** in the URDF.
* d7 was obtained by adding the distance between joint 5 -> 6 and joint 6 -> 7 (or gripper joint) along **X** in the URDF

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


Then, this function was called several times to construct the tranformation matrix between the joint frame **Ri-1** to **Ri** using the appropriate DH parameters (parameters of the ith-row in the DH parameter table). Such matrices are featured below:

T0_1:

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
   
Note that the angles returned by the function **euler_from_quaternion** are extrinsic euler angles corresponding to an **XYZ** convention. Thus, the composition of matrices for such convention corresponds to rotation around **Z** (yaw), followed by a rotation around **Y** (pitch), and a rotation about **X** (roll).



### PART 3. Inverse Kinematic Model (IKM)
The inverse kinematic model consists in computing the joint positions given the position of the end-effector. This part of the analysis is the most important because it will allow us to compute the *joint space* trajectories that correspond to the *task space trajectories* given by the motion planner.

Since the robot has a spherical wrist the IK problem can be divided into two parts: The *inverse position kinematics* problem and the *inverse orientation* kinematics problem. The former is used to compute the first three joint values (which determine the position of the end-effector), while the latter is used to compute the last three joint values (which determine the orientation of the end-effector).

#### 1) Inverse Position Kinematics
The first three joint variables (theta1, theta2 and theta3) were computed as follows:

First, the wrist center position was computed, given the end-effector pose:

![](https://latex.codecogs.com/gif.latex?w_x%20%3D%20p_x%20-%20%28d_6%20&plus;%20d_7%29n_x)

![](https://latex.codecogs.com/gif.latex?w_y%20%3D%20p_y%20-%20%28d_6%20&plus;%20d_7%29n_y)

![](https://latex.codecogs.com/gif.latex?w_z%20%3D%20p_z%20-%20%28d_6%20&plus;%20d_7%29n_z)

Where n represents the z-axis of matrix R0_EE.


Then, the angles were determined using this information along side the robot's geometric parameters and trigonometric identities:

* Theta1 computation 

  ![](https://latex.codecogs.com/gif.latex?\theta_1%3D%20%5Ctan%20%5E%7B-1%7D%28w_y/w_x%29)
  
  ![alt text][image2]

  Note that the atan2 function was used for this and the rest of the inverse tangent computations in the actual code.

* Theta2 computation

  ![](https://latex.codecogs.com/gif.latex?\theta_2%20%3D%20\pi/2%20-%20A%20-%20alp_1)

  The following figure illustrates each of the angles involved in the above computation. To determine them, two auxiliary triangles were used.

  ![alt text][image3]

  First, the sides of the auxiliary triangle 1 and 2 were computed:

  ![](https://latex.codecogs.com/gif.latex?a%20%3D%20%5Csqrt%7Bd_4%5E%7B2%7D%20&plus;%20a_3%5E%7B2%7D%7D)

  ![](https://latex.codecogs.com/gif.latex?s_1%20%3D%20w_z%20-%20d_1)

  ![](https://latex.codecogs.com/gif.latex?s_2%20%3D%20%5Csqrt%7Bw_x%5E2%20&plus;%20w_y%5E2%20%7D%20-%20a_1)

  ![](https://latex.codecogs.com/gif.latex?b%20%3D%20%5Csqrt%7Bs_2%5E2%20&plus;%20s_1%5E2%7D)

  ![](https://latex.codecogs.com/gif.latex?c%20%3D%20a_2)

  Then, the angles of interest were computed:

  ![](https://latex.codecogs.com/gif.latex?alp1%20%3D%20%5Ctan%5E%7B-1%7D%28s1/s2%29)

  ![](https://latex.codecogs.com/gif.latex?A%20%3D%20%5Ccos%5E%7B-1%7D%28%28b%5E2%20&plus;%20c%5E2%20-%20a%5E2%29/%282bc%29%29)

  ![](https://latex.codecogs.com/gif.latex?B%20%3D%20%5Ccos%5E%7B-1%7D%28%28c%5E2%20&plus;%20a%5E2%20-%20b%5E2%29/%282ac%29%29)

  ![](https://latex.codecogs.com/gif.latex?C%20%3D%20%5Ccos%5E%7B-1%7D%28%28a%5E2%20&plus;%20b%5E2%20-%20c%5E2%29/%282ab%29%29)

* Theta3 computation

  ![](https://latex.codecogs.com/gif.latex?%5Ctheta_3%20%3D%20%5Cpi/2%20-%20B%20-%20%5Cbeta)

  The following figure illustrates each of the angles involved in the above computation. To determine them, three auxiliary triangles were used.

  ![alt text][image4]


  ![](https://latex.codecogs.com/gif.latex?%5Cgamma%20%3D%20%5Ctan%5E%7B-1%7D%28a_%7B3%7D/g%29)

  ![](https://latex.codecogs.com/gif.latex?f%20%3D%20%5Csqrt%7Bg%5E2%20&plus;%20a_3%5E2%7D)

  ![](https://latex.codecogs.com/gif.latex?%5Cvarepsilon%20%3D%20cos%5E%7B-1%7D%28%28-e%5E2%20&plus;%20a%5E2%20&plus;%20f%5E2%29/%282af%29%29)

  ![](https://latex.codecogs.com/gif.latex?%5Cbeta%20%3D%20%7C%5Cgamma%7C%20-%20%7C%5Cvarepsilon%7C)

  Where g = 0.96 and e = 0.54.


#### 2) Inverse Orientation Kinematics
The final three angles were computed by substituting the first three angles and the known end-effector pose in the following equation, and the solving for matrix R36. Note that matrix R36 is function of angles theta4, theta5 and theta6 only.


![](https://latex.codecogs.com/gif.latex?_%7B3%7D%5E%7B0%7D%5Ctextrm%7BR%7D%20_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%20%3D%20_%7BEE%7D%5E%7B0%7D%5Ctextrm%7BR%7D)

![](https://latex.codecogs.com/gif.latex?_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%20%3D%20_%7B0%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5C%2C%20_%7BEE%7D%5E%7B0%7D%5Ctextrm%7BR%7D)

Where,

![](https://latex.codecogs.com/gif.latex?_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%20%3D%20%5Cbegin%7Bbmatrix%7D%20-sin%28%5Ctheta_%7B4%7D%29sin%28%5Ctheta_%7B6%7D%29%20&plus;%20cos%28%5Ctheta_%7B4%7D%29cos%28%5Ctheta_%7B5%7D%29cos%28%5Ctheta_%7B6%7D%29%20%26%20-sin%28%5Ctheta_%7B4%7D%29cos%28%5Ctheta_%7B6%7D%29%20-%20sin%28%5Ctheta_%7B6%7D%29cos%28%5Ctheta_%7B4%7D%29cos%28%5Ctheta_%7B5%7D%29%20%26%20-sin%28%5Ctheta_%7B5%7D%29cos%28%5Ctheta_%7B4%7D%29%5C%5C%20sin%28%5Ctheta_%7B5%7D%29cos%28%5Ctheta_%7B6%7D%29%20%26%20-sin%28%5Ctheta_%7B5%7D%29sin%28%5Ctheta_%7B6%7D%29%20%26%20cos%28%5Ctheta_%7B5%7D%29%5C%5C%20-sin%28%5Ctheta_%7B4%7D%29cos%28%5Ctheta_%7B5%7D%29cos%28%5Ctheta_%7B6%7D%29%20-%20sin%28%5Ctheta_%7B6%7D%29cos%28%5Ctheta_%7B4%7D%29%20%26%20sin%28%5Ctheta_%7B4%7D%29sin%28%5Ctheta_%7B6%7D%29cos%28%5Ctheta_%7B5%7D%29%20-%20cos%28%5Ctheta_%7B4%7D%29cos%28%5Ctheta_%7B6%7D%29%20%26%20sin%28%5Ctheta_%7B4%7D%29sin%28%5Ctheta_%7B5%7D%29%20%5Cend%7Bbmatrix%7D)


Equating the unknown terms on the RHS (right hand side) to the known terms on the LHS (left hand side) resulted in the following equations: 


* Theta4 computation

![](https://latex.codecogs.com/gif.latex?%5Ctheta_4%20%3D%20%5Ctan%5E%7B-1%7D%5Cleft%20%28%20_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5B2%5D%5B2%5D%5C%2C%20/%20-_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5B0%5D%5B2%5D%20%5Cright%20%29)

* Theta5 computation

![](https://latex.codecogs.com/gif.latex?%5Ctheta_5%20%3D%20%5Ctan%5E%7B-1%7D%5Cleft%20%28%20%5Cleft%20%28%5Csqrt%7B%28_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5B0%5D%5B2%5D%29%5E2%20&plus;%20%28_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5B2%5D%5B2%5D%29%5E2%7D%20%5Cright%20%29/_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5B1%5D%5B2%5D%20%5Cright%20%29)

* Theta6 computation

![](https://latex.codecogs.com/gif.latex?%5Ctheta_6%20%3D%20%5Ctan%5E%7B-1%7D%5Cleft%20%28%20-_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5B1%5D%5B1%5D%20%5C%2C%20/%5C%2C%20_%7B6%7D%5E%7B3%7D%5Ctextrm%7BR%7D%5B1%5D%5B0%5D%20%5Cright%20%29)


#### 3) Special Cases Checking
The previous equations were used to compute the joint trajectories corresponding to the desired end-effector position. However, there are special cases in which the computations have to be handled differently. Such cases are outlined below:

* Wrist singularities (when theta5 = 0, and theta4 and 6 become collinear): When this happens the values of joint 4 and 6 cannot be determined separately, only their sum (theta46). Thus, theta4 is assigned its previous value and theta6 is set to theta46 minus the current value of theta4. 

* Negative sine of theta5: this influences the computation of theta4 and theta6. Thus, their computations are altered accordingly.

* Large angular displacements from one joint position to the next (greater or less than +/- 180 degrees):  When this happens the shortest displacement to the desired joint position is computed and added to the current joint position.

The following code shows this special cases checking:


``` python

#--------- 3. Special cases checking -----------

# Check sign of S5
if sin(theta5) < 0:
    theta4 = (atan2(-R3_6[2,2],R3_6[0,2])).evalf()
    theta6 = (atan2(R3_6[1,1],-R3_6[1,0])).evalf()
    print("-------------> S5 < 0 ")

# Check wrist singularity
if (theta5 == 0): 
    theta4 = theta4_prev # keep q4 current value
    theta46 = atan2(-R3_6[0,1], -R3_6[2,1]).evalf()
    theta6 = theta46 - theta4
    print("-------------> A wrist singularity reached!")

# Check large angular displacements
delta_4 = theta4 - theta4_prev # compute displacement to new angle
...

while delta_4 > pi:                          # check if displacement to large
    theta4 = theta4_prev + (delta_4  - 2*pi) # if so, compute shorter displacement to same point
    delta_4 = theta4 - theta4_prev           # check if new difference is small enough
    print("delta_4 > pi")
while(delta_4  < -pi):
    theta4 = theta4_prev + (delta_4  + 2*pi)
    delta_4 = theta4 - theta4_prev  
    print("delta_4 < -pi")

...

```


---
## Project Implementation

The previous analysis was implemented as a python script: **IK_server.py**. A script which calculates the joint trajectories corresponding to desired end-effector poses. The code is divided into two main sections: forward kinematics and inverse kinematics.

* The forward kinematics section contains the implementation of PART 1 (Robot geometry and DH parameters) and PART 2 (Forward Kinematic Model (FKM)) of the previous analysis.
* The inverse kinematics section contains the implementation of PART 3 (Inverse Kinematic Model (IKM)) of the previous analysis. 

To validate the results of this project the robot was tested in 10 pick and place operations (spawn locations 1-9, location 7 was repeated to achieve 10 operations). The results show that the robot is able to successfully pick and place the objects 9/10 times while following the desired end-effector trajectories.
Click [here](https://www.youtube.com/watch?v=OULJVEaE0yI), to see a video of a pick and place operation in Rviz (2x actual simulation speed), and [here](https://www.youtube.com/watch?v=egHG-RJTBWs) to see another example in Gazebo (4x actual simulation speed).


Furthermore, there are a number of improvements that I still wish to implement:

* Reduce unecessary end-effector rotations: Although I check for large and uncessary angular displacements, the robot still exhibits some unecessary rotations, especially in the beginning. I will keep checking and improving this part of the code.
* In 2/10 occasions the desired end-effector pose at the dropping site was not reached. I still need to determine the cause of this. I noticed that in both occasions the code for checking large angular displacements was executed, so I'll try to determine if the error happens here or somewhere else.

Other improvements I would like to make:
* Optimize the code to make it faster.
* Compute the various IKM solutions for each end-effector position and select the best one (to stay within the robot's reachable workspace, avoid singularities etc.)
* I noticed that when the robot needs to move from point A to B (once the IKM solver has finished its computations), the motion is slow and not fluid. I think this could be either due to the fact that more points are needed along the trajectory and/or to the fact that my virtual machine has low graphics performance as pointed out by the following message: "No 3D support available from the host, hardware graphics acceleration is not available".




