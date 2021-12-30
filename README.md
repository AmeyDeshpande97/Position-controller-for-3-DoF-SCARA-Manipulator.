# Position-controller-for-3-DoF-SCARA-Manipulator.
The goal of this project is to develop and implement a joint space position control for a 3-DoF RevoluteRevolute-Prismatic (RRP) robot manipulator. We implemented a position-based controller and incorporated 
precise PID (Proportional Integral Derivative) control for joint space tracking of the robot in the Gazebo.
There were two main parts of this project:
• Calculate Inverse Kinematics of the robot. End effector position was given and all the joint angles of 
the robot are calculated.
• Once intended joint angles are calculated, they are fed to the PID controller to reach the desired joint 
destination and desired orientation of the robot.
The two parts will be explained thoroughly in following sections:
1. Inverse Kinematics:
In Inverse Kinematics we calculate the joint angles based on the end-effector position. 
There are 2 approaches to calculate the inverse kinematics, one is Geometrical approach and another 
is Arithmetic approach.
The arithmetic approach is quite tiresome, and many calculations had to be performed. So, we went 
with the geometric approach.
Following are the calculations for Inverse Kinematics using Geometric approach
2. PID Control
The second part of our project involved the use of PID to control the joint movements of the revolute 
and prismatic joint. The robot had to be controlled to reach 4 desired points. 
P1 = [0.0, 0.77, 0.34]
P2 = [-0.345, 0.425, 0.24]
P3 = [-0.67, -0.245, 0.14]
P4 = [0.77, 0.0, 0.39]
The joint angles of the individual points were computed using Inverse Kinematics, which has been 
described in the previous section. The jou=int angles were subscribed using the Joint States topic. 
The value being published is the u(t) value which was calculated as follows:
The Kp, Kd and Ki values for each of the joints are tuned as follows:
Kp Kd Ki
Joint 1 7 6 0.0001
Joint 2 2 1 -0.0005
Joint 3 750 5 -0.5
The joint torques were calculated by publishing to the ‘/rrp/jointN_effort_controller/command’ 
command (where N stands for the joint number) and the feedback values were taken by 
subscribing to the /rrp/joint_states topic. The control rate was kept at 100 Hz.
The simulation of the same is performed on Gazebo

Video Link:
https://wpi0-
my.sharepoint.com/:v:/g/personal/aadeshpande2_wpi_edu/EUj4Whvmu6dKlX740tALS1E
B5m7H_xa3PvFHi_wbuAVQRA
