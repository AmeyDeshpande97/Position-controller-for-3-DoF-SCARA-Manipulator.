#!/usr/bin/env python
import numpy as np
from math import pi, cos, sin, atan2, acos, sqrt, atan, degrees
import rospy
from rbe500_project.srv import rrpIK, rrpIKResponse

def angle_1 (a, b, c):
	return acos((c**2 - b**2 - a**2)/(2.0 * a * b))

def angle (a, b, c):
	#return acos((c**2 - b**2 - a**2)/(-2.0 * a * b))
	return acos((c**2 + b**2 - a**2)/(2.0 * b * c))

def print1(req):
	x = req.px
	print("x is:",x)
	y = req.py
	print("y is:",y)
	z = req.pz
	print("z is:",z)
	joint1 = 0.1
	joint2 = 0.2
	joint3 = 0.3
	joint_angles = [joint1, joint2, joint3]
	return rrpIKResponse(joint_angles)



def inverse_kinematics_1(req):
	x = req.px
	print("x is:",x)
	y = req.py
	print("y is:",y)
	z = req.pz
	print("z is:",z)
	a1 = 0.425
	a2 = 0.345
	r  = sqrt(x**2 + y**2)
	base_length = 0.1
	link_1 = 0.45
	gripper_length = 0.11
	b = ((r**2) - (a1**2) -(a2**2))/((2*a1*a2))
	joint2 = atan2(sqrt(1 - b**2),b)
	phi = atan2(y,x)
	beta = acos(((a1**2) + (r**2) -(a2**2))/(2*a1*r))
	joint1 = phi - beta
	joint3 = (base_length + link_1  - gripper_length) - z 
	joint_angles = [joint1, joint2, joint3]
	return rrpIKResponse(joint_angles)



def inverse_kinematics(req):
	x = req.px
	print("x is:",x)
	y = req.py
	print("y is:",y)
	z = req.pz
	print("z is:",z)
	a1 = 0.425
	a2 = 0.345
	r  = sqrt(x**2 + y**2)
	base_length = 0.05
	link_1 = 0.45
	gripper_length = 0.11
	cosjoint2 = (((a1**2) + (a2**2) - (r**2))/(2*a1*a2))
	sinjoint2 = sqrt(1 - cosjoint2**2)
	joint2 = atan2(sinjoint2,cosjoint2)
	phi = atan2(y,x)
	beta = acos(((a1**2) + (r**2) -(a2**2))/(2*a1*r))
	joint1 = phi - beta
	joint3 = (base_length + link_1  - gripper_length) - z 
	joint_angles = [joint1, joint2, joint3]
	return rrpIKResponse(joint_angles)


if __name__ == '__main__':
	rospy.init_node('rrp_server_node') # initialize the node

	rrp_service = rospy.Service('generate_joint_values',rrpIK, inverse_kinematics_1)

	rospy.spin()



