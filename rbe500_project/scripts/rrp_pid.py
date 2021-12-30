#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from math import pi
import time
import rospy
from std_msgs.msg import Header
#from rrpIK.srv import rrpIK, rrpIKResponse, rrpIKRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rbe500_project.srv import *


SERVICE_NAME = 'generate_joint_values'


class CommandToJointState():
	#Initialization 
	def __init__(self): 
		rospy.init_node("position_controller_node")
		rospy.loginfo("Press Ctrl + C to terminate")
		self.pub1 = rospy.Publisher('rrp/joint1_effort_controller/command', Float64, queue_size=10)
		self.pub2 = rospy.Publisher('rrp/joint2_effort_controller/command', Float64, queue_size=10)
		self.pub3 = rospy.Publisher('rrp/joint3_effort_controller/command', Float64, queue_size=10)
	
		self.rate = rospy.Rate(100)
		self.joint_state = JointState()
		
		self.px1 = 0.0
		self.px2 = 0.0
		self.px3 = 0.0
		
		self.joint1_flag = 0
		self.joint2_flag = 0
		self.joint3_flag = 0
		self.sub = rospy.Subscriber('/rrp/joint_states', JointState, self.joint_states_callback)
	
	#Function to check the angle and reset it if it completes one revolution. 
	def checkAngle(self, pos):
		if pos>pi:
			return (pos - pi)
		elif pos<-pi:
			return (pos + pi)
		else:
			return pos

	#Callback Function
	def joint_states_callback(self,msg):
		self.joint_state.position = msg.position
		self.joint_state.velocity = msg.velocity
		self.px1 = self.checkAngle(self.joint_state.position[0])
		self.px2 = self.checkAngle(self.joint_state.position[1])
		self.px3 = self.joint_state.position[2]
		# print("joint1 state position is",self.px1)
		# print("joint2 state position is",self.px2)
		# print("joint3 state position is",self.px3)

	#Function to call the inverse function and the function to control the joints
	def CalculateInverseAndPID(self,position):
		rospy.wait_for_service(SERVICE_NAME)
		rrp_client_node = rospy.ServiceProxy(SERVICE_NAME,rrpIK)
		response = rrp_client_node(position[0], position[1], position[2])
		rospy.loginfo(response.joint_angles)
		jointAngles = response.joint_angles

		self.destination_joint_1 = jointAngles[0]
		self.destination_joint_2 = jointAngles[1]
		self.destination_joint_3 = jointAngles[2]
		# print("destination_joint_1",self.destination_joint_1)
		# print("destination_joint_2",self.destination_joint_2)
		# print("destination_joint_3",self.destination_joint_3)
		p = self.ControlJoints()
		return p

	#Function to control the joints
	def ControlJoints(self):
		error = 0.0
		sum_error = 0.0
		prev_error = 0.0
		Kp = 7.0
		Kd = 4.0
		Ki = 0.0001

#JOINT 1
		while (self.joint1_flag == 0): 
			prev_error = error
			error = (self.destination_joint_1 - self.px1)
			delta_error = error - prev_error #(e(t) − e(t − ∆t))
			sum_error = sum_error + error*(prev_error)

			if(prev_error != 0):
				ut = (Kp*error + Kd*(delta_error/prev_error) + Ki*sum_error)
			else:
				ut = (Kp*error + Kd*(delta_error) + Ki*sum_error)

			if (ut > 1.0):
				ut = 0.3
			elif (ut < -0.5):
				ut = -0.3
			elif (ut < 1 and ut > 0):
				ut = 0.2
			elif (ut < 0 and ut > -0.5):
				ut = -0.2

			self.pub1.publish(ut)
			self.pub2.publish(0)
			self.pub3.publish(0)

			if ((self.px1 > (self.destination_joint_1 - 0.15)) and (self.px1 < (self.destination_joint_1 + 0.15))):
				self.joint1_flag = 1
				print("final pos:",self.px1)
				
			self.rate.sleep()
		self.pub1.publish(0)

#JOINT 2
		Kp = 2.0
		Kd = 1.0
		Ki = -0.0005
		error = 0.0
		sum_error = 0.0
		prev_error = 0.0
		while (self.joint2_flag == 0): 
			prev_error = error

			error = (self.destination_joint_2 - self.px2)
			delta_error = error - prev_error #(e(t) − e(t − ∆t))
			sum_error = sum_error + error*(prev_error)
			
			if(prev_error != 0):
				ut = (Kp*error + Kd*(delta_error/prev_error) + Ki*sum_error)
			else:
				ut = (Kp*error + Kd*(delta_error) + Ki*sum_error)
			
			if (ut > 1.0):
				ut = 0.5
			elif (ut < -0.5):
				ut = -0.5
			elif (ut < 1 and ut > 0):
				ut = 0.2
			elif (ut < 0 and ut > -0.5):
				ut = -0.2
			
			self.pub2.publish(ut)
			self.pub1.publish(0)
			self.pub3.publish(0)

			if ((self.px2 > (self.destination_joint_2 - 0.05)) and (self.px2 < (self.destination_joint_2 + 0.05))):
				self.joint2_flag = 1
				print("final pos:",self.px2)
		
		
			self.rate.sleep()
		self.pub2.publish(0)
	
#JOINT 3
		error = 0.0
		sum_error = 0.0
		prev_error = 0.0
		Kp = 750.0
		Kd = 5.0
		Ki = -0.5
		while (self.joint3_flag == 0): 
			prev_error = error
			error = (self.destination_joint_3 - self.px3)
			delta_error = error - prev_error #(e(t) − e(t − ∆t))
			sum_error = sum_error + error*(prev_error)

			if(prev_error != 0):
				ut = (Kp*error + Kd*(delta_error/prev_error) + Ki*sum_error)
			else:
				ut = (Kp*error + Kd*(delta_error) + Ki*sum_error)

			self.pub3.publish(ut)
			self.pub2.publish(0)
			self.pub1.publish(0)

			if ((self.px3 > (self.destination_joint_3 - 0.002)) and (self.px3 < (self.destination_joint_3 + 0.002))):
				self.joint3_flag = 1
				print("final pos:",self.px3)
				self.rate.sleep()
		self.pub3.publish(0)
		rospy.sleep(1)
		return True
		
			
if __name__ == '__main__':

	position1 = [0.0, 0.77, 0.34]
	position2 = [-0.345, 0.425, 0.24]
	position3 = [-0.67, -0.245, 0.14]
	position4 = [0.77, 0.0, 0.39]

	drive = CommandToJointState()
	a = drive.CalculateInverseAndPID(position1) 
	time.sleep(1.0)
	if a == True:
		drive2 = CommandToJointState()
		b = drive2.CalculateInverseAndPID(position2) 
		time.sleep(1.0)
	if b == True: 
		drive3 = CommandToJointState()
		c = drive3.CalculateInverseAndPID(position3) 
		time.sleep(1.0)
	if c == True:
		drive4 = CommandToJointState()
		drive4.CalculateInverseAndPID(position4) 
		time.sleep(1.0)
		