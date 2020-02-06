#!/usr/bin/env python

import rospy
import numpy as np
import tf
import time
import smbus
import os, sys
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from numpy import sqrt, pi

def quaternionMultiplication(p0, p1, p2, p3, q0, q1, q2, q3):
	r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3
	r1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2
	r2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1
	r3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0
	return r0, r1, r2, r3

def norm(q0, q1, q2, q3):
	return sqrt(q0**2 + q1**2 + q2**2 + q3**2)

def normalizeQuaternion(q0, q1, q2, q3):
	norm = sqrt(q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2)
	q0 = q0 / norm
	q1 = q1 / norm
	q2 = q2 / norm
	q3 = q3 / norm
	return q0, q1, q2, q3

class error_calculating:
	def __init__(self):
		rospy.Subscriber("/vrpn_client_node/quad_imu_2/pose", PoseStamped, self.get_cap_data)
		rospy.Subscriber("/quat", PoseWithCovarianceStamped, self.get_comp_data)
		#rospy.Subscriber("/Kalman_quat", PoseWithCovarianceStamped, self.get_kal_data)
		self.q0_init, self.q1_init, self.q2_init, self.q3_init = self.initialize()
		self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap = 1., 0., 0., 0.
		self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp = 1., 0., 0., 0.
		self.q0_kal, self.q1_kal, self.q2_kal, self.q3_kal = 1., 0., 0., 0.
		self.rate = rospy.Rate(100)

	def initialize(self):
		while (self.q0_cap == 1 and self.q1_cap == 0 and self.q2_cap == 0 and self.q3_cap ==0 ) == True:
			rospy.sleep(0.5)
		return self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap

	def get_cap_data(self, msg):
		self.q0_cap = msg.pose.orientation.x
		self.q1_cap = msg.pose.orientation.y
		self.q2_cap = msg.pose.orientation.z
		self.q3_cap = msg.pose.orientation.w

	def get_comp_data(self, msg):
		self.q0_comp = msg.pose.pose.orientation.x
		self.q1_comp = msg.pose.pose.orientation.y
		self.q2_comp = msg.pose.pose.orientation.z
		self.q3_comp = msg.pose.pose.orientation.w

	def get_kal_data(self, msg):
		self.q0_kal = msg.pose.pose.orientation.x
		self.q1_kal = msg.pose.pose.orientation.y
		self.q2_kal = msg.pose.pose.orientation.z
		self.q3_kal = msg.pose.pose.orientation.w

	def calculate_error(self):
		while not rospy.is_shutdown():
			self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp = quaternionMultiplication(self.q0_init, self.q1_init, self.q2_init, self.q3_init, self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp)
			self.q0_kal, self.q1_kal, self.q2_kal, self.q3_kal = quaternionMultiplication(self.q0_init, self.q1_init, self.q2_init, self.q3_init, self.q0_kal, self.q1_kal, self.q2_kal, self.q3_kal)

if __name__=="__main__":
	rospy.init_node("error", anonymous = True)
	rospy.loginfo("starting calculating error")
	error = error_calculating()
	try:
		rospy.loginfo("error calculating start!")
		error.calculate_error()
	except rospy.ROSInterruptException:
		print "ROS terminated"
		pass