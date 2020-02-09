#!/usr/bin/env python

import rospy
import numpy as np
import tf
import time
import smbus
import math
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from att_est.msg import error_msg
from numpy import sqrt, pi

def quaternionMultiplication(p0, p1, p2, p3, q0, q1, q2, q3):
	r0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3
	r1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2
	r2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1
	r3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0
	return r0, r1, r2, r3

def normalizeQuaternion(q0, q1, q2, q3):
	norm = sqrt(q0 ** 2 + q1 ** 2 + q2 ** 2 + q3 ** 2)
	q0 = q0 / norm
	q1 = q1 / norm
	q2 = q2 / norm
	q3 = q3 / norm
	return q0, q1, q2, q3

def rotateVectorQuaternion(x, y, z, q0, q1, q2, q3):
	vx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * x + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z
	vy = 2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z
	vz = 2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z
	return vx, vy, vz

def quaternionToEuler(q0, q1, q2, q3):
	roll = np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2))
	pitch = np.arcsin(2 * (q0 * q2 - q3 * q1))
	yaw = np.arctan2(2 * (q0 * q3 + q1 * q2),1 - 2  * (q2 ** 2 + q3 ** 2))
	return roll, pitch, yaw

class error_calculating:
	def __init__(self):
		self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap = 0., 0., 0., 0.
		self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp = 0., 0., 0., 0.
		self.q0_init, self.q1_init, self.q2_init, self.q3_init = 0., 0., 0., 0.
		self.isInit = False
		rospy.Subscriber("/vrpn_client_node/quad_imu_2/pose", PoseStamped, self.get_cap_data)
		#rospy.Subscriber("pose_covariance", PoseWithCovarianceStamped, self.get_comp_data)
		rospy.Subscriber("/quat", PoseWithCovarianceStamped, self.get_comp_data)
		while self.q0_comp == 0.0 or self.q0_cap == 0.0:
			time.sleep(0.1)
		self.pub = rospy.Publisher("/cap", PoseWithCovarianceStamped, queue_size = 1)
		self.error_pub = rospy.Publisher("/error", error_msg, queue_size = 1)
		self.rate = rospy.Rate(100)
		self.initialize()

	def get_cap_data(self, msg):
		self.time_cap = msg.header.stamp.secs + msg.header.stamp.nsecs*10**(-9)
		self.q0_cap = msg.pose.orientation.w
		self.q1_cap = msg.pose.orientation.x
		self.q2_cap = msg.pose.orientation.y
		self.q3_cap = msg.pose.orientation.z
		if self.isInit == True:
			self.cap_data()

	def get_comp_data(self, msg):
		self.time_comp = msg.header.stamp.secs + msg.header.stamp.nsecs*10**(-9)
		self.q0_comp = msg.pose.pose.orientation.w
		self.q1_comp = msg.pose.pose.orientation.x
		self.q2_comp = msg.pose.pose.orientation.y
		self.q3_comp = msg.pose.pose.orientation.z

	def initialize(self):
		q0_sum, q1_sum, q2_sum, q3_sum = 0.0, 0.0, 0.0, 0.0
		t_prev = time.time()
		t_now = time.time()
		num = 0
		while t_now - t_prev < 2:
		        q0_sum += self.q0_cap
		        q1_sum += self.q1_cap
		        q2_sum += self.q2_cap
		        q3_sum += self.q3_cap
		        num += 1
		        t_now = time.time()
		self.q0_init = q0_sum / num
		self.q1_init = q1_sum / num
		self.q2_init = q2_sum / num
		self.q3_init = q3_sum / num
		self.isInit = True

	def cap_data(self):
		self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap = quaternionMultiplication(self.q0_init, -self.q1_init, -self.q2_init, -self.q3_init, self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap) 

	def eulerError(self, q0, q1, q2, q3, p0, p1, p2, p3):
		r0, p0, y0 = quaternionToEuler(q0, q1, q2, q3)
		r1, p1, y1 = quaternionToEuler(p0, p1, p2, p3)
		roll = r0 - r1
		pitch = p0 - p1
		yaw = y0 - y1
		return roll, pitch, yaw

	def vectorError(self, q0, q1, q2, q3, p0, p1, p2, p3):
		dx = self.pointError(1, 0, 0, q0, q1, q2, q3, p0, p1, p2, p3)
		dy = self.pointError(0, 1, 0, q0, q1, q2, q3, p0, p1, p2, p3)
		dz = self.pointError(0, 0, 1, q0, q1, q2, q3, p0, p1, p2, p3)
		size = sqrt(dx ** 2 + dy ** 2 + dz ** 2)
		return size

	def pointError(self, x, y, z, q0, q1, q2, q3, p0, p1, p2, p3):
		x0, y0, z0 = rotateVectorQuaternion(x, y, z, q0, q1, q2, q3)
		x1, y1, z1 = rotateVectorQuaternion(x, y, z, p0, p1, p2, p3)
		x = x0 - x1
		y = y0 - y1
		z = z0 - z1
		return sqrt(x ** 2 + y ** 2 + z ** 2)

	def quaternionError(self, q0, q1, q2, q3, p0, p1, p2, p3):
		q0, q1, q2, q3 = quaternionMultiplication(q0, q1, q2, q3, p0, - p1, - p2, - p3)
		return q0, q1, q2, q3

	def calculate_error(self):
			error_topic = error_msg()
			error_topic.time = self.time_comp
			error_topic.mot_time = self.time_cap

			error_topic.size = self.vectorError(self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap, self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp)

			error_topic.roll, error_topic.pitch, error_topic.yaw = quaternionToEuler(self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp)
			error_topic.mot_roll, error_topic.mot_pitch, error_topic.mot_yaw = quaternionToEuler(self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap)
			error_topic.err_roll, error_topic.err_pitch, error_topic.err_yaw = self.eulerError(self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap, self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp)

			error_topic.quat_w, error_topic.quat_x, error_topic.quat_y, error_topic.quat_z = self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp
			error_topic.err_quat_w, error_topic.err_quat_x, error_topic.err_quat_y, error_topic.err_quat_z = self.quaternionError(self.q0_cap, self.q1_cap, self.q2_cap, self.q3_cap, self.q0_comp, self.q1_comp, self.q2_comp, self.q3_comp)

			quat_topic = PoseWithCovarianceStamped()
			quat_topic.header.stamp = rospy.Time.now()
			quat_topic.header.frame_id = "world"
			quat_topic.pose.pose.position.x = 0
			quat_topic.pose.pose.position.y = 0
			quat_topic.pose.pose.position.z = 0
			quat_topic.pose.pose.orientation.x = self.q1_cap
			quat_topic.pose.pose.orientation.y = self.q2_cap
			quat_topic.pose.pose.orientation.z = self.q3_cap
			quat_topic.pose.pose.orientation.w = self.q0_cap
			self.pub.publish(quat_topic)

			self.error_pub.publish(error_topic)
			self.rate.sleep()


if __name__=="__main__":
	rospy.init_node("error", anonymous = True)
	rospy.loginfo("starting calculating error")
	error = error_calculating()
	try:
		rospy.loginfo("error calculating start!")
		while not rospy.is_shutdown():
			error.calculate_error()
	except rospy.ROSInterruptException:
		print "ROS terminated"
		pass
