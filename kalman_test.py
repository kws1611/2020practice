#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
import smbus
import numpy as np
import time
import math
import numpy.linalg as lin
import tf

def quat_mult(a_1, a_2, a_3, a_4, b_1, b_2, b_3, b_4):
	q_0 = a_1*b_1 - a_2*b_2 - a_3*b_3 - a_4*b_4
	q_1 = a_1*b_2 + a_2*b_1 + a_3*b_4 - a_4*b_3
	q_2 = a_1*b_3 - a_2*b_4 + a_3*b_1 + a_4*b_2
	q_3 = a_1*b_4 + a_2*b_3 - a_3*b_2 + a_4*b_1
	q = np.matrix([q_0, q_1, q_2, q_3])
	q = q.T
	return q

def norm_quat(a_1, a_2, a_3, a_4):
	q_0 = a_1/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
	q_1 = a_2/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
	q_2 = a_3/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2) 
	q_3 = a_4/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
	q = np.matrix([q_0, q_1, q_2, q_3])
	q = q.T
	return q
def normalization(v1, v2, v3):
	norm = math.sqrt(v1 ** 2 + v2 ** 2 + v3 ** 2)
	v1 = v1 / norm
	v2 = v2 / norm
 	v3=  v3 / norm
	return v1, v2, v3

def rotateVectorQuaternion(x, y, z, q0, q1, q2, q3):
	vx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * x + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z
	vy = 2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z
	vz = 2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z
	return vx, vy, vz

class kalman_Filter:
	
	def imu_raw_data(self, msg):
		self.imu_data = msg
		self.acc_x = float(self.imu_data.linear_acceleration.x)
		self.acc_y = float(self.imu_data.linear_acceleration.y)
		self.acc_z = float(self.imu_data.linear_acceleration.z)

		self.gyro_x = float(self.imu_data.angular_velocity.x)
		self.gyro_y = float(self.imu_data.angular_velocity.y)
		self.gyro_z = float(self.imu_data.angular_velocity.z)

	def mag_raw_data(self, msg):
		self.mag_data = msg
		self.mag_x = float(self.mag_data.magnetic_field.x)+1335.0
		self.mag_y = float(self.mag_data.magnetic_field.y)
		self.mag_z = float(self.mag_data.magnetic_field.z)

	def __init__(self):
		self.kalman_topic = Quaternion()
		self.X = np.matrix('1;0;0;0')
		self.P = np.identity(4)
		self.dt = float(1.0/100)
		self.H = np.identity(4)
		self.Q = 10**(-4)*np.matrix('0.64 0 0 0; 0 0.64 0 0 ; 0 0 0.169 0; 0 0 0 0.64 ')
		#		self.Q = np.matrix('0 -0.4 -0.65 -0.4; 0.4 0 0.4 -0.65; 0.65 -0.4 0 0.4; 0.4 0.65 -0.4 0')
		self.R = 10**(-4)*np.matrix('0.5929 0 0 0;0 0.0289 0 0; 0 0 0.0289 0; 0 0 0 0.5929')
		# Subscriber created
		self.mag_x = 0.01
		self.mag_y = 0.01
		self.mag_z = 0.01
		self.acc_x = 0.01
		self.acc_y = 0.01
		self.acc_z = 0.01
		self.gyro_x = 0.01
		self.gyro_y = 0.01
		self.gyro_z = 0.01
		self.rate = rospy.Rate(100)
		rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
		rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)
		self.Kalman_cov_pub = rospy.Publisher("/pose_covariance",PoseWithCovarianceStamped, queue_size=1)
		self.Kalman_pub = rospy.Publisher("/Kalman_quat",Quaternion, queue_size=1)

	def get_acc_quat(self):
		
		self.ax = self.acc_x / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
		self.ay = self.acc_y / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
		self.az = self.acc_z / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
		self.q_acc = np.matrix([math.sqrt(0.5*(self.az + 1)), -self.ay/(2*math.sqrt(0.5*(self.az+1))), self.ax/(2*math.sqrt(0.5*(self.az+1))), 0])
	
	def get_mag_quat(self):
		self.mag_x, self.mag_y, self.mag_z = normalization(self.mag_x, self.mag_y, self.mag_z)
		lx, ly, lz = rotateVectorQuaternion(self.mag_x, self.mag_y, self.mag_z, self.q_acc[0,0], self.q_acc[0,1], self.q_acc[0,2], self.q_acc[0,3])
		self.gamma = lx ** 2 + ly ** 2
		if lx >= 0:
			self.q0_mag = math.sqrt(self.gamma + lx * math.sqrt(self.gamma))/ math.sqrt(2 * self.gamma)
			self.q1_mag = 0
			self.q2_mag = 0
			self.q3_mag = ly / math.sqrt(2 * (self.gamma + lx * math.sqrt(self.gamma)))
			self.q_mag= norm_quat(self.q0_mag, self.q1_mag, self.q2_mag, self.q3_mag)
		if lx < 0:
			self.q0_mag = ly / math.sqrt(2 * (self.gamma - lx * math.sqrt(self.gamma)))
			self.q1_mag = 0
			self.q2_mag = 0
			self.q3_mag = math.sqrt(self.gamma - lx * math.sqrt(self.gamma))/ math.sqrt(2 * self.gamma) 
			self.q_mag= norm_quat(self.q0_mag, self.q1_mag, self.q2_mag, self.q3_mag)
	

	def kalman(self):
		pose_topic = PoseWithCovarianceStamped()
		kalman_topic = Quaternion()
				
		self.get_acc_quat()
		self.get_mag_quat()

		self.Z = quat_mult(self.q_acc[0,0],self.q_acc[0,1],self.q_acc[0,2],self.q_acc[0,3],self.q_mag[0,0],self.q_mag[1,0],self.q_mag[2,0],self.q_mag[3,0])
			
		self.A = np.identity(4) + self.dt*float(0.5)*np.matrix([[0 ,-self.gyro_x,-self.gyro_y,-self.gyro_z], [self.gyro_x, 0 ,self.gyro_z, -self.gyro_y], [self.gyro_y, -self.gyro_z ,0, self.gyro_x], [self.gyro_z, self.gyro_y ,-self.gyro_x, 0 ]])
		# Kalman Filter
		
		self.Xp = self.A*self.X
		self.Pp = self.A*self.P*self.A.T +self.Q
		self.K = self.Pp*self.H.T*lin.inv(self.H*self.Pp*self.H.T + self.R)
		self.X = self.Xp + self.K*(self.Z - self.H*self.Xp)
		self.X = norm_quat(self.X[0,0],self.X[1,0],self.X[2,0],self.X[3,0])
		self.P = self.Pp - self.K*self.H*self.Pp

		kalman_topic.x = self.X[0,0]
		kalman_topic.y = self.X[1,0]
		kalman_topic.z = self.X[2,0]
		kalman_topic.w = self.X[3,0]
		pose_topic.header.stamp = rospy.Time.now()
		pose_topic.header.frame_id = "map"
		pose_topic.pose.pose.position.x = 0
		pose_topic.pose.pose.position.y = 0
		pose_topic.pose.pose.position.z = 0
		pose_topic.pose.pose.orientation.x = self.X[1,0]
		pose_topic.pose.pose.orientation.y = self.X[2,0]
		pose_topic.pose.pose.orientation.z = self.X[3,0]
		pose_topic.pose.pose.orientation.w = self.X[0,0]
		pose_topic.pose.covariance = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0289, 0.0289, 0.1207,0,0,0,0.0289, 0.0289, 0.1207,0,0,0,0.1207,0.1207,0.5041]
		self.Kalman_pub.publish(kalman_topic)
		self.Kalman_cov_pub.publish(pose_topic)
		self.rate.sleep()
		
		
if __name__ == "__main__":

	rospy.init_node("Kalman_Filter", anonymous=True)
	rospy.loginfo("Kalman filter node initialized")

	try:
		rospy.loginfo("Kalman filter start!")
	
		Filtering = kalman_Filter()

		while not rospy.is_shutdown():

			 
			Filtering.kalman()
	except rospy.ROSInterruptException: 
		print "ROS terminated"
		pass