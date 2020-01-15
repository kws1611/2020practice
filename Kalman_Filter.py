#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
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
	result = np.matrix('q_0; q_1; q_2; q_3')
	return result

def norm_quat(a_1, a_2, a_3, a_4):
    q_0 = a_1/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
    q_1 = a_2/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
    q_2 = a_3/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2) 
    q_3 = a_4/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
    q = np.matrix('q_0; q_1; q_2; q_3')
    return q

X = np.matrix('1;0;0;0')
P = np.identity(4)
dt = 0.01
H = np.identity(4)
Q = np.matrix('0 -0.4 -0.65 -0.4; 0.4 0 0.4 -0.65; 0.65 -0.4 0 0.4; 0.4 0.65 -0.4 0')
R = 10**(-4)*np.matrix('0.5929 0 0 0;0 0.0289 0 0; 0 0 0.0289 0; 0 0 0 0.5929')

class kalman_Filter:
	def imu_raw_data(self, msg):
		self.imu_data = msg
		self.acc_x = self.imu_data.linear_acceleration.x
		self.acc_y = self.imu_data.linear_acceleration.y
		self.acc_z = self.imu_data.linear_acceleration.z

		self.gyro_x = self.imu_data.angular_velocity.x
		self.gyro_y = self.imu_data.angular_velocity.y
		self.gyro_z = self.imu_data.angular_velocity.z
	def mag_raw_data(self, msg):
		self.mag_data = msg
		self.mag_x = float(self.mag_data.magnetic_field.x)
		self.mag_y = float(self.mag_data.magnetic_field.y)
		self.mag_z = float(self.mag_data.magnetic_field.z)
	def __init__(self):
		# Subscriber created
		self.mag_x = 1
		self.mag_y = 1
		self.mag_z = 1
		self.acc_x = 1
		self.acc_y = 1
		self.acc_z = 1
		self.gyro_x = 1
		self.gyro_y = 1
		self.gyro_z = 1	
		rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
		rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)


	def get_acc_quat(self):
		
		self.ax = self.acc_x / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
		self.ay = self.acc_y / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
		self.az = self.acc_z / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
		self.q_acc = np.matrix([[math.sqrt(0.5*(self.az + 1))], [-self.ay/(2*math.sqrt(0.5*(self.az+1)))], [self.ax/(2*math.sqrt(0.5*(self.az+1)))] [0]])
	
	def get_mag_quat(self):

		self.gamma = math.sqrt(self.mag_x**2 + self.mag_y**2)
		if self.mag_x >= 0:
			self.q_mag_x = math.sqrt(self.gamma+self.mag_x*math.sqrt(self.gamma))/math.sqrt(2*self.gamma)
			self.q_mag_y = 0
			self.q_mag_z = 0
			self.q_mag_w = float(self.mag_y/(2*0.5*math.sqrt(self.gamma+self.mag_x*math.sqrt(self.gamma))))
			self.q_mag = np.matrix([self.q_mag_x, self.q_mag_y, self.q_mag_z, self.q_mag_w])
		
		if self.mag_x < 0:
			self.q_mag_x = self.mag_y/(2*0.5*math.sqrt(self.gamma-self.mag_x*math.sqrt(self.gamma)))
			self.q_mag_y = 0
			self.q_mag_z = 0
			self.q_mag_w = math.sqrt(self.gamma - self.mag_x*math.sqrt(self.gamma))/math.sqrt(2*self.gamma)
			self.q_mag = np.matrix([[self.q_mag_x], [self.q_mag_y], [self.q_mag_z], [self.q_mag_w]])
	
	def kalman(self):
	
		self.get_mag_quat()
		self.get_acc_quat()

		self.Z = quat_mult(self.q_acc[1,1],self.q_acc[1,2],self.q_acc[1,3],self.q_acc[1,4],self.q_mag[1,1],self.q_mag[1,2],self.q_mag[1,3],self.q_mag[1,4])

		self.A = np.identity(4) + dt*0.5*np.matrix('0 -self.gyro_x -self.gyro_y -self.gyro_z; self.gyro_x 0 self.gyro_z -self.gyro_y; self.gyro_y -self.gyro_z 0 self.gyro_x; self.gyro_z self.gyro_y -self.gyro_x 0 ')

		# Kalman Filter
		self.Xp = self.A*X
		self.Pp = self.A*P*self.A.T +Q
		self.K = self.Pp*H.T*lin.lnv(H*self.Pp*H.T + R)
		X = self.Xp + self.K*(self.Z - H*self.Xp)
		X = norm_quat(X[0,0],X[0,1],X[0,2],X[0,3])
		P = self.Pp - self.K*H*self.Pp
		print(X.T)
		
		
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
