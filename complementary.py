#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from numpy import sqrt, pi
import tf
import time

import smbus
import numpy as np
import os, sys

def quaternionMultiplication(p0, p1, p2, p3, q0, q1, q2, q3):
    r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3
    r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2
    r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1
    r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0
    return r0, r1, r2, r3

def normalizeQuaternion(q0, q1, q2, q3):
    norm = np.linalg.norm([q0, q1, q2, q3])
    q0 = q0 / norm
    q1 = q1 / norm
    q2 = q2 / norm
    q3 = q3 / norm
    return q0, q1, q2, q3

def rotateVectorQuaternion(x, y, z, q0, q1, q2, q3):
    vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z
    vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z
    vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z
    return vx, vy, vz

class complement_Filter:
	def __init__(self):
		rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
		rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)
		self.g_xBias, self.g_yBias, self.g_zBias = 0, 0, 0
		self.m_xBias, self.m_yBias, self.m_zBias = 0, 0, 0
		self.m_xScale, self.m_yScale, self.m_zScale = 0, 0 ,0
		self.Alpha, self.Beta = 0.5, 0.5
		self.q0, self.q1, self.q2, self.q3 = 1, 0, 0, 0
		self.a_x, self.a_y, self.a_z = 0, 0, 1
		self.g_x, self.g_y, self.g_z = 0, 0, 0
		self.m_x, self.m_y, self.m_z = 0, 0, 0
		self.t_prev = time.time()
		self.frequency = 0.5
		self.Calibration(100)

	def calcDT(self):
		t_now = time.time()
		self.dt = t_now - self.t_prev
		self.t_prev = t_now

	def imu_raw_data(self, msg):
		self.a_x = msg.linear_acceleration.x
		self.a_y = msg.linear_acceleration.y
		self.a_z = msg.linear_acceleration.z
		self.g_x = msg.angular_velocity.x
		self.g_y = msg.angular_velocity.y
		self.g_z = msg.angular_velocity.z
		
	def mag_raw_data(self, msg):
		self.m_x = msg.magnetic_field.x
		self.m_y = msg.magnetic_field.y
		self.m_z = msg.magnetic_field.z

	def Calibration(self, goal):
		IsBias = False
		numBias = 0
		goalBias = goal
		# number of data for measuring bias of gyroscope 
		g_x_sum, g_y_sum, g_z_sum = 0, 0, 0
		m_x_max, m_y_max, m_z_max = 0, 0, 0
		m_x_min, m_y_min, m_z_min = 0, 0, 0
		while IsBias == False :
			if goalBias == 100 :
				m_x_max, m_y_max, m_z_max, m_x_min, m_y_min, m_z_min = self.getMagCali(m_x_max, m_y_max, m_z_max, m_x_min, m_y_min, m_z_min)
			g_x_sum, g_y_sum, g_z_sum, numBias = self.getGyroCali(g_x_sum, g_y_sum, g_z_sum, numBias, goalBias)

	def getGyroCali(self, sum_x, sum_y, sum_z, num, goal):
		sum_x = sum_x + self.g_x
		sum_y = sum_y + self.g_y
		sum_z = sum_z + self.g_z
		num = num + 1
		if num == goal:
			self.IsBias = True
			self.g_xBias = sum_x / goal
			self.g_yBias = sum_y / goal
			self.g_zBias = sum_z / goal
		return sum_x, sum_y, sum_z, num

	def getMagCali(self,max_x,max_y,max_z,min_x,min_y,min_z):
		if self.m_x > max_x:
			max_x = self.m_x
		if self.m_x < min_x:
			min_x = self.m_x
		if self.m_y > max_y:
			max_y = self.m_y
		if self.m_y < min_y:
			min_y = self.m_y
		if self.m_z > max_z:
			max_z = self.m_z
		if self.m_z < min_z:
			min_z = self.m_z
		if num == goal:
			self.m_xBias = (max_x + min_x) / 2
			self.m_yBias = (max_y + min_y) / 2
			self.m_zBias = (max_z + min_z) / 2
			avg = (max_x - min_x + max_y - min_y + max_z - min_z) / 3
			self.m_xScale = avg / (max_x - min_x)
			self.m_yScale = avg / (max_y - min_y)
			self.m_zScale = avg / (max_z - min_z)
		return max_x, max_y, max_z, min_x, min_y, min_z

	def getAccelGyro(self):
		self.g_x = self.g_x - self.g_xBias
		self.g_y = self.g_y - self.g_yBias
		self.g_z = self.g_z - self.g_zBias

	def getMagnetic(self):
		self.m_x = (self.m_x - self.m_xBias) * self.scale_x
		self.m_y = (self.m_y - self.m_yBias) * self.scale_y
		self.m_z = (self.m_z - self.m_zBias) * self.scale_z

	def getPrediction(self):
		self.calcDT()
		self.getAccelGyro()
		q0_gyro,q1_gyro,q2_gyro,q3_gyro = quaternionMultiplication(0, self.g_x, self.g_y, self.g_z, self.q0, self.q1, self.q2, self.q3)
		q0_gyro = self.q0 - 0.5 * q0_gyro * self.dt
		q1_gyro = self.q1 - 0.5 * q1_gyro * self.dt
		q2_gyro = self.q2 - 0.5 * q2_gyro * self.dt
		q3_gyro = self.q3 - 0.5 * q3_gyro * self.dt
		return q0_gyro, q1_gyro, q2_gyro, q3_gyro
		
	def gainFunction(self, a, b, c):
		norm = sqrt(a ** 2 + b ** 2 + c ** 2)
		error = abs(norm - 1)
		# d ==0 : Acceleration case, d ==0 : Magnetic case
		if error < 0.1: 
			return 1
		elif error < 0.2:
			return 1 - 5 * error
		else:
			return 0
	
	def acc_Correction(self):
		alpha = self.gainAcc(self.a_x, self.a_y, self.a_z) * self.Alpha
		gx, gy, gz = rotateVectorQuaternion(self.a_x, self.a_y, self.a_z, self.q0, -self.q1, -self.q2, -self.q3)
		q0_acc = alpha * sqrt(0.5 * (gx + 1)) + (1 - alpha)
		q1_acc = alpha * (-gy / sqrt(2 * (gz + 1)))
		q2_acc = alpha * (gx / sqrt(2 * (gz + 1)))
		q3_acc = 0
		q0_acc, q1_acc, q2_acc, q3_acc = normalizeQuaternion(q0_acc, q1_acc, q2_acc, q3_acc)
		return q0_acc, q1_acc, q2_acc, q3_acc

	def mag_Correction(self):
		self.getMagnetic()
		beta = self.gainMag(self.m_x, self.m_y, self.m_z) * self.Beta
		lx, ly, lz = rotateVectorQuaternion(self.m_x, self.m_y, self.m_z, self.q0, -self.q1, -self.q2, -self.q3)
		gamma = lx ** 2 + ly ** 2
		q0_mag = beta * sqrt(gamma + lx * sqrt(gamma))/ sqrt(2 * gamma) + (1 - beta)
		q1_mag = 0
		q2_mag = 0
		q3_mag = beta * ly / sqrt(2 * (gamma + lx * sqrt(gamma)))
		q0_mag, q1_mag, q2_mag, q3_mag = normalizeQuaternion(q0_mag, q1_mag, q2_mag, q3_mag)
		return q0_mag, q1_mag, q2_mag, q3_mag

	def steadyState(self):
		norm = sqrt(self.g_x ** 2 + self.g_y ** 2 + self.g_z ** 2)
		if norm < 0.2
			return True
		return False
		
	def imu_Mag_Complementary(self):
		while not rospy.is_shutdown():
			if self.steadyState() == True:
				Calibration(10)
			q0_gyro, q1_gyro, q2_gyro, q3_gyro = self.getPrediction()
			q0_acc, q1_acc, q2_acc, q3_acc = self.acc_Correction()
			self.q0, self.q1, self.q2, self.q3 = quaternionMultiplication(q0_gyro, q1_gyro, q2_gyro, q3_gyro, q0_acc, q1_acc, q2_acc, q3_acc)
			q0_mag, q1_mag, q2_mag, q3_mag = self.mag_Correction()
			self.q0, self.q1, self.q2, self.q3 = quaternionMultiplication(self.q0, self.q1, self.q2, self.q3, q0_mag, q1_mag, q2_mag, q3_mag)
			print(str(self.q0) + str(self.q1) + str(self.q2) + str(self.q3))

if __name__=="__main__":
	rospy.init_node("Complementary", anonymous = True)
	rospy.loginfo("starting Complementary Filter")
	complement = complement_Filter()
	try:
		rospy.loginfo("complementary filter start!")
		complement.imu_Mag_Complementary()
		rospy.sleep(self.frequency)
	except rospy.ROSInterruptException:
		print "ROS terminated"
		pass
		

		
		

	
