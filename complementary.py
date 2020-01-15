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

def nomalizeQuaternion(q0, q1, q2, q3):
    norm = np.linalg.norm([q0, q1, q2, q3])
    q0 = q0 / norm
    q1 = q1 / norm
    q2 = q2 / norm
    q3 = q3 / norm
    return q0, q1, q2, q3

def rotateVectorByQuaternion(x, y, z, q0, q1, q2, q3):
    vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z
    vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z
    vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z
    return vx, vy, vz

class complement_Filter:
	def __init__(self):
		self.IsBias = False
		self.numBias = 0
		# number of data for measuring bias of gyroscope
		self.goalBias = 100 
		self.g_xSum = 0
		self.g_ySum = 0
		self.g_zSum = 0
		self.g_xBias = 0
		self.g_yBias = 0
		self.g_zBias = 0
		self.q0 = 1
		self.q1 = 0
		self.q2 = 0
		self.q3 = 0
		self.Alpha = 0.5
		self.Beta = 0.5
		self.a_x = 0
		self.a_y = 0
		self.a_z = 1
		self.g_x = 0
		self.g_y = 0
		self.g_z = 0
		self.m_x = 0
		self.m_y = 0
		self.m_z = 0
		self.t_prev = time.time()
		while self.IsBias == False:
			rospy.Subscriber("/imu_raw", Imu,self.imu_raw_data)
			self.getGyroCali()
			rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)
			
			rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)

	def calcDT(self):
		t_now = time.time()
		self.dt = t_now - self.t_prev
		self.t_prev = t_now

	def imu_raw_data(self,msg):
		self.a_x = msg.linear_acceleration.x
		self.a_y = msg.linear_acceleration.y
		self.a_z = msg.linear_acceleration.z
		self.g_x = msg.angular_velocity.x
		self.g_y = msg.angular_velocity.y
		self.g_z = msg.angular_velocity.z

	def mag_raw_data(self,msg):
		self.m_x=msg.magnetic_field.x
		self.m_y=msg.magnetic_field.y
		self.m_z=msg.magnetic_field.z

	def getGyroCali(self):
		self.g_xSum = self.g_xSum + self.g_x
		self.g_ySum = self.g_ySum + self.g_y
		self.g_zSum = self.g_zSum + self.g_z
		if self.numBias == self.goalBias :
			self.IsBias = True
			self.g_xBias = self.g_xSum/self.goalBias
			self.g_yBias = self.g_ySum/self.goalBias
			self.g_zBias = self.g_zSum/self.goalBias

	def getMagCali(self):


	def getAccelGyro(self):
		rospy.Subscriber("/imu_raw",Imu,self.imu_raw_data)
		self.g_x = self.g_x - self.g_xBias
		self.g_y = self.g_y - self.g_yBias
		self.g_z = self.g_z - self.g_zBias
	
	def getMagnetic(self):
		rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)
		
		

	def getPrediction(self):
		self.calcDT()
		self.getAccelGyro()
		q0_gyro,q1_gyro,q2_gyro,q3_gyro = quaternionMultiplication(0, self.g_x, self.g_y, self.g_z, self.q0, self.q1, self.q2, self.q3)
		q0_gyro = self.q0 - 0.5 * q0_gyro * self.dt
		q1_gyro = self.q1 - 0.5 * q1_gyro * self.dt
		q2_gyro = self.q2 - 0.5 * q2_gyro * self.dt
		q3_gyro = self.q3 - 0.5 * q3_gyro * self.dt
		return q0_gyro, q1_gyro, q2_gyro, q3_gyro
		
	def gainFunction(self, ax, ay, az):
		norm = sqrt(ax ** 2 + ay ** 2 + az ** 2)
		error = abs(norm - 1)
		if error < 0.1: 
			return 1
		elif error < 0.2:
			return 1 - 5 * error
		else:
			return 0
	
	def Acc_Correction(self):
		alpha = self.gainAcc(self.a_x, self.a_y, self.a_z) * self.Alpha
		gx, gy, gz = rotateVectorQuaternion(self.a_x, self.a_y, self.a_z, self.q0, -self.q1, -self.q2, -self.q3)
		q0_acc = alpha * sqrt(0.5 * (gx + 1)) + (1-alpha)
		q1_acc = alpha * (-gy / sqrt(2 * (gz + 1)))
		q2_acc= q2 = alpha * (gx / sqrt(2 * (gz + 1)))
		q3_acc = 0
		q0_acc, q1_acc, q2_acc, q3_acc = normalizeQuaternion(q0_acc, q1_acc, q2_acc, q3_acc)
		return q0_acc, q1_acc, q2_acc, q3_acc

	def Mag_Correction(self):
		self.getMagnetic()
		beta = self.gainMag(self.m_x, self.m_y, self.m_z) * self.Beta
		lx, ly, lz = rotate(self.m_x, self.m_y, self.m_z, self.q0, - self.q1, - self.q2, - self.q3)
		gamma = lx ** 2 + ly ** 2
		q0_mag = beta * sqrt(gamma + lx * sqrt(gamma))/ sqrt(2 * gamma) + (1 - beta)
		q1_mag = 0
		q2_mag = 0
		q3_mag = beta * ly / sqrt(2 * (gamma + lx * sqrt(gamma)))
		q0_mag, q1_mag, q2_mag, q3_mag= normalizeQuaternion(q0_mag, q1_mag, q2_mag, q3_mag)
		return q0_mag, q1_mag, q2_mag, q3_mag
		
	def Imu_Mag_Complementary(self):
		q0_gyro, q1_gyro, q2_gyro, q3_gyro = self.getPrediction()
		q0_acc, q1_acc, q2_acc, q3_acc = self.Acc_Correction()
		self.q0, self.q1, self.q2, self.q3 = quaternionMultiplication(q0_gyro, q1_gyro, q2_gyro, q3_gyro, q0_acc, q1_acc, q2_acc, q3_acc)
		q0_mag, q1_mag, q2_mag, q3_mag = self.Mag_Correction()
		self.q0, self.q1, self.q2, self.q3 = quaternionMultiplication(self.q0, self.q1, self.q2, self.q3, q0_mag, q1_mag, q2_mag, q3_mag)
		print(str(self.q0) + str(self.q1) + str(self.q2) + str(self.q3))

if __name__=="__main__":
	rospy.init_node("Complementary", anonymous = True)
	rospy.loginfo("starting Complementary Filter")
	complementary = complement_Filter()
	
	try:
		rospy.loginfo("complementary filter start!")
		complement_Filter.Imu_Mag_Complementary()
	except rospy.ROSInterruptException:
		print "ROS terminated"
		pass
		

		
		

	
