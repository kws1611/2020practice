#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import tf

import smbus
import numpy as np
from numpy import sqrt, pi
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

def rotateQuaternion(x, y, z, q0, q1, q2, q3):
    vx = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*x + 2*(q1*q2 - q0*q3)*y + 2*(q1*q3 + q0*q2)*z
    vy = 2*(q1*q2 + q0*q3)*x + (q0*q0 - q1*q1 + q2*q2 - q3*q3)*y + 2*(q2*q3 - q0*q1)*z
    vz = 2*(q1*q3 - q0*q2)*x + 2*(q2*q3 + q0*q1)*y + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*z
    return vx, vy, vz

Gravity = 9.81
AngularVelocityThreshold = rospy.get_param("/complementary/AngularVelocityThreshold")
AccelerationThreshold = rospy.get_param("/complementary/AccelerationThreshold")
DeltaAngularVelocityThreshold = rospy.get_param("/complementary/DeltaAngularVelocityThreshold")

class complementary(self, msg):
	def __init__(self):
		rospy.subscriber("/imu_raw", Imu, self.imu_raw_cb)
		rospy.subscriber("/mag_raw", MagneticField, self.mag_raw_cb)

	def imu_raw_data(self, msg):
		self.imu_raw = msg

		self.acc_x = self.imu_raw.linear_acceleration.x
		self.acc_y = self.imu_raw.linear_acceleration.y
		self.acc_z = self.imu_raw.linear_acceleration.z

		self.gyro_x = self.imu_raw.angular_velocity.x
		self.gyro_y = self.imu_raw.angular_velocity.y
		self.gyro_z = self.imu_raw.angular_velocity.z

	def mag_raw_data(self, msg):
		self.mag_raw = msg
		
		self.mag_x = self.magnetic_field.x
		self.mag_y = self.magnetic_field.y
		self.mag_z = self.magnetic_field.z

	def Prediction(self, wx, wy, wz):
		wx = wx - wx_bias
		wy = wy - wy_bias
		wz = wz - wz_bias
		q0_pre,q1_pre,q2_pre,q3_pre = -0.5 * multipleQuaternion(0,wx,wy,yz,q0,q1,q2,q3) * dt
		np.matrix('q0_pre q1_pre q2_pre q3_pre')=np.matrix('q0_pre q1_pre q2_pre q3_pre')+np.matrix('q0 q1 q2 q3')
	def correctionAcc():
		alpha = gainFunction(0, ax, ay, az) * Alpha
		normalizeQuaternion(0, ax, ay, az)
		g_x=rotateQuaternion(ax, ay, az, q0_pre, q1_pre, q2_pre, q3_pre)
		


if __name__ == "__main__":
    rospy.init_node("Complementary_Filter", anonymous=True)
    rospy.loginfo("Complementary filter node initialized")


    try:

    except rospy.ROSInterruptException:
        print "ROS terminated"
        pass
	

