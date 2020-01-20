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


class Sub:
	def __init__(self):
		self.rate = rospy.Rate(100)
		# Subscriber created
		rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
		rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)
		print("1")
		self.acc_x = 0
		self.acc_z=0
		self.acc_y=0
		self.gyro_x=0
		self.gyro_y=0
		self.gyro_z=0
		self.mag_x=0
		self.mag_y=0
		self.mag_z =0
	def imu_raw_data(self,msg):
		print("2")
		self.imu_msg=msg
		self.acc_x = self.imu_msg.linear_acceleration.x
		self.acc_y = self.imu_msg.linear_acceleration.y
		self.acc_z = self.imu_msg.linear_acceleration.z

		self.gyro_x = self.imu_msg.angular_velocity.x
		self.gyro_y = self.imu_msg.angular_velocity.y
		self.gyro_z = self.imu_msg.angular_velocity.z
		print(self.acc_x)
		print(self.acc_y)
		print(self.acc_z)

		print(self.gyro_x)
		print(self.gyro_y)
		print(self.gyro_z)
	def mag_raw_data(self,msg):
		self.mag_msg = msg
		self.mag_x = self.mag_msg.magnetic_field.x
		self.mag_y = self.mag_msg.magnetic_field.y
		self.mag_z = self.mag_msg.magnetic_field.z
		print("2-1")
		print(self.mag_x)
		print(self.mag_y)
		print(self.mag_z)

	def prints(self):
		print("3")		
		print(self.mag_x)
		print(self.gyro_x)
		self.rate.sleep()
		
if __name__ == "__main__":
	rospy.init_node("Test_Filter", anonymous=True)
	rospy.loginfo("Kalman filter node initialized")

	try:
		rospy.loginfo("Kalman filter start!")
		test = Sub()
		while not rospy.is_shutdown():

			test.prints()
			
	except rospy.ROSInterruptException:
		print "ROS terminated"
		pass

		

