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



def Sub():
	# Subscriber created
	rospy.Subscriber("/imu_raw", Imu, imu_raw_data)
	rospy.Subscriber("/mag_raw", MagneticField, mag_raw_data)
	rospy.spin()

def imu_raw_data(msg):
	print("1")

	acc_x = msg.linear_acceleration.x
	acc_y = msg.linear_acceleration.y
	acc_z = msg.linear_acceleration.z

	gyro_x = msg.angular_velocity.x
	gyro_y = msg.angular_velocity.y
	gyro_z = msg.angular_velocity.z
	print(acc_x)
	print(acc_y)
	print(acc_z)

	print(gyro_x)
	print(gyro_y)
	print(gyro_z)
def mag_raw_data(msg):
	mag_x = msg.magnetic_field.x
	mag_y = msg.magnetic_field.y
	mag_z = msg.magnetic_field.z
	print(mag_x)
	print(mag_y)
	print(mag_z)


if __name__ == "__main__":
	rospy.init_node("Test_Filter", anonymous=True)
	rospy.loginfo("Kalman filter node initialized")

	try:
		rospy.loginfo("Kalman filter start!")
		Sub()
	except rospy.ROSInterruptException:
		print "ROS terminated"
		pass

		

