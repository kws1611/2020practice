#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
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

class error:

	def __init__(self):
		# Subscriber created
		self.rate = rospy.Rate(100)
		rospy.Subscriber("/pose_covariance", PoseWithCovarianceStamped, self.kalman_cb)
                #rospy.Subscriber("/quat", PoseWithCovarianceStamped, self.comp_cb)
		rospy.Subscriber("/vrpn_client_node/Quadcopter/pose",PoseStamped,self.motion_cb)
                self.error_kalman_pub = rospy.Publisher("/kalman_error",PoseStamped, queue_size=1)
                #self.error_comp_pub = rospy.Publisher("/comp_error",Quaternion, queue_size=1)
                self.kal_X= 0.0
                self.kal_Y= 0.0
                self.kal_z= 0.0
                self.kal_w= 0.0
                self.motion_x = 0.0
                self.motion_Y = 0.0
                self.motion_Z = 0.0
                self.motion_W = 0.0

	def motion_cb(self,msg):
		self.mot_msg = msg
		self.motion_x = self.mot_msg.pose.orientation.x
		self.motion_y = self.mot_msg.pose.orientation.y
		self.motion_z = self.mot_msg.pose.orientation.z
		self.motion_w = self.mot_msg.pose.orientation.w

	def kalman_cb(self,msg):
		self.kalman_msg = msg
                self.kal_x=self.kalman_msg.pose.pose.orientation.x
                self.kal_y=self.kalman_msg.pose.pose.orientation.y
                self.kal_z=self.kalman_msg.pose.pose.orientation.z
                self.kal_w=self.kalman_msg.pose.pose.orientation.w

	def comp_cb(self,msg):
		self.comp_msg = msg
		self.comp_x=self.comp_msg.pose.pose.orientation.x
		self.comp_y=self.comp_msg.pose.pose.orientation.y
		self.comp_z=self.comp_msg.pose.pose.orientation.z
		self.comp_w=self.comp_msg.pose.pose.orientation.w

	def error_cal(self):
                error_kal_topic = PoseStamped()
                #error_camp_topic = Quaternion()

		# error calculating
                self.Kal_error = quat_mult(self.Kal_w,self.Kal_x,self.Kal_y,self.Kal_z,self.motion_w,self.motion_x,self.motion_y,self.motion_z)
                #self.comp_error = quat_mult(self.comp_w,self.comp_x,self.comp_y,self.comp_z,self.motion_w,self.motion_x,self.motion_y,self.motion_z)
                error_kal_topic.header.frame_id = "world"
                error_kal_topic.pose.position.x = 0
                error_kal_topic.pose.position.y = 0
                error_kal_topic.pose.position.z = 0
                error_kal_topic.pose.orientation.x = self.kal_error[1,0]
                error_kal_topic.pose.orientation.y = self.kal_error[2,0]
                error_kal_topic.pose.orientation.z = self.kal_error[3,0]
                error_kal_topic.pose.orientation.w = self.kal_error[0,0]
                """
		error_comp_topc.x = self.comp_error[1,0]
		error_comp_topc.y = self.comp_error[2,0]
		error_comp_topc.z = self.comp_error[3,0]
		error_comp_topc.w = self.comp_error[0,0]
                """

		self.error_kal_pub.publish(error_kal_topic)
                #self.error_comp_pub.publish(error_comp_topic)
		self.rate.sleep()


if __name__ == "__main__":

	rospy.init_node("Error_node", anonymous=True)
	rospy.loginfo("error calculating node initialized")

	try:
		rospy.loginfo("error calculation start!")

		Error = error()
		while not rospy.is_shutdown():
			Error.error_cal()
	except rospy.ROSInterruptException:
		print "ROS terminated"
		pass
