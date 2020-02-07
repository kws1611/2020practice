#!/usr/bin/python

import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from att_est.msg import error_msg
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

def rotateVectorQuaternion(x, y, z, q0, q1, q2, q3):
        q_0 = a_1/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_1 = a_2/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_2 = a_3/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_3 = a_4/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        vx = ((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * (x) + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z)
        vy = (2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z)
        vz = (2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z)
        return vx, vy, vz

def quat_to_matrix(q0,q1,q2,q3):
        rotation_mat = np.matrix([[1-2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],[2*q1*q2 + 2*q0*q3, 1-2*q1**2 -2*q3**2, 2*q2*q3 - q2*q0*q1],[2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q3*q1, 1-2*q1**2 - 2*q2**2]])

        return rotation_mat

class error:
        def motion_cb(self,msg):
                self.mot_msg = msg
                self.motion_time = self.mot_msg.header.stamp.secs + self.mot_msg.header.stamp.nsecs*10**(-9)
                self.motion_x = self.mot_msg.pose.orientation.x
                self.motion_y = self.mot_msg.pose.orientation.y
                self.motion_z = self.mot_msg.pose.orientation.z
                self.motion_w = self.mot_msg.pose.orientation.w

        def kalman_cb(self,msg):
                self.kalman_msg = msg
                self.kal_time = self.kalman_msg.header.stamp.secs + self.kalman_msg.header.stamp.nsecs*10**(-9)
                self.kal_x=self.kalman_msg.pose.pose.orientation.x
                self.kal_y=self.kalman_msg.pose.pose.orientation.y
                self.kal_z=self.kalman_msg.pose.pose.orientation.z
                self.kal_w=self.kalman_msg.pose.pose.orientation.w

        def comp_cb(self,msg):
                self.comp_msg = msg
                self.comp_time = self.comp_msg.header.stamp.secs + self.comp_msg.header.stamp.nsecs*10**(-9)
                self.comp_x=self.comp_msg.pose.pose.orientation.x
                self.comp_y=self.comp_msg.pose.pose.orientation.y
                self.comp_z=self.comp_msg.pose.pose.orientation.z
                self.comp_w=self.comp_msg.pose.pose.orientation.w

        def initialize(self):
                q0_sum, q1_sum, q2_sum, q3_sum = 0.0, 0.0, 0.0, 0.0
                t_prev = time.time()
                t_now = time.time()
                num = 0
                while t_now - t_prev < 2:
                        q0_sum += self.motion_w
                        q1_sum += self.motion_x
                        q2_sum += self.motion_y
                        q3_sum += self.motion_z
                        num += 1
                        t_now = time.time()
                self.q0_init = q0_sum / num
                self.q1_init = q1_sum / num
                self.q2_init = q2_sum / num
                self.q3_init = q3_sum / num

	def __init__(self):
		# Subscriber created
                self.rate = rospy.Rate(78.5)
		rospy.Subscriber("/pose_covariance", PoseWithCovarianceStamped, self.kalman_cb)
                #rospy.Subscriber("/quat", PoseWithCovarianceStamped, self.comp_cb)
                rospy.Subscriber("/vrpn_client_node/quad_imu_2/pose",PoseStamped,self.motion_cb)
                self.error_comp_pub = rospy.Publisher("/comp_error",error_msg, queue_size=1)
                self.error_kal_pub = rospy.Publisher("/kal_error",error_msg, queue_size=1)

                self.kal_x= 0.0
                self.kal_y= 0.0
                self.kal_z= 0.0
                self.kal_w= 0.0
                self.kal_time = 0.0
                self.comp_x= 0.0
                self.comp_y= 0.0
                self.comp_z= 0.0
                self.comp_w= 0.0
                self.comp_time = 0.0
                self.motion_x = 0.0
                self.motion_y = 0.0
                self.motion_z = 0.0
                self.motion_w = 0.0
                self.motion_time = 0.0
                self.q0_init = 0.0
                self.q1_init = 0.0
                self.q2_init = 0.0
                self.q3_init = 0.0
                while self.kal_w == 0.0 :
                        time.sleep(0.1)
                self.initialize()

        def motion_calibration(self):
                motion = quat_mult(self.motion_w,self.motion_x,self.motion_y,self.motion_z,self.q0_init,-self.q1_init,-self.q2_init,-self.q3_init)

                self.motion_w = motion[0,0]
                self.motion_x = motion[1,0]
                self.motion_y = motion[2,0]
                self.motion_z = motion[3,0]

        def kal_coordinate_cal(self, x , y, z):
                kal_turned_coor = quat_to_matrix(self.kal_w, self.kal_x, self.kal_y, self.kal_z)*np.matrix([[x],[y],[z]])

                return kal_turned_coor

        def motion_coordinate_cal(self, x , y, z):
                motion_turned_coor = quat_to_matrix(self.motion_w, self.motion_x, self.motion_y, self.kal_z)*np.matrix([[x],[y],[z]])

                return motion_turned_coor

        def comp_coordinate_cal(self, x , y, z):
                comp_turned_coor = quat_to_matrix(self.kal_w, self.kal_x, self.kal_y, self.kal_z)*np.matrix([[x],[y],[z]])

                return comp_turned_coor
        def kal_coordinate_error_calculation(self):
                self.kal_x_diff = self.motion_coordinate_cal(1.0,0.0,0.0) - self.kal_coordinate_cal(1.0,0.0,0.0)
                self.kal_y_diff = self.motion_coordinate_cal(0.0,1.0,0.0) - self.kal_coordinate_cal(0.0,1.0,0.0)
                self.kal_z_diff = self.motion_coordinate_cal(0.0,0.0,1.0) - self.kal_coordinate_cal(0.0,0.0,1.0)
                self.kal_x_dist = math.sqrt(self.kal_x_diff[0,0]**2 + self.kal_x_diff[1,0]**2 + self.kal_x_diff[2,0]**2)
                self.kal_y_dist = math.sqrt(self.kal_y_diff[0,0]**2 + self.kal_y_diff[1,0]**2 + self.kal_y_diff[2,0]**2)
                self.kal_z_dist = math.sqrt(self.kal_z_diff[0,0]**2 + self.kal_z_diff[1,0]**2 + self.kal_z_diff[2,0]**2)

                self.kal_diff_size = math.sqrt(self.kal_x_dist**2 + self.kal_y_dist**2 + self.kal_z_dist**2)

        def comp_coordinate_error_calculation(self):
                self.comp_x_diff = self.motion_coordinate_cal(1,0,0) - self.comp_coordinate_cal(1,0,0)
                self.comp_y_diff = self.motion_coordinate_cal(0,1,0) - self.comp_coordinate_cal(0,1,0)
                self.comp_z_diff = self.motion_coordinate_cal(0,0,1) - self.comp_coordinate_cal(0,0,1)
                self.comp_x_dist = math.sqrt(self.comp_x_diff[0,0]**2 + self.comp_x_diff[1,0]**2 + self.comp_x_diff[2,0]**2)
                self.comp_y_dist = math.sqrt(self.comp_y_diff[0,0]**2 + self.comp_y_diff[1,0]**2 + self.comp_y_diff[2,0]**2)
                self.comp_z_dist = math.sqrt(self.comp_z_diff[0,0]**2 + self.comp_z_diff[1,0]**2 + self.comp_z_diff[2,0]**2)

                self.comp_diff_size = math.sqrt(self.comp_x_diff**2 + self.comp_y_diff**2 + self.comp_z_diff**2)

        def error_rpy(self,q0,q1,q2,q3):
                roll = math.atan2(2*(q0*q1 + q2*q3),(1-2*(q1**2 + q2**2)))
                if 2*(q0*q2 - q3*q1) > 1 or 2*(q0*q2 - q3*q1) < -1 :
                        pitch = 10

                else :
                        pitch = math.asin(2*(q0*q2 - q3*q1))
                yaw = math.atan2(2*(q0*q3 + q1*q2),1-2*(q2**2 + q3**2))

                return roll, pitch, yaw

        def error_comp_coordinate_cal(self):
                self.comp_err_x,self.comp_err_y,self.comp_err_z = rotateVectorQuaternion(1, 1, 1, self.comp_w, self.comp_x, self.comp_y, self.comp_z)
                self.mot_err_x,self.mot_err_y,self.mot_err_z = rotateVectorQuaternion(1, 1, 1, self.motion_w, self.motion_x, self.motion_y, self.motion_z)

                self.comp_err_dist = math.sqrt((self.comp_err_x-self.mot_err_x)**2 + (self.comp_err_y-self.mot_err_y)**2 + (self.comp_err_z - self.mot_err_z)**2)

        def error_rpy_cal(self):
                self.kal_roll, self.kal_pitch , self.kal_yaw = self.error_rpy(self.kal_w, self.kal_x, self.kal_y, self.kal_z)
                self.mot_roll, self.mot_pitch , self.mot_yaw = self.error_rpy(self.motion_w, self.motion_x, self.motion_y, self.motion_z)
                self.comp_roll, self.comp_pitch, self.comp_yaw = self.error_rpy(self.comp_w, self.comp_x, self.comp_y, self.comp_z)
                
                self.error_kal_roll = self.kal_roll - self.mot_roll
                self.error_kal_pitch = self.kal_pitch - self.mot_pitch
                self.error_kal_yaw = self.kal_yaw - self.mot_yaw
                """
                self.error_comp_roll = self.comp_roll - self.mot_roll
                self.error_comp_pitch = self.comp_pitch - self.mot_pitch
                self.error_comp_yaw = self.comp_yaw - self.mot_yaw
                """
	def error_cal(self):
                self.motion_calibration()
                err_kal_topic=error_msg()
                #error_comp_topic = error_msg()
                self.error_rpy_cal()
                self.kal_coordinate_error_calculation()
                #self.comp_coordinate_error_calculation()
		# error calculating
                self.kal_error = quat_mult(self.kal_w,self.kal_x,self.kal_y,self.kal_z,self.motion_w,-self.motion_x,-self.motion_y,-self.motion_z)
                #self.comp_error = quat_mult(self.comp_w,self.comp_x,self.comp_y,self.comp_z,self.motion_w,-self.motion_x,-self.motion_y,-self.motion_z)
                err_kal_topic.x = self.kal_x_dist
                err_kal_topic.y =self.kal_y_dist
                err_kal_topic.z =self.kal_z_dist
                err_kal_topic.size =self.kal_diff_size

                err_kal_topic.time = self.kal_time
                err_kal_topic.mot_time = self.motion_time

                err_kal_topic.roll = self.kal_roll
                err_kal_topic.pitch = self.kal_pitch
                err_kal_topic.yaw = self.kal_yaw

                err_kal_topic.mot_roll = self.mot_roll
                err_kal_topic.mot_pitch = self.mot_pitch
                err_kal_topic.mot_yaw = self.mot_yaw

                err_kal_topic.err_roll = self.error_kal_roll
                err_kal_topic.err_pitch = self.error_kal_pitch
                err_kal_topic.err_yaw = self.error_kal_yaw

                err_kal_topic.err_quat_x = self.kal_error[1,0]
                err_kal_topic.err_quat_y = self.kal_error[2,0]
                err_kal_topic.err_quat_z = self.kal_error[3,0]
                err_kal_topic.err_quat_w = self.kal_error[0,0]

                err_kal_topic.quat_x = self.kal_x
                err_kal_topic.quat_y = self.kal_y
                err_kal_topic.quat_z = self.kal_z
                err_kal_topic.quat_w = self.kal_w
                """
		error_comp_topc.x = self.comp_error[1,0]
		error_comp_topc.y = self.comp_error[2,0]
		error_comp_topc.z = self.comp_error[3,0]
		error_comp_topc.w = self.comp_error[0,0]
                """

                if self.kal_diff_size != 0.0:
                        print("size")
                        print("%20.10f" %self.kal_diff_size)

                #if self.kal_error[0,0] != 1.0:
                #        print("quat")
                #print("kalman error distance %.5f " %self.kal_err_dist)
                #print("complementary error distance %.5f " %self.comp_err_dist)
                #print("kalman error roll %.5f pitch %.5f yaw %.5f"  %(self.error_kal_roll,self.error_kal_pitch,self.error_kal_yaw))
                #print("complementary error roll %.5f pitch %.5f yaw %.5f"  %(self.error_comp_roll,self.error_comp_pitch,self.error_comp_yaw))
                self.error_kal_pub.publish(err_kal_topic)

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
