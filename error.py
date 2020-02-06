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
        vx = ((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * (x) + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z)
        vy = (2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z)
        vz = (2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z)
        return vx, vy, vz

def quat_to_matrix(q0,q1,q2,q3):
        rotation_mat = np.matrix([[1-2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],[2*q1*q2 + 2*q0*q3, 1-2*q1** -2*q3**2, 2*q2*q3 - q2*q0*q1],[2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q3*q1, 1-2*q1**2 - 2*q2**2]])

        return rotation_mat



class error:
<<<<<<< HEAD
        def motion_cb(self):
                A = quat_mult(self.q0_init, -self.q1_init,-self.q2_init, self.q3_init,self.motion_w, self.motion_x, self.motion_y, self.motion_z)
                self.motion_w, self.motion_x, self.motion_y, self.motion_z = A[0,0], A[0,1], A[0,2], A[0,3]
        
        def get_motion_cb(self,msg):
=======

        def motion_cb(self,msg):
>>>>>>> 884479a0ef4077bfeea0e9e83b74ca047f05d282
                self.mot_msg = msg
                self.motion_time = self.mot_msg.header.stamp.secs + self.mot_msg.header.stamp.nsecs*10**(-9)
                self.motion_x = self.mot_msg.pose.orientation.x
                self.motion_y = self.mot_msg.pose.orientation.y
                self.motion_z = self.mot_msg.pose.orientation.z
                self.motion_w = self.mot_msg.pose.orientation.w
                self.motion_cb()

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
                rospy.Subscriber("/vrpn_client_node/quad_imu_2/pose",PoseStamped,self.get_motion_cb)
                self.error_kalman_pub = rospy.Publisher("/kalman_error",PoseStamped, queue_size=1)
                self.error_pub = rospy.Publisher("/error",error_msg, queue_size=1)
                #self.error_comp_pub = rospy.Publisher("/comp_error",Quaternion, queue_size=1)
                self.kal_x= 0.0
                self.kal_y= 0.0
                self.kal_z= 0.0
                self.kal_w= 0.0
                self.motion_x = 0.0
                self.motion_y = 0.0
                self.motion_z = 0.0
                self.motion_w = 0.0
                self.motion_time = 0.0
                self.initialize()


        def kalman_coordinate_cal(self, x , y, z):
                z_rotated_coordinate = quat_to_matrix(self.kal_w, self.kal_x, self.kal_y, self.kal_z) * np.matrix([[np.cos(np.pi/2), -np.sin(np.pi/2), 0 ],[np.sin(np.pi/2),np.cos(np.pi/2),0],[0,0,1]])*np.matrix([[x],[y],[z]])

                return z_rotated_coordinate

        def error_kal_coordinate_cal(self):
                self.kal_err_x,self.kal_err_y,self.kal_err_z = rotateVectorQuaternion(1, 1, 1, self.kal_w, self.kal_x, self.kal_y, self.kal_z)
                self.mot_err_x,self.mot_err_y,self.mot_err_z = rotateVectorQuaternion(1, 1, 1, self.motion_w, self.motion_x, self.motion_y, self.motion_z)

                self.kal_err_dist = math.sqrt((self.kal_err_x-self.mot_err_x)**2 + (self.kal_err_y-self.mot_err_y)**2 + (self.kal_err_z - self.mot_err_z)**2)

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
                #self.comp_roll, self.comp_pitch, self.comp_yaw = self.error_rpy(self.comp_w, self.comp_x, self.comp_y, self.comp_z)

                self.error_kal_roll = self.kal_roll - self.mot_roll
                self.error_kal_pitch = self.kal_pitch - self.mot_pitch
                self.error_kal_yaw = self.kal_yaw - self.mot_yaw
                """
                self.error_comp_roll = self.comp_roll - self.mot_roll
                self.error_comp_pitch = self.comp_pitch - self.mot_pitch
                self.error_comp_yaw = self.comp_yaw - self.mot_yaw
                """
	def error_cal(self):
                error_topic=error_msg()
                error_kal_topic = PoseStamped()
                #error_camp_topic = Quaternion()
                #self.error_compt_coordinate_cal()
                print("%10.10f" %self.motion_time)
                self.error_kal_coordinate_cal()

                self.error_rpy_cal()
                """
                change_100=self.kalman_coordinate_cal(1,0,0) - quat_to_matrix(self.motion_w, self.motion_x, self.motion_y, self.motion_z)*np.matrix([[1],[0],[0]])
                print(change_100)
                change_010=self.kalman_coordinate_cal(0,1,0) - quat_to_matrix(self.motion_w, self.motion_x, self.motion_y, self.motion_z)*np.matrix([[0],[1],[0]])
                print(change_010)
                change_001=self.kalman_coordinate_cal(0,0,1) - quat_to_matrix(self.motion_w, self.motion_x, self.motion_y, self.motion_z)*np.matrix([[0],[0],[1]])
                print(change_001)
                """

		# error calculating
                self.kal_error = quat_mult(self.kal_w,self.kal_x,self.kal_y,self.kal_z,self.motion_w,self.motion_x,self.motion_y,self.motion_z)
                #self.comp_error = quat_mult(self.comp_w,self.comp_x,self.comp_y,self.comp_z,self.motion_w,self.motion_x,self.motion_y,self.motion_z)
                error_kal_topic.header.frame_id = "world"
                error_topic.x = 1.0
                error_topic.y = 2.0
                error_topic.z = 3.0
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
                print("kalman error distance %.5f " %self.kal_err_dist)
                #print("complementary error distance %.5f " %self.comp_err_dist)
                print("kalman error roll %.5f pitch %.5f yaw %.5f"  %(self.error_kal_roll,self.error_kal_pitch,self.error_kal_yaw))
                #print("complementary error roll %.5f pitch %.5f yaw %.5f"  %(self.error_comp_roll,self.error_comp_pitch,self.error_comp_yaw))
                self.error_kalman_pub.publish(error_kal_topic)
                self.error_pub.publish(error_topic)
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
