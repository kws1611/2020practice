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
        vx = -((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * (x) + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z)
        vy = -(2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z)
        vz = -(2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z)
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
                self.mag_x = float(self.mag_data.magnetic_field.x)
                self.mag_y = float(self.mag_data.magnetic_field.y)
                self.mag_z = float(self.mag_data.magnetic_field.z)
        def __init__(self):
                self.kalman_topic = Quaternion()
                self.mag_x = 0.0
                self.mag_y = 0.0
                self.mag_z = 0.0
                self.acc_x = 0.0
                self.acc_y = 0.0
                self.acc_z = 0.0
                self.gyro_x = 0.0
                self.gyro_y = 0.0
                self.gyro_z = 0.0
                self.rate = rospy.Rate(78.5)
                rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
                rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)
        def first_mag_cal(self):
                self.mag_cal_x = 0
                self.mag_cal_y = 0
                self.mag_cal_z = 0
                self.cal_count = 0
                self.mag_x_h = -500
                self.mag_x_l = 0
                self.mag_y_h = -500
                self.mag_y_l = 0
                self.mag_z_h = -500
                self.mag_z_l = 0
                while self.mag_x == 0.0:
                        time.sleep(0.1)
                if self.mag_x != 0.0:
                        self.calibration_time = time.time() + 60
                        while time.time() <= self.calibration_time:
                                self.mag_cal_x += self.mag_x
                                self.mag_cal_y += self.mag_y
                                self.mag_cal_z += self.mag_z
                                if self.mag_x_h < self.mag_x:
                                        self.mag_x_h = self.mag_x
                                if self.mag_x_l > self.mag_x:
                                        self.mag_x_l = self.mag_x
                                if self.mag_y_h < self.mag_y:
                                        self.mag_y_h = self.mag_y
                                if self.mag_y_l > self.mag_y:
                                        self.mag_y_l = self.mag_y
                                if self.mag_z_h < self.mag_z:
                                        self.mag_z_h = self.mag_z
                                if self.mag_z_l > self.mag_z:
                                        self.mag_z_l = self.mag_z
                                self.cal_count += 1

                        print("mag_x")
                        print((self.mag_x_h + self.mag_x_l)/2)
                        print("mag_y")
                        print((self.mag_y_h + self.mag_y_l)/2)
                        print("mag_z")
                        print((self.mag_z_h + self.mag_z_l)/2)

                        print("mag_delta_x")
                        print((self.mag_x_h - self.mag_x_l)/2)
                        print("mag_delta_y")
                        print((self.mag_y_h - self.mag_y_l)/2)
                        print("mag_delta_z")
                        print((self.mag_z_h - self.mag_z_l)/2)

                        print("%f   %f    %f      %f      %f      %f" %(self.mag_x_l,self.mag_x_h,self.mag_y_l,self.mag_y_h,self.mag_z_l, self.mag_z_h))
if __name__ == "__main__":
        rospy.init_node("mag_test_calibration", anonymous=True)
        rospy.loginfo("mag_test_calibration")
        try:
                rospy.loginfo("Kalman filter start!")
                Filtering = kalman_Filter()
                Filtering.first_mag_cal()
        except rospy.ROSInterruptException:
                print "ROS terminated"
                pass
