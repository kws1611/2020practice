#!/usr/bin/python
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import smbus
import numpy as np
import time
import math
import numpy.linalg as lin
import tf

def quat_mult(a_1, a_2, a_3, a_4, b_1, b_2, b_3, b_4):
        #quaternion multiplication
        q_0 = a_1*b_1 - a_2*b_2 - a_3*b_3 - a_4*b_4
        q_1 = a_1*b_2 + a_2*b_1 + a_3*b_4 - a_4*b_3
        q_2 = a_1*b_3 - a_2*b_4 + a_3*b_1 + a_4*b_2
        q_3 = a_1*b_4 + a_2*b_3 - a_3*b_2 + a_4*b_1
        q = np.matrix([q_0, q_1, q_2, q_3])
        q = q.T
        return q

def norm_quat(a_1, a_2, a_3, a_4):
        #making quaternion size 1
        q_0 = a_1/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_1 = a_2/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_2 = a_3/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q_3 = a_4/math.sqrt(a_1**2 + a_2**2 + a_3**2 + a_4**2)
        q = np.matrix([q_0, q_1, q_2, q_3])
        q = q.T
        return q
def normalization(v1, v2, v3):
        #making the vector size 1
        norm = math.sqrt(v1 ** 2 + v2 ** 2 + v3 ** 2)
        v1 = v1 / norm
        v2 = v2 / norm
        v3=  v3 / norm
        return v1, v2, v3

def rotateVectorQuaternion(x, y, z, q0, q1, q2, q3):
        #rotate vector using quaternion
        vx = ((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * (x) + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z)
        vy = (2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z)
        vz = (2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z)
        return vx, vy, vz

def quat_2_matrix(w,x,y,z):
        return np.matrix([[w**2+ x**2 - y**2 - z**2, 2*x*y - 2*w*z, 2*w*y + 2*x*z],
                            [2*x*y + 2*w*z, w**2 -x**2 + y**2 - z**2, 2*y*z - 2*w*x],
                            [2*x*z - 2*w*y, 2*y*z + 2*w*x, w**2 - x**2 - y**2 + z**2]])

class kalman_Filter:

        def motion_cb(self,msg):
                self.mot_msg = msg
                self.motion_time = self.mot_msg.header.stamp.secs + self.mot_msg.header.stamp.nsecs*10**(-9)
                self.motion_x = self.mot_msg.pose.orientation.x
                self.motion_y = self.mot_msg.pose.orientation.y
                self.motion_z = self.mot_msg.pose.orientation.z
                self.motion_w = self.mot_msg.pose.orientation.w
                self.motion_pos_x = self.mot_msg.pose.position.x
                self.motion_pos_y = self.mot_msg.pose.position.y
                self.motion_pos_z = self.mot_msg.pose.position.z
                

        def imu_raw_data(self, msg):
                self.imu_data = msg
                self.imu_secs = self.imu_data.header.stamp.secs
                self.imu_nsecs = self.imu_data.header.stamp.nsecs
                self.acc_x = float(self.imu_data.linear_acceleration.x)*9.81
                self.acc_y = float(self.imu_data.linear_acceleration.y)*9.81
                self.acc_z = float(self.imu_data.linear_acceleration.z)*9.81

                self.gyro_x = float(self.imu_data.angular_velocity.x)
                self.gyro_y = float(self.imu_data.angular_velocity.y)
                self.gyro_z = float(self.imu_data.angular_velocity.z)

        def mag_raw_data(self, msg):
                self.mag_data = msg

                # bias when doing calibration at the motion capture lab
                self.mag_bias_x = -651.379996566
                self.mag_bias_y = 256.111658654
                self.mag_bias_z = -267.566586538

                self.mag_delta_x = 411.05001717
                self.mag_delta_y = 419.652764423
                self.mag_delta_z = 437.548798077
                """

                self.mag_bias_x = -117.11354739
                self.mag_bias_y = 506.397836538
                self.mag_bias_z = 653.636358173

                self.mag_delta_x = 1599.84215316
                self.mag_delta_y = 2077.56346154
                self.mag_delta_z = 1765.5636154
                """
                self.mag_average = (self.mag_delta_x + self.mag_delta_y + self.mag_delta_z)/3

                #magnetometer sensor's axis is twisted so we have to change axis

                self.mag_x = (self.mag_data.magnetic_field.y -self.mag_bias_y) * (self.mag_average)/(self.mag_delta_y)
                self.mag_y = (self.mag_data.magnetic_field.x -self.mag_bias_x) * (self.mag_average)/(self.mag_delta_x)
                self.mag_z = -(self.mag_data.magnetic_field.z -self.mag_bias_z) * (self.mag_average)/(self.mag_delta_z)
                """
                self.mag_x = (self.mag_data.magnetic_field.y -self.mag_bias_y)
                self.mag_y = (self.mag_data.magnetic_field.x -self.mag_bias_x)
                self.mag_z = -(self.mag_data.magnetic_field.z -self.mag_bias_z)
                """
        def __init__(self):


                # the gyro sensor standard variation
                self.Q = 10.0**(-10)*np.matrix([[1.0, 0, 0, 0],[0, 1.50628058**2, 0, 0],[0, 0, 1.4789602**2, 0],[0, 0, 0, 1.37315181**2]])

                # accelometer and magnetometer's standard variation
                self.R = 5*np.matrix([[0.00840483082215**2, 0, 0, 0],[0, 0.00100112198402**2, 0, 0], [0, 0, 0.00102210818946**2, 0], [0, 0, 0, 0.0114244938775**2]])

                # Subscriber created
                self.motion_x = 0.0 , self.motion_y = 0.0, self.motion_z = 0.0, self.motion_w = 0.0
                self.motion_time = 0.0
                self.motion_time_prev = 0.0
                self.motion_pos_x = 0.0, self.motion_pos_y = 0.0, self.motion_pos_z = 0.0
                self.motion_pos_prev_x = 0.0 ,self.motion_pos_prev_y = 0.0 ,self.motion_pos_prev_z = 0.0
                self.motion_vel_x = 0.0, self.motion_vel_y = 0.0, self.motion_vel_z = 0.0 

                # state variable
                self.position_x = 0.0, self.position_y = 0.0, self.position_z = 0.0
                self.position_prev_x = 0.0, self.position_prev_y = 0.0, self.position_prev_z = 0.0
                self.velocity_x = 0.0,self.velocity_y = 0.0, self.velocity_z = 0.0
                self.quat_x = 0.0, self.quat_y = 0.0, self.quat_z = 0.0, self.quat_w = 0.0

                # error variable
                self.position_er_x = 0.0, self.position_er_y = 0.0, self.position_er_z = 0.0
                self.position_prev_er_x = 0.0, self.position_prev_er_y = 0.0, self.position_prev_er_z = 0.0
                self.velocity_er_x = 0.0,self.velocity_er_y = 0.0, self.velocity_er_z = 0.0
                self.angle_er_x = 0.0, self.angle_er_y = 0.0, self.angle_er_z = 0.0

                #sensor data variable
                self.mag_x = 0.01, self.mag_y = 0.01, self.mag_z = 0.01 
                self.acc_x = 0.01, self.acc_y = 0.01, self.acc_z = 0.01
                self.acc_bias_x = 0.0, self.acc_bias_y = 0.0, self.acc_bias_z = 0.0
                self.gyro_x = 0.01, self.gyro_y = 0.01, self.gyro_z = 0.01
                self.gyro_bias_x = 0, self.gyro_bias_y = 0, self.gyro_bias_z = 0
                
                self.rate = rospy.Rate(78.5)

                #for bias calculation
                self.motion_cal_x = 0,  self.motion_cal_y = 0, self.motion_cal_z = 0 , self.motion_cal_w = 0
                self.mag_cal_x = 0, self.mag_cal_y = 0, self.mag_cal_z = 0
                self.gyro_cal_x = 0, self.gyro_cal_y = 0, self.gyro_cal_z = 0
                self.acc_cal_x = 0, self.acc_cal_y = 0, self.acc_cal_z = 0

                rospy.Subscriber("/vrpn_client_node/quad_imu_2/pose",PoseStamped,self.motion_cb)
                rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
                rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)

                self.Kalman_cov_pub = rospy.Publisher("/pose_covariance",PoseWithCovarianceStamped, queue_size=1)

                self.pos = np.matrix([[self.position_x],[self.position_y],[self.position_z]])
                self.vel = np.matrix([[self.velocity_x],[self.velocity_y],[self.velocity_z]])
                self.quat = np.matrix([[self.quat_w],[self.quat_x],[self.quat_y],[self.quat_z]])
                self.acc_bias = np.matrix([[self.acc_bias_x],[self.acc_bias_y],[self.acc_bias_z]])
                self.gyro_bias = np.matrix([[self.gyro_bias_x],[self.gyro_bias_y],[self.gyro_bias_z]])
                self.gravity = np.matrix([[0],[0],[1]])

                self.pos_er = np.matrix([[self.position_er_x],[self.position_er_y],[self.position_er_z]])
                self.vel_er = np.matrix([[self.velocity_er_x],[self.velocity_er_y],[self.velocity_er_z]])
                self.quat_er = np.matrix([[self.angle_er_x],[self.angle_er_y],[self.angle_er_z]])
                self.acc_bias_er = np.matrix([[self.acc_bias_er_x],[self.acc_bias_er_y],[self.acc_bias_er_z]])
                self.gyro_bias_er = np.matrix([[self.gyro_bias_er_x],[self.gyro_bias_er_y],[self.gyro_bias_er_z]])
                self.gravity_er = np.matrix([[0],[0],[0]])

                self.global_mag = np.matrix([[0], [0], [0]])
                self.correction_quat = np.matrix([[0], [0], [0], [0]]) # correct the difference between motion capture and IMU
                self.X = np.concatenate((self.pos, self.vel, self.quat, self.acc_bias, self.gyro_bias, self.gravity), axis = 0)
                self.X_error = np.concatenate((self.pos_er, self.vel_er, self.quat_er, self.acc_bias_er, self.gyro_bias_er, self.gravity_er), axis = 0)
                self.Cov = np.identity(13)
                self.Cov_error = np.identity(12)
                self.dt = float(1.0/78.5)
                self.H =  np.identity()


        def AF_matrix(self, q, a, w, dt):  
                # q = 4 * 1 matrix
                # a = 3 * 1 matrix bais calculated accel
                # w = 3 * 1 matrix bais calculated rotational velocity

                qw = q[0,0], qx = q[1,0], qy = q[2,0], qz = q[3,0]
                ax = a[0,0], ay = a[1,0], az = a[2,0]
                wx = w[0,0], wy = w[1,0], wz = w[3,0]

                rot_accel = np.matrix([[qw*ax + 2*qz*ay, qx*ax + 2*qy*ay, -qy*ax - 2*qw*az, -qz*ax + 2*qx*az],
                                            [qw*ay + 2*qx*az, -qx*ay + 2*qy*ax, qy*ay + 2*qz*az, -qz*ay - 2*qw*ax ],
                                            [qw*az - 2*qx*ay, -qx*az + 2*qz*ax, -qy*az + 2*qw*ax, qz*az + 2*qy*ay]])

                rot_quat = np.identity(4) + 0.5*dt*np.matrix([[0, wx, wy, wz],
                                                            [-wx, 0, wz, -wy],
                                                            [-wy, -wz, 0, wx],
                                                            [-wz, wy, -wx, 0]])

                A_1 = np.concatenate((np.identity(3), np.identity(3)*dt/2, rot_accel*dt**2/2, np.zeros((3,3)), np.zeros((3,3)), np.identity(3)*dt**2/2),axis = 1)
                A_2 = np.concatenate((np.zeros((3,3)), np.identity(3), rot_accel*dt, np.zeros((3,3)), np.zeros((3,3)), np.identity(3)*dt), axis = 1)
                A_3 = np.concatenate((np.zeros((4,4)), np.zeros((4,4)), rot_quat, np.zeros((4,4)), np.zeros((4,4))), axis = 1)
                A_4 = np.concatenate((np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.identity(3), np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                A_5 = np.concatenate((np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3)), np.identity(3), np.zeros((3,3))), axis = 1)
                A_6 = np.concatenate((np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3)), np.zeros((3,3)), np.identity(3)), axis = 1)

                err_rot = quat_2_matrix(qw, qx, qy, qz)

                err_acc = np.matrix([[0, -az, ay],
                                    [az, 0, -ax],
                                    [-ay, ax, 0]])

                err_rot_vel = np.matrix([[0, -wz, wy],
                                        [wz, 0, -wx],
                                        [-wy, wx, 0]])

                F_1 = np.concatenate((np.identity(3), np.identity(3)*dt/2, np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.identity(3)*dt**2/2),axis = 1)
                F_2 = np.concatenate((np.zeros((3,3)), np.identity(3), -err_rot*err_acc*dt, np.zeros((3,3)), np.zeros((3,3)), np.identity(3)*dt), axis = 1)
                F_3 = np.concatenate((np.zeros((3,3)), np.identity(3), err_rot.T*err_rot_vel*dt, np.zeros((3,3)), -np.identity(3)*dt, np.zeros((3,3))), axis = 1)
                F_4 = np.concatenate((np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.identity(3), np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                F_5 = np.concatenate((np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.identity(3), np.zeros((3,3))), axis = 1)
                F_6 = np.concatenate((np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.identity(3)), axis = 1)

                A = np.concatenate((A_1, A_2, A_3, A_4, A_5, A_6), axis = 0)
                F = np.concatenate((F_1, F_2, F_3, F_4, F_5, F_6), axis = 0)
                return A, F

        def H_matrix(self, q, mag_global = self.global_mag):
                mx= mag_global[0,0], my = mag_global[1,0], mz = mag_global[2,0]  # magnetometer global 
                qw = q[0,0], qx = q[1,0], qy = q[2,0], qz = q[3,0]

                error_quat = 0.5*np.matrix([[-qx, -qy, qz], 
                                            [qw, -qz, qy], 
                                            [qz, qw, -qx], 
                                            [-qy, qx, qw]])

                rot_mag = np.matrix([[qw*mx + 2*qz*my, qx*mx + 2*qy*my, -qy*mx - 2*qw*mz, -qz*mx + 2*qx*mz],
                                            [qw*my + 2*qx*mz, -qx*my + 2*qy*mx, qy*my + 2*qz*mz, -qz*my - 2*qw*mx ],
                                            [qw*mz - 2*qx*my, -qx*mz + 2*qz*mx, -qy*mz + 2*qw*mx, qz*mz + 2*qy*my]])

                H_1 = np.concatenate((np.identity(3), np.zeros((3,3)), np.zeros((3,4)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                H_1 = np.concatenate((np.zeros((3,3)), np.identity(3), np.zeros((3,4)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                H_1 = np.concatenate((np.zeros((3,3)) , np.zeros((4,4)), rot_mag , np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                self.H = np.concatenate((H_1, H_2, H_3), axis = 0)

                H_error_1 = np.concatenate((np.identity(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                H_error_2 = np.concatenate((np.zeros((3,3)), np.identity(3), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                H_error_3 = np.concatenate((np.zeros((3,3)), np.zeros((3,3)), error_quat, np.zeros((3,3)), np.zeros((3,3)), np.zeros((3,3))), axis = 1)
                H_error_4 = np.concatenate((np.zeros((9,9)), np.identity(9)), axis = 1)

                self.H_error_term = np.concatenate((H_error_1, H_error_2, H_error_3, H_error_4), axis = 0)
                self.H_error =self.H*self.H_error_term

                return self.H, self.H_error

        def error_reset(self):
                self.pose = self.pose + self.pose_er
                self.vel = self.vel + self.vel_er
                self.acc_bias = self.acc_bias + self.acc_bias_er
                self.gyro_bias = self.gyro_bias + self.gyro_bias_er
                self.gravity = self.gravity + self.gravity_er 

                self.quat = quat_mult(self.quat[0,0], self.quat[0,0], self.quat[0,0], self.quat[0,0], 0, self.angle_er_x, self.angle_er_y, self.angle_er_z)

                self.X = np.concatenate((self.pose, self.vel, self.quat, self.acc_bias, self.gyro_bias, self.gravity), axis = 0)
                self.X_error = np.matrix([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],])

        def first_cal(self):
                self.cal_count = 0
                # waits until the value is available
                while self.mag_x == 0.01:
                        time.sleep(0.1)
                # for 1.5 sec the average value is saved
                # the average value is calculated for calculating the yaw value
                if self.mag_x != 0.01:
                        self.calibration_time = time.time() + 1.5
                        while time.time() <= self.calibration_time:
                                self.mag_cal_x += self.mag_x
                                self.mag_cal_y += self.mag_y
                                self.mag_cal_z += self.mag_z
                                self.gyro_cal_x += self.gyro_x
                                self.gyro_cal_y += self.gyro_y
                                self.gyro_cal_z += self.gyro_z
                                self.acc_cal_x += self.acc_x
                                self.acc_cal_y += self.acc_y
                                self.acc_cal_z += self.acc_z
                                self.motion_cal_w += self.motion_w
                                self.motion_cal_x += self.motion_x
                                self.motion_cal_y += self.motion_y
                                self.motion_cal_z += self.motion_z
                                self.cal_count += 1

                        self.mag_cal_x /= self.cal_count
                        self.mag_cal_y /= self.cal_count
                        self.mag_cal_z /= self.cal_count
                        self.gyro_cal_x /= self.cal_count
                        self.gyro_cal_y /= self.cal_count
                        self.gyro_cal_z /= self.cal_count
                        self.acc_cal_x /= self.cal_count
                        self.acc_cal_y /= self.cal_count
                        self.acc_cal_z /= self.cal_count
                        self.motion_cal_w /= self.cal_count
                        self.motion_cal_x /= self.cal_count
                        self.motion_cal_y /= self.cal_count
                        self.motion_cal_z /= self.cal_count
                
                IMU_global = quat_mult(self.get_acc_quat(self.acc_cal_x, self.acc_cal_y, self.acc_cal_z), self.get_mag_quat(self.mag_cal_x, self.mag_cal_y, self.mag_cal_z))
                motion_global = norm_quat(self.motion_cal_w, self.motion_cal_x, self.motion_cal_y, self.motion_cal_z)
                self.correction_quat = quat_mult(motion_global[0,0], motion_global[1,0], motion_global[2,0], motion_global[3,0], IMU_global[0,0], -IMU_global[1,0], -IMU_global[2,0], -IMU_global[3,0])
                self.global_mag = quat_2_matrix(self.correction_quat[0,0], self.correction_quat[1,0], self.correction_quat[2,0], self.correction_quat[3,0]) * np.matrix([[self.mag_cal_x], [self.mag_cal_y], [self.mag_cal_z]])
        def get_acc_quat(self, accel_x, accel_y, accel_z):

                #normalize the accel value
                self.ax = accel_x / math.sqrt(accel_x**2 +accel_y**2 + accel_z**2)
                self.ay = accel_y / math.sqrt(accel_x**2 +accel_y**2 + acelc_z**2)
                self.az = accel_z / math.sqrt(accel_x**2 +accel_y**2 + accel_z**2)

                if self.az >= 0:
                        self.q_acc = np.matrix([math.sqrt(0.5*(self.az + 1)), -self.ay/(2*math.sqrt(0.5*(self.az+1))), self.ax/(2*math.sqrt(0.5*(self.az+1))), 0])
                else :
                        self.q_acc_const = math.sqrt((1.0-self.az) * 0.5)
                        self.q_acc = np.matrix([-self.ay/(2.0*self.q_acc_const), self.q_acc_const, 0.0, self.ax/(2.0*self.q_acc_const)])
                
                return self.q_acc

        def get_mag_quat(self, mag_x, mag_y, mag_z):
                #rotating the magnetometer's value using accelometer's quaternion
                lx, ly, lz = rotateVectorQuaternion(mag_x, mag_y, mag_z, self.q_acc[0,0], -self.q_acc[0,1], -self.q_acc[0,2], -self.q_acc[0,3])
                #calculating the yaw using rotated magnetometer value
                self.gamma = lx**2 + ly**2
                if lx >= 0:
                        self.q0_mag = math.sqrt(self.gamma + lx * math.sqrt(self.gamma))/ math.sqrt(2 * self.gamma)
                        self.q1_mag = 0
                        self.q2_mag = 0
                        self.q3_mag = ly / math.sqrt(2 * (self.gamma + lx * math.sqrt(self.gamma)))
                        self.q_mag= norm_quat(self.q0_mag, self.q1_mag, self.q2_mag, self.q3_mag)
                if lx < 0:
                        self.q0_mag = ly / math.sqrt(2 * (self.gamma - lx * math.sqrt(self.gamma)))
                        self.q1_mag = 0
                        self.q2_mag = 0
                        self.q3_mag = math.sqrt(self.gamma - lx * math.sqrt(self.gamma))/ math.sqrt(2 * self.gamma)
                        self.q_mag= norm_quat(self.q0_mag, self.q1_mag, self.q2_mag, self.q3_mag)

                return self.q_mag
        
        def measurement(self):
                self.motion_vel_x = (self.motion_pos_x - self.motion_pos_prev_x)/self.dt
                self.motion_vel_y = (self.motion_pos_y - self.motion_pos_prev_y)/self.dt
                self.motion_vel_z = (self.motion_pos_z - self.motion_pos_prev_z)/self.dt
                y = np.matrix([[self.motion_pos_x], [self.motion_pos_y], [self.motion_pos_z],
                                [self.motion_vel_x], [self.motion_vel_y], [self.motion_vel_z],
                                [self.mag_x], [self.mag_y], [self.mag_z]])
                self.motion_pos_prev_x = self. motion_pos_x
                self.motion_pos_prev_y = self. motion_pos_y
                self.motion_pos_prev_z = self. motion_pos_z
                return y
        
        def Ex_Kalman(self):
                A, F = self.AF_matrix(self.X[6,0], self.X[7,0], self.X[8,0], self.X[9,0]) # putting the quaternion
                H, H_er = self.H_matrix(self.X[6,0], self.X[7,0], self.X[8,0], self.X[9,0])
                y = self.measurement()

                # predict part for state and error state
                X_pre = A*self.X 
                X_er_pre = F*self.X_error
                cov_pre = A*self.Cov*A.T + self.R
                cov_er_pre = F*self.Cov_error*F.T + self.R_error ######### R matrix has to be set

                #Kalman gain part
                Kal = cov_pre*H.T*np.linalg.inv(H*cov_pre*H.T + self.Q)
                kal_er = cov_er_pre*H_er.T*np.linalg.inv(H_er*cov_er_pre0*H_er.T + self.Q_er)  ########### Q matrix has to be set

                #measurement correction
                X = x_pre + kal*(y - H*x_pre)
                x_er = kal_er*(y - H*x_er_pre) 
                self.Cov = (np.identity(19) - kal*H)*cov_pre
                self.Cov_error = (np.identity(18) - kal_er*H_er)*cov_er_pre

                self.X = X
                self.X_error = x_er ####################### normalize the quaternion has to be made
                self.simulation_pub()

        def simulation_pub(self):
                pose_topic = PoseWithCovarianceStamped()
                pose_topic.header.stamp.secs = self.imu_secs
                pose_topic.header.stamp.nsecs = self.imu_nsecs
                pose_topic.header.frame_id = "world"
                pose_topic.pose.pose.position.x = self.X[0,0]
                pose_topic.pose.pose.position.y = self.X[1,0]
                pose_topic.pose.pose.position.z = self.X[2,0]
                pose_topic.pose.pose.orientation.x = self.X[6,0]
                pose_topic.pose.pose.orientation.y = self.X[7,0]
                pose_topic.pose.pose.orientation.z = self.X[8,0]
                pose_topic.pose.pose.orientation.w = self.X[9,0]
                pose_topic.pose.covariance = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0289, 0.0289, 0.1207,0,0,0,0.0289, 0.0289, 0.1207,0,0,0,0.1207,0.1207,0.5041]
                self.Kalman_cov_pub.publish(pose_topic)

if __name__ == "__main__":

        rospy.init_node("Kalman_Filter", anonymous=True)
        rospy.loginfo("Kalman filter node initialized")

        try:
                rospy.loginfo("EXtended Kalman filter start!")
                # initialize the class
                Filtering = kalman_Filter()
                # starting the calibration
                Filtering.first_cal()
                
                while not rospy.is_shutdown():
                        #kalman filter starting
                        Filtering.Ex_kalman()
        except rospy.ROSInterruptException:
                print ("ROS terminated")
                pass
