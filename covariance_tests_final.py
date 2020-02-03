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
        vx = ((q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * (x) + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z)
        vy = (2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z)
        vz = (2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z)
        return vx, vy, vz

class kalman_Filter:

        def imu_raw_data(self, msg):
                self.imu_data = msg
                """
                self.acc_x = -float(self.imu_data.linear_acceleration.y)
                self.acc_y = float(self.imu_data.linear_acceleration.x)
                self.acc_z = float(self.imu_data.linear_acceleration.z)

                self.gyro_x = -float(self.imu_data.angular_velocity.y)
                self.gyro_y = float(self.imu_data.angular_velocity.x)
                self.gyro_z = float(self.imu_data.angular_velocity.z)
                """
                self.acc_x = float(self.imu_data.linear_acceleration.x)*9.81
                self.acc_y = float(self.imu_data.linear_acceleration.y)*9.81
                self.acc_z = float(self.imu_data.linear_acceleration.z)*9.81

                self.gyro_x = float(self.imu_data.angular_velocity.x)
                self.gyro_y = float(self.imu_data.angular_velocity.y)
                self.gyro_z = float(self.imu_data.angular_velocity.z)

        def mag_raw_data(self, msg):
                self.mag_data = msg
                """
                self.mag_bias_x = -567.687070742
                self.mag_bias_y = 211.888161058
                self.mag_bias_z = -335.368569712
                """

                self.mag_bias_x = -651.379996566
                self.mag_bias_y = 256.111658654
                self.mag_bias_z = -267.566586538

                self.mag_delta_x = 411.05001717
                self.mag_delta_y = 419.652764423
                self.mag_delta_z = 437.548798077
                """

                self.mag_bias_x = -568.578554258
                self.mag_bias_y = 258.323016827
                self.mag_bias_z = -302.82361778

                self.mag_delta_x = 420.907331731
                self.mag_delta_y = 450.703305288
                self.mag_delta_z = 458.34140625
                """
                self.mag_average = (self.mag_delta_x + self.mag_delta_y + self.mag_delta_z)/3

                self.mag_x = (self.mag_data.magnetic_field.y -self.mag_bias_y) * (self.mag_average)/(self.mag_delta_y)
                self.mag_y = (self.mag_data.magnetic_field.x -self.mag_bias_x) * (self.mag_average)/(self.mag_delta_x)
                self.mag_z = (self.mag_data.magnetic_field.z -self.mag_bias_z) * (self.mag_average)/(self.mag_delta_z)
                """
                self.mag_x = (self.mag_data.magnetic_field.x -self.mag_bias_x) * (self.mag_average)/(self.mag_delta_x)
                self.mag_y = (self.mag_data.magnetic_field.y -self.mag_bias_y) * (self.mag_average)/(self.mag_delta_y)
                self.mag_z = (self.mag_data.magnetic_field.z -self.mag_bias_z) * (self.mag_average)/(self.mag_delta_z)
                """
        def __init__(self):
                self.kalman_topic = Quaternion()
                self.X = np.matrix('1;0;0;0')
                self.P = np.identity(4)
                self.dt = float(1.0/78.5)
                self.H = np.identity(4)
                self.Q = np.matrix([[0.0299009184657**2, 0, 0, 0],[0, 0.0315001603291**2, 0, 0],[0, 0, 0.0751542253079**2, 0],[0, 0, 0, 0.0964859324556**2]])
                #self.Q = np.matrix([[0, 0, 0, 0],[0, 0.0315001603291**2, 0, 0],[0, 0, 0.0751542253079**2, 0],[0, 0, 0, 0.0964859324556**2]])
                #self.Q = 10**(-4)*np.matrix('0.64 0 0 0; 0 0.64 0 0 ; 0 0 0.169 0; 0 0 0 0.64 ')
                #self.Q = np.matrix('0 -0.4 -0.65 -0.4; 0.4 0 0.4 -0.65; 0.65 -0.4 0 0.4; 0.4 0.65 -0.4 0')
                self.R =np.matrix([[0.0209789550647**2, 0, 0, 0],[0, 0.00541388117932**2, 0, 0], [0, 0, 0.0102877460488**2, 0], [0, 0, 0, 0.0161420424737**2]])
                #self.R =np.matrix([[0, 0, 0, 0],[0, 0.00541388117932**2, 0, 0], [0, 0, 0.0102877460488**2, 0], [0, 0, 0, 0.0161420424737**2]])

                #self.R =np.matrix([[0.0209789550647**2, 0, 0, 0],[0, 0.00541388117932**2, 0, 0], [0, 0, 0.0102877460488**2, 0], [0, 0, 0, 1]])
                # Subscriber created
                self.mag_x = 0.01
                self.mag_y = 0.01
                self.mag_z = 0.01
                self.acc_x = 0.01
                self.acc_y = 0.01
                self.acc_z = 0.01
                self.gyro_x = 0.01
                self.gyro_y = 0.01
                self.gyro_z = 0.01
                self.gyro_bias_x = 0
                self.gyro_bias_y = 0
                self.gyro_bias_z = 0
                self.rate = rospy.Rate(80.0)
                self.Z_saved_w =[]
                self.Z_saved_x =[]
                self.Z_saved_y =[]
                self.Z_saved_z =[]
                self.gyro_saved_x=[]
                self.gyro_saved_y=[]
                self.gyro_saved_z=[]
                """
                self.gyro_past_x = 0.0
                self.gyro_past_y = 0.0
                self.gyro_past_z = 0.0
                """

                rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
                rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)

                self.Kalman_cov_pub = rospy.Publisher("/pose_covariance",PoseWithCovarianceStamped, queue_size=1)
                self.Kalman_pub = rospy.Publisher("/Kalman_quat",Quaternion, queue_size=1)

        def first_mag_cal(self):
                self.mag_cal_x = 0
                self.mag_cal_y = 0
                self.mag_cal_z = 0
                self.gyro_cal_x = 0
                self.gyro_cal_y = 0
                self.gyro_cal_z = 0
                self.acc_cal_x = 0
                self.acc_cal_y = 0
                self.acc_cal_z = 0
                self.cal_count = 0
                while self.mag_x == 0.01:
                        time.sleep(0.1)

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

        def get_acc_quat(self):
                """
                self.acc_x -= (self.gyro_x - self.gyro_past_x) / self.dt
                self.acc_y -=(self.gyro_y - self.gyro_past_y) / self.dt
                self.acc_z -=(self.gyro_z - self.gyro_past_z) / self.dt
                """
                self.ax = self.acc_x / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
                self.ay = self.acc_y / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
                self.az = self.acc_z / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)

                if self.az >= 0:
                        self.q_acc = np.matrix([math.sqrt(0.5*(self.az + 1)), -self.ay/(2*math.sqrt(0.5*(self.az+1))), self.ax/(2*math.sqrt(0.5*(self.az+1))), 0])
                else :
                        self.q_acc_const = math.sqrt((1.0-self.az) * 0.5)

                        self.q_acc = np.matrix([-self.ay/(2.0*self.q_acc_const), self.q_acc_const, 0.0, self.ax/(2.0*self.q_acc_const)])

                #self.q_acc = norm_quat(self.q_acc[0,0], self.q_acc[0,1], self.q_acc[0,2], self.q_acc[0,3])
                #self.q_acc = self.q_acc.T
                """
                self.gyro_past_x = self.gyro_x
                self.gyro_past_z = self.gyro_y
                self.gyro_past_z = self.gyro_z
                """


        def get_mag_quat(self):
                #self.mag_x, self.mag_y, self.mag_z = normalization(self.mag_x, self.mag_y, self.mag_z)
                lx, ly, lz = rotateVectorQuaternion(self.mag_x, self.mag_y, self.mag_z, self.q_acc[0,0], self.q_acc[0,1], self.q_acc[0,2], self.q_acc[0,3])
                self.gamma = lx**2 + ly**2
                if lx >= 0:
                        self.q0_mag = math.sqrt(self.gamma + lx * math.sqrt(self.gamma))/ math.sqrt(2 * self.gamma)
                        self.q1_mag = 0
                        self.q2_mag = 0
                        self.q3_mag = ly / math.sqrt(2 * (self.gamma + lx * math.sqrt(self.gamma)))
                        print("     ")
                        self.q_mag= norm_quat(self.q0_mag, self.q1_mag, self.q2_mag, self.q3_mag)
                if lx < 0:
                        self.q0_mag = ly / math.sqrt(2 * (self.gamma - lx * math.sqrt(self.gamma)))
                        self.q1_mag = 0
                        self.q2_mag = 0
                        self.q3_mag = math.sqrt(self.gamma - lx * math.sqrt(self.gamma))/ math.sqrt(2 * self.gamma)
                        self.q_mag= norm_quat(self.q0_mag, self.q1_mag, self.q2_mag, self.q3_mag)
                        print("alpha")


        def covariance_test(self):

                self.get_acc_quat()
                self.get_mag_quat()

                self.Z = quat_mult(self.q_acc[0,0],self.q_acc[0,1],self.q_acc[0,2],self.q_acc[0,3],self.q_mag[0,0],self.q_mag[1,0],self.q_mag[2,0],self.q_mag[3,0])
                self.Z_saved_w.append(self.Z[0,0])
                self.Z_saved_x.append(self.Z[1,0])
                self.Z_saved_y.append(self.Z[2,0])
                self.Z_saved_z.append(self.Z[3,0])
                self.gyro_saved_x.append(self.gyro_x)
                self.gyro_saved_y.append(self.gyro_y)
                self.gyro_saved_z.append(self.gyro_z)
                self.rate.sleep()

        def prints(self):
                """
                std_w = np.std(self.Z_saved_w)
                std_x = np.std(self.Z_saved_x)
                std_y = np.std(self.Z_saved_y)
                std_z = np.std(self.Z_saved_z)
                print(std_w)
                print(std_x)
                print(std_y)
                print(std_z)
                """
                std_gyro_x = np.std(self.gyro_saved_x)
                std_gyro_y = np.std(self.gyro_saved_y)
                std_gyro_z = np.std(self.gyro_saved_z)
                print(std_gyro_x)
                print(std_gyro_y)
                print(std_gyro_z)
        def final(self):

                self.gyro_cov_x = 0.00236486050407
                self.gyro_cov_y = 0.00232170974699
                self.gyro_cov_z = 0.00215584834261
                self.A = np.identity(4) + self.dt*float(0.5)*np.matrix([[0 ,-self.gyro_cov_x,-self.gyro_cov_y,-self.gyro_cov_z], [self.gyro_cov_x, 0 ,self.gyro_cov_z, -self.gyro_cov_y], [self.gyro_cov_y, -self.gyro_cov_z ,0, self.gyro_cov_x], [self.gyro_cov_z, self.gyro_cov_y ,-self.gyro_cov_x, 0 ]])
                self.X = self.A * self.X
                print(self.X)
if __name__ == "__main__":

        rospy.init_node("Kalman_Filter", anonymous=True)
        rospy.loginfo("Kalman filter node initialized")

        try:
                rospy.loginfo("Kalman filter start!")

                Filtering = kalman_Filter()
                Filtering.first_mag_cal()
                #for i in range(0,10000):
                #        Filtering.covariance_test()

                Filtering.final()
        except rospy.ROSInterruptException:
                print "ROS terminated"
                pass
