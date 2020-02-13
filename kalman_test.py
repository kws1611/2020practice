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

class kalman_Filter:

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
                self.X = np.matrix('1;0;0;0')
                self.P = np.identity(4)
                self.dt = float(1.0/78.5)
                self.H =  np.identity(4)

                # the gyro sensor standard variation
                self.Q = 10.0**(-10)*np.matrix([[1.0, 0, 0, 0],[0, 1.50628058**2, 0, 0],[0, 0, 1.4789602**2, 0],[0, 0, 0, 1.37315181**2]])

                # accelometer and magnetometer's standard variation
                self.R = 5*np.matrix([[0.00840483082215**2, 0, 0, 0],[0, 0.00100112198402**2, 0, 0], [0, 0, 0.00102210818946**2, 0], [0, 0, 0, 0.0114244938775**2]])

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
                self.rate = rospy.Rate(78.5)

                rospy.Subscriber("/imu_raw", Imu, self.imu_raw_data)
                rospy.Subscriber("/mag_raw", MagneticField, self.mag_raw_data)

                self.Kalman_cov_pub = rospy.Publisher("/pose_covariance",PoseWithCovarianceStamped, queue_size=1)

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
                #normalize the accel value
                self.ax = self.acc_x / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
                self.ay = self.acc_y / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)
                self.az = self.acc_z / math.sqrt(self.acc_x**2 +self.acc_y**2 + self.acc_z**2)

                if self.az >= 0:
                        self.q_acc = np.matrix([math.sqrt(0.5*(self.az + 1)), -self.ay/(2*math.sqrt(0.5*(self.az+1))), self.ax/(2*math.sqrt(0.5*(self.az+1))), 0])
                else :
                        self.q_acc_const = math.sqrt((1.0-self.az) * 0.5)
                        self.q_acc = np.matrix([-self.ay/(2.0*self.q_acc_const), self.q_acc_const, 0.0, self.ax/(2.0*self.q_acc_const)])

        def state_check(self):
                # when gyro is almost 0 and accel's x and y axis is almost 0 then the system will assume the left over value is gyro bias
                gyro_scale = math.sqrt(self.gyro_x**2 + self.gyro_y**2 + self.gyro_z**2)
                acc_scale = math.sqrt(self.acc_x**2 + self.acc_y**2)
                if gyro_scale < 0.01 and acc_scale < 0.1 :
                        return True
                else :
                        return False

        def gyro_bias_update(self):
                # getting rid of gyro bias
                if self.state_check():
                        self.gyro_bias_x += self.gyro_x
                        self.gyro_bias_y += self.gyro_z
                        self.gyro_bias_z += self.gyro_y

                        self.gyro_x -= self.gyro_bias_x
                        self.gyro_y -= self.gyro_bias_y
                        self.gyro_z -= self.gyro_bias_z

        def get_mag_quat(self):
                #rotating the magnetometer's value using accelometer's quaternion
                lx, ly, lz = rotateVectorQuaternion(self.mag_x, self.mag_y, self.mag_z, self.q_acc[0,0], -self.q_acc[0,1], -self.q_acc[0,2], -self.q_acc[0,3])
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
        def get_mag_acc_calibration(self):
                # calculating the starting yaw (the angle between the north)
                # and get rid of the starting yaw
                self.ax_cal = self.acc_cal_x / math.sqrt(self.acc_cal_x**2 +self.acc_cal_y**2 + self.acc_cal_z**2)
                self.ay_cal = self.acc_cal_y / math.sqrt(self.acc_cal_x**2 +self.acc_cal_y**2 + self.acc_cal_z**2)
                self.az_cal = self.acc_cal_z / math.sqrt(self.acc_cal_x**2 +self.acc_cal_y**2 + self.acc_cal_z**2)
                self.q_acc_cal = np.matrix([math.sqrt(0.5*(self.az_cal + 1)), -self.ay_cal/(2*math.sqrt(0.5*(self.az_cal+1))), self.ax_cal/(2*math.sqrt(0.5*(self.az_cal+1))), 0])

                self.mag_cal_x, self.mag_cal_y, self.mag_cal_z = normalization(self.mag_cal_x, self.mag_cal_y, self.mag_cal_z)
                lx_cal, ly_cal, lz_cal = rotateVectorQuaternion(self.mag_cal_x, self.mag_cal_y, self.mag_cal_z, -self.q_acc_cal[0,0], -self.q_acc_cal[0,1], -self.q_acc_cal[0,2], -self.q_acc_cal[0,3])
                self.gamma_cal = lx_cal ** 2 + ly_cal ** 2
                if lx_cal >= 0:
                        self.q0_mag_cal = math.sqrt(self.gamma_cal + lx_cal * math.sqrt(self.gamma_cal))/ math.sqrt(2 * self.gamma_cal)
                        self.q1_mag_cal = 0
                        self.q2_mag_cal = 0
                        self.q3_mag_cal = ly_cal / math.sqrt(2 * (self.gamma_cal + lx_cal * math.sqrt(self.gamma_cal)))
                        self.q_mag_cal= norm_quat(self.q0_mag_cal, self.q1_mag_cal, self.q2_mag_cal, self.q3_mag_cal)
                if lx_cal < 0:
                        self.q0_mag_cal = ly_cal / math.sqrt(2 * (self.gamma_cal - lx_cal * math.sqrt(self.gamma_cal)))
                        self.q1_mag_cal = 0
                        self.q2_mag_cal = 0
                        self.q3_mag_cal = math.sqrt(self.gamma_cal - lx_cal * math.sqrt(self.gamma_cal))/ math.sqrt(2 * self.gamma_cal)
                        self.q_mag_cal= norm_quat(self.q0_mag_cal, self.q1_mag_cal, self.q2_mag_cal, self.q3_mag_cal)

        def kalman(self):
                pose_topic = PoseWithCovarianceStamped()
                #self.gyro_bias_update()
                self.get_acc_quat()
                self.get_mag_quat()

                self.q_mag_calibrating = quat_mult(self.q_acc[0,0],self.q_acc[0,1],self.q_acc[0,2],self.q_acc[0,3],self.q_mag[0,0],self.q_mag[1,0],self.q_mag[2,0],self.q_mag[3,0])
                self.Z = quat_mult(self.q_mag_calibrating[0,0],self.q_mag_calibrating[1,0],self.q_mag_calibrating[2,0],self.q_mag_calibrating[3,0],self.q_mag_cal[0,0],-self.q_mag_cal[1,0],-self.q_mag_cal[2,0],-self.q_mag_cal[3,0])
                self.Z = norm_quat(self.Z[0,0],self.Z[1,0],self.Z[2,0],self.Z[3,0])
                #making the gyro matrix
                self.A = np.identity(4)-self.dt*0.5*np.matrix([[0,-self.gyro_x,-self.gyro_y,-self.gyro_z],[self.gyro_x,0,-self.gyro_z,self.gyro_y],[self.gyro_y,self.gyro_z,0,-self.gyro_x],[self.gyro_z,-self.gyro_y,self.gyro_x,0]])
                # Kalman Filter

                #calculating the predict value (gyro)
                self.Xp = self.A*self.X
                #normalize the quaternion
                self.Xp = norm_quat(self.Xp[0,0],self.Xp[1,0],self.Xp[2,0],self.Xp[3,0])
                #calculat predict covariance value
                self.Pp = self.A*self.P*self.A.T +self.Q
                #calculate kalman gain
                self.K = self.Pp*self.H.T*lin.inv(self.H*self.Pp*self.H.T + self.R)

                if (self.Xp[0,0] - self.Z[0,0])>1 or (self.Xp[0,0] - self.Z[0,0])< -1 or (self.Xp[1,0] - self.Z[1,0])>1 or (self.Xp[1,0] - self.Z[1,0])< -1 or (self.Xp[2,0] - self.Z[2,0])>1 or (self.Xp[2,0] - self.Z[2,0])< -1 or (self.Xp[3,0] - self.Z[3,0])>1 or (self.Xp[3,0] - self.Z[3,0])< -1 :
                        # change the + -
                        # + - multiplication is same quaternion
                        self.Z = np.matrix([-self.Z[0,0],-self.Z[1,0],-self.Z[2,0],-self.Z[3,0]])
                        self.Z = self.Z.T
                # calculating  the quaternion using the sensor fusion
                self.X = self.Xp + self.K*(self.Z - self.H*self.Xp)
                # normalize the quaternion
                self.X = norm_quat(self.X[0,0],self.X[1,0],self.X[2,0],self.X[3,0])
                # calculating the covariance
                self.P = self.Pp - self.K*self.H*self.Pp


                #self.X = self.Z
                pose_topic.header.stamp.secs = self.imu_secs
                pose_topic.header.stamp.nsecs = self.imu_nsecs
                pose_topic.header.frame_id = "world"
                pose_topic.pose.pose.position.x = 0
                pose_topic.pose.pose.position.y = 0
                pose_topic.pose.pose.position.z = 0
                pose_topic.pose.pose.orientation.x = -self.X[1,0]
                pose_topic.pose.pose.orientation.y = -self.X[2,0]
                pose_topic.pose.pose.orientation.z = -self.X[3,0]
                pose_topic.pose.pose.orientation.w = self.X[0,0]
                pose_topic.pose.covariance = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0289, 0.0289, 0.1207,0,0,0,0.0289, 0.0289, 0.1207,0,0,0,0.1207,0.1207,0.5041]
                self.Kalman_cov_pub.publish(pose_topic)
                self.rate.sleep()

if __name__ == "__main__":

        rospy.init_node("Kalman_Filter", anonymous=True)
        rospy.loginfo("Kalman filter node initialized")

        try:
                rospy.loginfo("Kalman filter start!")
                # initialize the class
                Filtering = kalman_Filter()
                # starting the calibration
                Filtering.first_mag_cal()
                # calculating the first yaw calculation
                Filtering.get_mag_acc_calibration()
                while not rospy.is_shutdown():
                        #kalman filter starting
                        Filtering.kalman()
        except rospy.ROSInterruptException:
                print "ROS terminated"
                pass
