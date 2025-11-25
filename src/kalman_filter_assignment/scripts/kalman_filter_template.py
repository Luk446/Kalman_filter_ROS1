#!/usr/bin/env python3
import rospy
import numpy as np
from math import sin, cos, atan2, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class SimpleKalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_simple', anonymous=True)

        # param
        self.dt = rospy.get_param('~dt', 0.1)  # Time step

        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/fake_gps', Odometry, self.gps_callback)      
        rospy.Subscriber('/odom1', Odometry, self.odom1_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Publisher 
        self.pub = rospy.Publisher('/kalman_estimate', Odometry, queue_size=10)

        # Initial State: [x, y, yaw]
        self.x = np.zeros((3,1))
        self.P = np.eye(3) * 0.5  # more uncertain

        # initial imu state
        self.imu_yaw = None
        self.imu_yaw_rate = None

        # Noise 

        # set noise for imu
        self.R_imu_yaw = np.array([[ (1.0*pi/180.0)**2 ]])   # 1° std dev
        self.R_imu_rate = np.array([[ (0.02)**2 ]])          # 0.02 rad/s std dev

        # Process noise, vels and yaw
        self.Q = np.diag([0.05**2, 0.05**2, (2.0*pi/180.0)**2])  # ~5cm std, ~2deg std
        # gps noise
        self.R_gps = np.diag([0.02**2, 0.02**2])  # 2cm std
        # Noisy odom measurement noise (x,y,yaw)
        self.R_odom1 = np.diag([0.20**2, 0.20**2, (5.0*pi/180.0)**2])  # 20cm std, 5deg std

        # Latest command velocities
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0
        self.yaw_rate_input = 0.0  # what the filter actually integrates

        # Latest measurements
        self.gps = None            
        self.odom1 = None          

        # Timer for Kalman update
        rospy.Timer(rospy.Duration.from_sec(self.dt), self.update_kalman)

        rospy.loginfo("Kalman Filter start")

    def cmd_vel_callback(self, msg):
        """Store the latest cmd velocities."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.yaw_rate = msg.angular.z

    def gps_callback(self, msg):
        """Store the latest GPS (accurate) measurement (x,y)."""
        self.gps = np.array([[msg.pose.pose.position.x],
                              [msg.pose.pose.position.y]])

    def odom1_callback(self, msg):
        """Store the latest noisy odom measure (x,y,yaw)."""
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.odom1 = np.array([[pos_x], [pos_y], [yaw]])

    def imu_callback(self, msg):
        quat = [msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w]
        self.imu_yaw = euler_from_quaternion(quat)[2]
        self.imu_yaw_rate = msg.angular_velocity.z    

    @staticmethod
    def _normalize_angle(angle):
        return atan2(sin(angle), cos(angle))

    def update_kalman(self, event):

        dt = self.dt # time step for integration

        # --- Prediction ---
        # Prefer IMU yaw rate if available, otherwise fall back to cmd_vel
        yaw_rate_input = self.imu_yaw_rate if self.imu_yaw_rate is not None else self.yaw_rate
        self.yaw_rate_input = yaw_rate_input

        # State transition x_k+1 = x_k + dt * [vx, vy, yaw_rate]
        u = np.array([[self.vx], [self.vy], [yaw_rate_input]])
        F = np.eye(3)
        B = dt * np.eye(3)
        x_pred = F @ self.x + B @ u
        x_pred[2,0] = self._normalize_angle(x_pred[2,0])

        # Covariance prediction: P' = F P F^T + Q
        P_pred = F @ self.P @ F.T + self.Q

        # --- Correction ---
        x_upd = x_pred
        P_upd = P_pred

        # 1) GPS correction (measures x,y)
        if self.gps is not None:
            H_gps = np.array([[1,0,0],[0,1,0]])  # map state to measurement space
            z = self.gps  # shape (2,1)
            y = z - H_gps @ x_upd                  # innovation
            S = H_gps @ P_upd @ H_gps.T + self.R_gps
            K = P_upd @ H_gps.T @ np.linalg.inv(S) # Kalman gain
            x_upd = x_upd + K @ y
            P_upd = (np.eye(3) - K @ H_gps) @ P_upd

        # 2) Noisy odom correction (measures x,y,yaw)
        if self.odom1 is not None:
            H_o = np.eye(3)
            z = self.odom1
            # Normalize yaw innovation 
            y = z - H_o @ x_upd
            y[2,0] = self._normalize_angle(y[2,0])
            S = H_o @ P_upd @ H_o.T + self.R_odom1
            K = P_upd @ H_o.T @ np.linalg.inv(S)
            x_upd = x_upd + K @ y
            P_upd = (np.eye(3) - K @ H_o) @ P_upd

        # 3) IMU yaw correction (measures yaw only)
        if self.imu_yaw is not None:
            H_imu = np.array([[0.0, 0.0, 1.0]])
            z = np.array([[self.imu_yaw]])
            y = z - H_imu @ x_upd
            y[0,0] = self._normalize_angle(y[0,0])
            S = H_imu @ P_upd @ H_imu.T + self.R_imu_yaw
            K = P_upd @ H_imu.T @ np.linalg.inv(S)
            x_upd = x_upd + K @ y
            P_upd = (np.eye(3) - K @ H_imu) @ P_upd


        # Final state
        x_upd[2,0] = self._normalize_angle(x_upd[2,0])
        self.x = x_upd
        self.P = P_upd

        self.publish_estimate()

    def publish_estimate(self):
        """Publish the current state estimate as Odometry message."""
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"

        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])

        q = quaternion_from_euler(0,0,float(self.x[2]))
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.twist.twist.linear.x = float(self.vx)
        msg.twist.twist.linear.y = float(self.vy)
        msg.twist.twist.angular.z = float(self.yaw_rate_input)

        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        node = SimpleKalmanFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

