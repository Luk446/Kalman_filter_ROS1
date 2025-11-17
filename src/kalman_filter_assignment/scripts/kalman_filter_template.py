#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class SimpleKalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_simple', anonymous=True)

        # param
        self.dt = rospy.get_param('~dt', 0.1)  # Time step

        # Subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/fake_gps', Odometry, self.gps_callback)      # Accurate, low rate position (x,y)
        rospy.Subscriber('/odom1', Odometry, self.odom1_callback)       # Noisy, high rate (x,y,yaw)

        # Publisher 
        self.pub = rospy.Publisher('/kalman_estimate', Odometry, queue_size=10)

        # Initial State: [x, y, yaw]
        self.x = np.zeros((3,1))
        self.P = np.eye(3) * 0.5  # start with moderate uncertainty

        # Noise Covariances (tunable)
        # Process noise: uncertainty in executing commanded velocities and yaw rate
        self.Q = np.diag([0.05**2, 0.05**2, (2.0*np.pi/180.0)**2])  # ~5cm std, ~2deg std
        # GPS measurement noise (x,y only) - /fake_gps doesn't publish covariance, using empirical estimate
        self.R_gps = np.diag([0.02**2, 0.02**2])  # 2cm std
        # Noisy odom measurement noise (x,y,yaw) - from /odom1 pose.covariance: [0.0001, 0.0001, 0.01]
        # Noisy odom measurement noise (x,y,yaw)
        self.R_odom1 = np.diag([0.20**2, 0.20**2, (5.0*np.pi/180.0)**2])  # 20cm std, 5deg std

        # Latest command velocities
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0

        # Latest measurements
        self.gps = None            # shape (2,1)
        self.odom1 = None          # shape (3,1)

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
        """Store the latest noisy odometry measurement (x,y,yaw)."""
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.odom1 = np.array([[pos_x], [pos_y], [yaw]])

    @staticmethod
    def _normalize_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def update_kalman(self, event):
        """Kalman filter cycle: predict then (optionally) correct with available sensors.

        Assumptions:
        - Commanded linear velocities vx, vy are already expressed in the world frame (simplified).
        - Yaw rate is small between updates so simple integration is acceptable.
        - If both GPS and odom1 measurements are available, we fuse GPS first (more accurate positions),
          then refine with odom1 (adds yaw info and high-rate positional correction).
        """
        dt = self.dt
        # --- Prediction ---
        # State transition x_k+1 = x_k + dt * [vx, vy, yaw_rate]
        u = np.array([[self.vx], [self.vy], [self.yaw_rate]])
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
            H_gps = np.array([[1,0,0],[0,1,0]])  # maps state to measurement space
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
            # Normalize yaw innovation to avoid discontinuities
            y = z - H_o @ x_upd
            y[2,0] = self._normalize_angle(y[2,0])
            S = H_o @ P_upd @ H_o.T + self.R_odom1
            K = P_upd @ H_o.T @ np.linalg.inv(S)
            x_upd = x_upd + K @ y
            P_upd = (np.eye(3) - K @ H_o) @ P_upd

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
        msg.twist.twist.angular.z = float(self.yaw_rate)

        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        node = SimpleKalmanFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

