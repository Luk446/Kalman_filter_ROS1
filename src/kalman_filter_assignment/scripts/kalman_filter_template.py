#!/usr/bin/env python3
from re import U
import rospy
import numpy as np
from math import sin, cos, atan2, pi
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from kalman_tuning_params import Q_DRIFT, Q_YAW, IC, R_GPS, ODOM_XY, ODOM_YAW, USE_ODOM, USE_IMU, USE_GPS

class SimpleKalmanFilterNode:
    """
    Extended Kalman Filter (EKF) for robot localization.

    State: [x, y, yaw]^T  (position in world frame, heading)

    Motion model (non-linear):
        x'   = x + (vx*cos(yaw) - vy*sin(yaw)) * dt
        y'   = y + (vx*sin(yaw) + vy*cos(yaw)) * dt
        yaw' = yaw + omega * dt

    where vx, vy are body-frame velocities from odometry,
    and omega is yaw rate from IMU.

    Architecture:
        - Prediction runs at odom rate (~10Hz) for smooth estimates
        - GPS correction runs when GPS arrives (~1Hz)
    """

    def __init__(self):

        rospy.init_node('kalman_filter_simple', anonymous=True)

        # Use instance flags (allows overriding via ROS params if desired)
        # Default to values imported from kalman_tuning_params if available
        self.use_odom = rospy.get_param('~use_odom', USE_ODOM)
        self.use_imu  = rospy.get_param('~use_imu',  USE_IMU)
        self.use_gps  = rospy.get_param('~use_gps',  USE_GPS)

        # param
        self.dt = rospy.get_param('~dt', 0.1)  # Default time step

        # Subscribers (only subscribe if the corresponding sensor is enabled)
        if self.use_gps:
            rospy.Subscriber('/fake_gps', Odometry, self.gps_callback)
        if self.use_odom:
            rospy.Subscriber('/odom1', Odometry, self.odom1_callback)
        if self.use_imu:
            rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Publisher
        self.pub = rospy.Publisher('/kalman_estimate', Odometry, queue_size=10)

        # ----------------- state representation -----------------------

        # Initial State: [x, y, yaw]
        self.x = np.zeros((3,1))
        self.P = np.eye(3) * IC  # Initial covariance

        # IMU state
        self.imu_yaw = None
        self.imu_yaw_rate = None

        # ----------------- noise models -----------------------

        # Process noise standard deviations (will be scaled by dt)
        self.sigma_xy = Q_DRIFT      # Position drift std dev (m/sqrt(s))
        self.sigma_yaw = Q_YAW * pi / 180.0  # Yaw drift std dev (rad/sqrt(s))

        # GPS measurement noise covariance
        self.R_gps = np.diag([R_GPS**2, R_GPS**2])

        # Noisy odom measurement noise (for potential odom updates)
        self.R_odom1 = np.diag([ODOM_XY**2, ODOM_XY**2, (ODOM_YAW*pi/180.0)**2])

        # Latest motion information (body frame velocities)
        self.vx = 0.0       # body-frame forward velocity
        self.vy = 0.0       # body-frame lateral velocity
        self.yaw_rate = 0.0 # yaw rate from IMU

        # Odometry velocities (body frame)
        self.odom_vx = None
        self.odom_vy = None

        # Latest measurements
        self.gps = None            # Stored GPS measurement for correction
        self.odom1 = None
        self.last_prediction_time = None
        self.initialized = False   # Track if filter has been initialized

        rospy.loginfo("Extended Kalman Filter (EKF) initialized")
        rospy.loginfo(f"Sensor usage -> odom: {self.use_odom}, gps: {self.use_gps}, imu: {self.use_imu}")

    # ----------------- storage callbacks -----------------------

    # latest GPS (x,y) as 2x1 - triggers correction step
    def gps_callback(self, msg):
        self.gps = np.array([[msg.pose.pose.position.x],
                             [msg.pose.pose.position.y]])
        # GPS correction will be applied on next odom callback (or next trigger)

    # noisy odom  (x,y,yaw) - triggers prediction step
    def odom1_callback(self, msg):
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] # conv from quat
        self.odom1 = np.array([[pos_x], [pos_y], [yaw]])     # store as 3x1
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vy = msg.twist.twist.linear.y

        # Get timestamp for prediction
        stamp = msg.header.stamp.to_sec()
        if stamp == 0.0:
            stamp = rospy.Time.now().to_sec()

        # Run prediction at odom rate (10Hz)
        self.run_prediction_and_update(stamp)

    # imu
    def imu_callback(self, msg):
        quat = [msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w]
        self.imu_yaw = euler_from_quaternion(quat)[2] # conv from quat
        self.imu_yaw_rate = msg.angular_velocity.z    # store angular v

    # method for keeping yaw between (-n,n)
    @staticmethod
    def _normalize_angle(angle):
        return atan2(sin(angle), cos(angle)) # reduce to -pi to pi

    # ----------------- EKF Motion Model -----------------------

    def predict_state(self, x, vx, vy, omega, dt):
        """
        Non-linear motion model: transform body-frame velocities to world frame.

        x_new = x + (vx*cos(yaw) - vy*sin(yaw)) * dt
        y_new = y + (vx*sin(yaw) + vy*cos(yaw)) * dt
        yaw_new = yaw + omega * dt
        """
        yaw = x[2, 0]
        c_yaw = cos(yaw)
        s_yaw = sin(yaw)

        x_new = np.zeros((3, 1))
        x_new[0, 0] = x[0, 0] + (vx * c_yaw - vy * s_yaw) * dt
        x_new[1, 0] = x[1, 0] + (vx * s_yaw + vy * c_yaw) * dt
        x_new[2, 0] = x[2, 0] + omega * dt

        return x_new

    def compute_jacobian_F(self, x, vx, vy, dt):
        """
        Compute the Jacobian of the motion model w.r.t. state.

        F = d(f(x,u))/dx = [[1, 0, (-vx*sin(yaw) - vy*cos(yaw))*dt],
                           [0, 1, (vx*cos(yaw) - vy*sin(yaw))*dt],
                           [0, 0, 1]]
        """
        yaw = x[2, 0]
        c_yaw = cos(yaw)
        s_yaw = sin(yaw)

        F = np.eye(3)
        F[0, 2] = (-vx * s_yaw - vy * c_yaw) * dt
        F[1, 2] = (vx * c_yaw - vy * s_yaw) * dt

        return F

    def compute_process_noise(self, dt):
        """
        Compute process noise covariance scaled by dt.
        Process noise increases with time step.
        """
        Q = np.diag([
            (self.sigma_xy * dt)**2,      # x position noise
            (self.sigma_xy * dt)**2,      # y position noise
            (self.sigma_yaw * dt)**2      # yaw noise
        ])
        return Q

    # ----------------- periodic filtering -----------------------

    def run_prediction_and_update(self, measurement_time):
        """Run EKF prediction at odom rate, apply GPS correction when available."""

        # Require odom data only if odom is used
        if self.use_odom and (self.odom_vx is None or self.odom_vy is None):
            # Wait until we have odom velocities before first prediction
            self.last_prediction_time = measurement_time
            return

        # Require IMU yaw rate only if IMU is used
        if self.use_imu and (self.imu_yaw_rate is None):
            # Wait until we have IMU yaw rate before first prediction
            self.last_prediction_time = measurement_time
            return

        # INITIAL TIMESTAMP SETUP (keep this one)
        if self.last_prediction_time is None:
            self.last_prediction_time = measurement_time
            return

        # Compute dt exactly once
        dt = measurement_time - self.last_prediction_time
        if dt <= 0.0:
            dt = self.dt
        self.last_prediction_time = measurement_time

        # Use odom values only when odom is enabled; otherwise assume zero velocities
        if not self.use_odom:
            # If odom is disabled, keep odom_vx/odom_vy at 0.0 for predictions
            vx_input = 0.0
            vy_input = 0.0
        else:
            vx_input = self.odom_vx if self.odom_vx is not None else 0.0
            vy_input = self.odom_vy if self.odom_vy is not None else 0.0

        # If IMU not used, set yaw rate to zero
        if not self.use_imu:
            omega_input = 0.0
        else:
            omega_input = self.imu_yaw_rate if self.imu_yaw_rate is not None else 0.0

        # ---------- EKF Prediction Step ----------

        # Non-linear state prediction
        x_pred = self.predict_state(self.x, vx_input, vy_input, omega_input, dt)
        x_pred[2, 0] = self._normalize_angle(x_pred[2, 0])

        # Compute Jacobian for covariance propagation
        F = self.compute_jacobian_F(self.x, vx_input, vy_input, dt)

        # Process noise scaled by dt
        Q = self.compute_process_noise(dt)

        # Propagate covariance
        P_pred = F @ self.P @ F.T + Q

        # Store motion state used for publishing diagnostics
        self.vx = float(vx_input)
        self.vy = float(vy_input)
        self.yaw_rate = float(omega_input)

        # ---------- EKF Update Step (GPS correction when available) ----------
        if self.use_gps and self.gps is not None:
            # GPS measures x, y position directly
            H_gps = np.array([[1, 0, 0],
                              [0, 1, 0]])
            z = self.gps

            # Innovation (measurement residual)
            y = z - H_gps @ x_pred

            # Innovation covariance
            S = H_gps @ P_pred @ H_gps.T + self.R_gps

            # Kalman gain
            K = P_pred @ H_gps.T @ np.linalg.inv(S)

            # State update
            x_upd = x_pred + K @ y

            # Covariance update (Joseph form for numerical stability)
            I_KH = np.eye(3) - K @ H_gps
            P_upd = I_KH @ P_pred @ I_KH.T + K @ self.R_gps @ K.T

            # Clear GPS after use
            self.gps = None
        else:
            # No GPS, just use prediction
            x_upd = x_pred
            P_upd = P_pred

        # Normalize yaw angle
        x_upd[2, 0] = self._normalize_angle(x_upd[2, 0])

        # Update state
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
