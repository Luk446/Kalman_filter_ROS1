#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import copy

class FakeOdometryAndGPS:
    """
    ROS node that:
    - Subscribes to /odom and publishes /odom1 (noisy + drifting odometry)
    - Subscribes to /gazebo/model_states and publishes /fake_gps (absolute pose at 1Hz)
    
    In this way the user has available two nodes that can be used for developing your own Kalman filter and evaluate it
    against /odom. It is impornta to keep in mind that all published topics here use the /odom frame so there is no need for transforms. 
    """

    def __init__(self):
        rospy.init_node('fake_odometry_and_gps', anonymous=True)

        # === Parameters ===
        # Odometry noise and drift
        self.position_noise_std = rospy.get_param('~position_noise_std', 0.3)
        self.orientation_noise_std = rospy.get_param('~orientation_noise_std', 0.02)
        self.velocity_noise_std = rospy.get_param('~velocity_noise_std', 0.01)
        self.angular_velocity_noise_std = rospy.get_param('~angular_velocity_noise_std', 0.005)
        self.position_drift_rate = np.array(rospy.get_param('~position_drift_rate', [0.05, 0.01, 0.0]))
        self.orientation_drift_rate = np.array(rospy.get_param('~orientation_drift_rate', [0.0, 0.0, 0.0005]))

        # Fake GPS noise and update rate
        self.gps_position_noise_std = rospy.get_param('~gps_position_noise_std', 0.01)
        self.gps_rate = rospy.get_param('~gps_rate', 1.0)

        # Drift accumulators 
        self.position_drift_acc = np.zeros(3)
        self.orientation_drift_acc = np.zeros(3)
        self.last_time = None

        # === ROS setup ===
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback, queue_size=10)
        self.odom1_pub = rospy.Publisher('/odom1', Odometry, queue_size=10)
        self.gps_pub = rospy.Publisher('/fake_gps', Odometry, queue_size=10)

        # === GPS publishing control ===
        self.last_gps_pub_time = rospy.Time.now()

        # Robot name (depends on your Gazebo model name)
        self.robot_name = rospy.get_param('~robot_name', 'rosbot')

        rospy.loginfo("FakeOdometryAndGPS initialized.")
        rospy.loginfo("Publishing /odom1 (noisy+drifting) and /fake_gps (5Hz).")

    # -------------------- ODOMETRY MODIFICATION --------------------

    def odom_callback(self, msg):
        """Adds both noise and drift to odometry and publishes as /odom1."""
        current_time = rospy.Time.now()
        if self.last_time is not None:
            dt = (current_time - self.last_time).to_sec()
            self.position_drift_acc += self.position_drift_rate * dt
            self.orientation_drift_acc += self.orientation_drift_rate * dt
        self.last_time = current_time

        odom_mod = copy.deepcopy(msg)

        # --- Add position noise and drift ---
        odom_mod.pose.pose.position.x += np.random.normal(0, self.position_noise_std) + self.position_drift_acc[0]
        odom_mod.pose.pose.position.y += np.random.normal(0, self.position_noise_std) + self.position_drift_acc[1]
        odom_mod.pose.pose.position.z += np.random.normal(0, 0.1*self.position_noise_std) + self.position_drift_acc[2]

        # --- Add orientation noise and drift ---
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        roll, pitch, yaw = euler_from_quaternion(quat)
        roll += np.random.normal(0, self.orientation_noise_std) + self.orientation_drift_acc[0]
        pitch += np.random.normal(0, self.orientation_noise_std) + self.orientation_drift_acc[1]
        yaw += np.random.normal(0, self.orientation_noise_std) + self.orientation_drift_acc[2]

        new_quat = quaternion_from_euler(roll, pitch, yaw)
        odom_mod.pose.pose.orientation.x = new_quat[0]
        odom_mod.pose.pose.orientation.y = new_quat[1]
        odom_mod.pose.pose.orientation.z = new_quat[2]
        odom_mod.pose.pose.orientation.w = new_quat[3]

        # --- Add velocity noise ---
        odom_mod.twist.twist.linear.x += np.random.normal(0, self.velocity_noise_std)
        odom_mod.twist.twist.linear.y += np.random.normal(0, self.velocity_noise_std)
        odom_mod.twist.twist.angular.z += np.random.normal(0, self.angular_velocity_noise_std)

        odom_mod.header.stamp = current_time
        self.odom1_pub.publish(odom_mod)

    # -------------------- FAKE GPS --------------------

    def model_callback(self, msg):
        """
        Publishes /fake_gps at 5Hz using Gazebo ground truth with small noise.
        This "fake GPS" is a simplification as it gives position in meters. A true GPS will give you lat and long measurements
        which you then need to convert to a local map or reference frame. This is not hard to do and there are many functions that 
        can be used for it. But here we are just trying to learn how a Kalman filters works. 
        """
        now = rospy.Time.now()
        if (now - self.last_gps_pub_time).to_sec() < 1.0 / self.gps_rate:
            return  # enforce GPS rate limit

        if self.robot_name not in msg.name:
            rospy.logwarn_throttle(5, f"Robot '{self.robot_name}' not found in /gazebo/model_states.")
            return

        idx = msg.name.index(self.robot_name)
        pose = msg.pose[idx]
        twist = msg.twist[idx]

        gps_odom = Odometry()
        gps_odom.header.stamp = now
        # The correct thing will be to use "map"
        # But since I am using a fake GPS with gazebo ground trouth I am using the odom frame
        # which is the same as gazebo ground truth 
        gps_odom.header.frame_id = "odom"  

        # Add noise to position (simulate GPS)
        gps_odom.pose.pose.position.x = pose.position.x + np.random.normal(0, self.gps_position_noise_std)
        gps_odom.pose.pose.position.y = pose.position.y + np.random.normal(0, self.gps_position_noise_std)
        gps_odom.pose.pose.position.z = pose.position.z

        # Orientation: can keep from ground truth (optional)
        gps_odom.pose.pose.orientation = pose.orientation

        # Velocity (optional, from ground truth)
        gps_odom.twist.twist = twist

        self.gps_pub.publish(gps_odom)
        self.last_gps_pub_time = now


if __name__ == '__main__':
    try:
        node = FakeOdometryAndGPS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

