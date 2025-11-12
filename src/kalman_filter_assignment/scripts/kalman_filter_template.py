#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler

class SimpleKalmanFilterNode:
    def __init__(self):
        rospy.init_node('kalman_filter_simple', anonymous=True)

        # param
        self.dt = rospy.get_param('~dt', 0.1)  # Time step

        #subscriber
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/fake_gps', Odometry, self.gps_callback)

        # Publisher 
        self.pub = rospy.Publisher('/kalman_estimate', Odometry, queue_size=10)

        # Initial State: [x, y, yaw]
        self.x = np.zeros((3,1))
        self.P = np.eye(3) * 0.1

        # Noise Covariances
        self.Q = 0 # process noise. What should be the values here? 
        self.R = 0 # GPS measurement noise

        # Latest command velocities
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0

        # Latest GPS measurement
        self.gps = None

        # Timer for Kalman update
        rospy.Timer(rospy.Duration(self.dt), self.update_kalman)

        rospy.loginfo("Kalman Filter start")

    def cmd_vel_callback(self, msg):
        """Store the latest cmd velocities."""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.yaw_rate = msg.angular.z

    def gps_callback(self, msg):
        """Store the latest GPS measurement."""
        self.gps = np.array([
            [msg.pose.pose.position.x],
            [msg.pose.pose.position.y]
        ])

    def update_kalman(self, event):
        """
        This is the main Kalman filter loop. In this function
        you should do a prediction plus a correction step. """
        # --- Prediction ---
        x_pred = np.zeros((3,1))

	# what goes here??? 
	
        self.x = x_pred
        self.P = P_pred

        # --- Correction ---
        
        
        self.x = 0 # what should go here? 
        

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

