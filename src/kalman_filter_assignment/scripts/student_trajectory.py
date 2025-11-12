#!/usr/bin/env python3
"""
Student Trajectory Node
Generates different trajectories based on student name hash.
The robot will continuously execute the pattern forever.

Trajectories:
- Circle: Continuous circular motion
- Square: Move forward for 4s, turn 90° for 1s, repeat
- Figure-8: Half left, full right, full left, full right ... (returns to same spot)
"""

import rospy
from geometry_msgs.msg import Twist
import math


def get_trajectory(student_name):
    """
    Returns a generator that produces (linear_vel, angular_vel) commands
    based on a hash of the student name.
    """
    name_hash = sum(ord(c) for c in student_name) % 3

    rospy.loginfo(f"Student: {student_name}")
    rospy.loginfo(f"Trajectory hash: {name_hash}")

    if name_hash == 0:
        # ===== CIRCLE =====
        rospy.loginfo("Trajectory: CIRCLE")

        def circle():
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                yield (0.3, 0.3)  # v = 0.3 m/s, omega = 0.3 rad/s
                rate.sleep()
        return circle()

    elif name_hash == 1:
        # ===== SQUARE =====
        rospy.loginfo("Trajectory: SQUARE")

        def square():
            rate = rospy.Rate(10)
            t0 = rospy.Time.now().to_sec()

            while not rospy.is_shutdown():
                t = rospy.Time.now().to_sec() - t0
                cycle_time = t % 5.0  # 4s forward + 1s turn

                if cycle_time < 4.0:
                    linear = 0.2
                    angular = 0.0
                else:
                    linear = 0.0
                    angular = math.pi / 2.0  # 90°/s

                yield (linear, angular)
                rate.sleep()
        return square()

    else:
        # ===== FIGURE-8 =====
        rospy.loginfo("Trajectory: FIGURE-8 (½ left, full right, full left, full right...)")

        def figure8():
            rate = rospy.Rate(10)
            t0 = rospy.Time.now().to_sec()

            # Parameters
            linear_speed = rospy.get_param("~figure8_linear", 0.25)  # m/s
            radius = rospy.get_param("~figure8_radius", 1.0)         # m
            omega = linear_speed / radius                            # rad/s

            # Compute durations
            half_circle_time = math.pi / omega
            full_circle_time = 2 * math.pi / omega

            # Total cycle = 0.5 + 1 + 1 + 1 = 3.5 circles worth of motion
            total_cycle_time = half_circle_time + 3 * full_circle_time

            while not rospy.is_shutdown():
                t = rospy.Time.now().to_sec() - t0
                cycle_time = t % total_cycle_time

                # Define phases
                if cycle_time < half_circle_time:
                    # Half left circle
                    v = linear_speed
                    w = omega
                elif cycle_time < half_circle_time + full_circle_time:
                    # Full right circle
                    v = linear_speed
                    w = -omega
                elif cycle_time < half_circle_time + 2 * full_circle_time:
                    # Full left circle
                    v = linear_speed
                    w = omega
                else:
                    # Full right circle again
                    v = linear_speed
                    w = -omega

                yield (v, w)
                rate.sleep()

        return figure8()


if __name__ == "__main__":
    rospy.init_node("student_trajectory_node")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    student_name = rospy.get_param("~student_name", "student")
    traj_gen = get_trajectory(student_name)

    rospy.loginfo("Student trajectory node started. Publishing to /cmd_vel")
    rospy.loginfo("Press Ctrl+C to stop")

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            v, w = next(traj_gen)
            msg = Twist()
            msg.linear.x = v
            msg.angular.z = w
            pub.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        stop = Twist()
        pub.publish(stop)
        rospy.loginfo("Stopping robot")

