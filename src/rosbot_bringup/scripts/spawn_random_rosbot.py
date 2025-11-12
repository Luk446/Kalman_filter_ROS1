#!/usr/bin/env python3
import rospy
import hashlib, random, math, subprocess

def deterministic_random_pose(student_name):
    seed = int(hashlib.sha256(student_name.encode('utf-8')).hexdigest(), 16) % (2**32)
    random.seed(seed)
    x = random.uniform(-5, 5)
    y = random.uniform(-5, 5)
    yaw = random.uniform(-math.pi, math.pi)
    return x, y, yaw

if __name__ == "__main__":
    rospy.init_node("spawn_random_rosbot")

    student_name = rospy.get_param("~student_name", "default_student")
    robot_name = rospy.get_param("~robot_name", "rosbot")

    # Wait until 'robot_description' is available
    rospy.loginfo("Waiting for robot_description param...")
    while not rospy.has_param('robot_description') and not rospy.is_shutdown():
        rospy.sleep(0.1)
    rospy.loginfo("robot_description found!")

    x, y, yaw = deterministic_random_pose(student_name)
    rospy.loginfo(f"Spawning {robot_name} at x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°")

    # Call gazebo_ros spawn_model service via subprocess
    cmd = [
        "rosrun", "gazebo_ros", "spawn_model",
        "-param", "robot_description",
        "-urdf",
        "-model", robot_name,
        "-x", str(x),
        "-y", str(y),
        "-z", "0",
        "-Y", str(yaw)
    ]
    subprocess.call(cmd)

