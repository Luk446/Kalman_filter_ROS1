# Extended kalman filter instructions

- integrate IMU data with your system
- tune EKF filter
- provide a systematic analysis of the resulting localisation error -- should include localisayion performances with different combinations of sensors (odometry only, odom+gps, gps+imu odom+gps+imu)

# rosbag record command

""
rosbag record -O kf_test /cmd_vel /fake_gps /odom1 /imu /kalman_estimate /odom
""


# documentation

## initial ekf  (v4)

IC = 0.5
IMU_YAW = 1.0
IMU_RATE = 0.02
Q_DRIFT = 0.05
Q_YAW = 2.0
R_GPS = 0.02
ODOM_XY = 0.20
ODOM_YAW = 5.0

### observation

close to ground truth but jagged

## v5

IC = 0.5
IMU_YAW = 2.0
IMU_RATE = 0.02
Q_DRIFT = 0.02
Q_YAW = 1.0
R_GPS = 0.05
ODOM_XY = 0.30
ODOM_YAW = 8.0

### observation

smooth but further displaced - less noise but worse performance?

## v6 

IC = 0.5
IMU_YAW = 2.0
IMU_RATE = 0.02
Q_DRIFT = 0.04
Q_YAW = 1.5
R_GPS = 0.03
ODOM_XY = 0.30
ODOM_YAW = 8.0


### obsvervations

odom still erratic?

## v7

we are upping the odom noise factor

IC = 0.5
IMU_YAW = 2.0
IMU_RATE = 0.02
Q_DRIFT = 0.04
Q_YAW = 1.5
R_GPS = 0.03
ODOM_XY = 0.35
ODOM_YAW = 9.0
