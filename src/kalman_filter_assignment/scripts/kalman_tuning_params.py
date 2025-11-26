#!/usr/bin/env python3
import numpy as np
from math import pi

# initial covariance
IC = 0.5

# imu noise
IMU_YAW = 2.0
IMU_RATE = 0.02

# noise matrix
Q_DRIFT = 0.04
Q_YAW = 1.5

# gps noise
R_GPS = 0.03

# R_odom 1 x,y + yaw 
ODOM_XY = 0.35
ODOM_YAW = 9.0


