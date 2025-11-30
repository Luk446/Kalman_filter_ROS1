#!/usr/bin/env python3
import numpy as np
from math import pi

# initial covariance
IC = 0.5

# Process noise matrix (scaled by dt in the filter)
# These represent uncertainty in the motion model
# Lower Q = trust motion model more = smoother between GPS updates
Q_DRIFT = 0.05      # Position drift std dev (m/s)
Q_YAW = 2.0         # Yaw drift std dev (deg/s)

# GPS measurement noise - larger value = trust GPS less = smoother corrections
# Higher R_GPS prevents abrupt jumps when GPS arrives
R_GPS = 0.25        # GPS position std dev (m)

# R_odom 1 x,y + yaw 
ODOM_XY = 0.35
ODOM_YAW = 9.0

# sensor config
USE_ODOM = True
USE_IMU = True
USE_GPS = True
