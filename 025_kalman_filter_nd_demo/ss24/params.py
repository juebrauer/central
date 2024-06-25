import numpy as np


dt = 1.0 # simulation resolution / increment
max_simu_time = 60 # 1 minute

start_pos = 0 # meter
start_speed = 13.8 # [m/s] = ca. 50 km/h

# PROCESS PARAMS ---------------------------------------

# true_F * x = | 1.0 dt  |  | pos   | = | pos + dt*speed |
#              | 0.0 1.0 |  | speed |   | speed          |
#
#                  (2,2)      (2,1)  -->  (2,1)

true_F = np.array( [[1.0, dt],
                    [0.0, 1.0]])

u = np.array( [0.003858025] ) # [m/s^2]
# e.g. a car accelerates from 100km/h to 150 km/h in one hour:
# a [m/s^2] = delta_speed / delta_time = 50000m / (3600^2)s^2 = 0.003858025 m/s^2

# [pos_change, speed_change] = [0*acceleration, dt*acceleration]

# true_B * u = | 0  | | a | = | 0*a  |
#              | dt |         | dt*a |
true_B = np.array( [[0],
                    [dt]]) 

# process noise covariance matrix Q
true_Q = np.array( [[1.0, 0.0],
                    [0.0, 0.01]])

# true measurement matrix
true_H = np.array( [[0.98, 0.0],
                    [0.0,  1.02]])

# measurement noise covariance matrix R
true_R = np.array( [[1000.0, 0.0],
                    [0.0,   1.0]])