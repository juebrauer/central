import numpy as np

# how long to simulate?
max_simu_time = 60 # [s]

# simulation time increment
dt = 1.0

# start position and speed of the car
start_pos = 0.0
start_speed = 13.8 # [m/s] = ca. 50 km/h

# prepare matrices for simulation

# prepare state transition matrix
# [new_pos, new_speed] = [pos+dt*speed, speed]
# |1.0 dt |   | pos   |   | pos + dt*speed |
# |       | @ |       | = |                |
# |0.0 1.0|   | speed |   | speed          |
true_F = np.array( [[1.0, dt],
                    [0.0, 1.0]])

# prepare control matrix
# [pos_change, speed_change] = []
# car accelerates from 100km/h to 150km/h in one hour:
# [m/s^2] = delta_speed / delta_time =
#         = 50000m / 3600^2
u = np.array( [0.003858025] ) * 3.0 # m/s^2
true_B = np.array( [[0.0],
                    [dt]])

# define process noise covariance matrix
true_Q = np.array( [[1.0, 0.0],
                    [0.0, 0.01]])

# define measurement matrix
true_H = np.array( [[1.0, 0.0],
                    [0.0, 1.0]])
true_R = np.array( [[10,  0.0],
                    [0.0, 1.0]])


