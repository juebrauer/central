import numpy as np

# how long to simulate?
max_simu_time = 60 # [s]

# simulation time increment
dt = 1.0

# prepare matrices for simulation

# define process noise covariance matrix
true_Q = np.array( [[0.01, 0.00, 0.00],
                    [0.00, 0.01, 0.00],
                    [0.00, 0.00, 1.00]])


true_R = np.array( [[0.01, 0.00, 0.00],
                    [0.00, 2.00, 0.00],
                    [0.00, 0.00, 2.00]])

measurement_each_nth_step = 5


