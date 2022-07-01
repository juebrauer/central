import numpy as np

# the state x in this demo is 2D:
# [pos, speed]    
# define:
# - start position and
# - start speed    
start_state0 = 0
start_state1 = 0
start_state2 = 13.8



# for how long do we want to simulate?
#max_simu_time = 60 # 1minute
#max_simu_time = 900.0 # 0.25h
max_simu_time = 600.0


# simulation time increment for one simulation loop
dt = 1.0


# how frequently do we correct the state estimate
# using new measurements?
measurement_correct_each_nth_step = 1

# f
# 60: 0.415
# 10: 0.213
#  5: 0.216
#  2: 0.150

# est_f
# 60: 1.697
# 10: 1.425
#  5: 1.386
#  2: 1.684
#  1: 2.591

# set process noise covariance matrix
true_Q = np.array( [[0.01, 0.0, 0.0],
                    [0.0,  0.1, 0.0],
                    [0.0, 0.0, 0.0]
                    ])

# set measurement noise covariance matrix
true_R = np.array( [[0.01, 0.0, 0.0],
                    [0.0,  0.0, 0.0],
                    [0.0,  0.0, 2.0]
                    ])
