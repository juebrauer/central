import numpy as np

# the state x in this demo is 3D:
# [counter, pos, speed]    
# define:
# - start counter and
# - start position and
# - start speed
start_counter = 0  
start_pos     = 0
start_speed   = 13.8 # [m/s] = ca. 50 km/h



# for how long do we want to simulate?
max_simu_time = 60 # 1minute
#max_simu_time = 900.0 # 0.25h
#max_simu_time = 3600.0 # 1h


# simulation time increment for one simulation loop
dt = 1.0


# how frequently do we correct the state estimate
# using new measurements?
measurement_each_nth_step = 5

# set process noise covariance matrix
true_Q = np.array( [[0.01, 0.0,  0.0],
                    [0.0,  0.1,  0.0],
                    [0.0,  0.0,  1.0]
                    ])


# set true measurement matrix
true_H = np.array( [[1.0, 0.0],
                    [0.0, 1.0]
                    ]
                    )

# set measurement noise covariance matrix
true_R = np.array( [[0.01, 0.0, 0.0],
                    [0.0,  2.0, 0.0],
                    [0.0,  0.0, 2.0]
                    ])

