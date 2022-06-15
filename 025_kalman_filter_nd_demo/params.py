import numpy as np

# the state x in this demo is 2D:
# [pos, speed]    
# define:
# - start position and
# - start speed    
start_pos   = 0
start_speed = 13.8 # [m/s] = ca. 50 km/h



# for how long do we want to simulate?
max_simu_time = 60 # 1minute
#max_simu_time = 900.0 # 0.25h
#max_simu_time = 3600.0 # 1h


# simulation time increment for one simulation loop
dt = 1.0


# how frequently do we correct the state estimate
# using new measurements?
measurement_each_nth_step = 5


# ---

# prepare matrices for simulation:
#    - state transition matrix and transition noise covariance matrix
#    - control matrix
#    - measurement matrix and measurement noise covariance matrix
# set true state transition matrix
# [new_pos, new_speed] = [pos+dt*speed, speed]
true_F = np.array( [[1.0, dt],
                    [0.0, 1.0]
                    ])

# set true control matrix
# [pos_change, speed_change] = [0*acceleration, dt*acceleration] = [0*u,dt*u]
u = np.array( [0.003858025] ) # [m/s^2]
# e.g. car accelerates from 100km/h to 150km/h in one hour:
# a [m/s^2] = delta_speed / delta_time = 50000m / (3600^2)s^2 = 0.003858025 m/s^2
true_B = np.array( [[0.0],
                    [dt]
                    ])
# B is from R^(2,1)
# u is from R^(1,1)
# so Bu is from R^(2,1)   

# set process noise covariance matrix
true_Q = np.array( [[1.0, 0.0],
                    [0.0, 0.01],
                    ])


# set true measurement matrix
true_H = np.array( [[1.0, 0.0],
                    [0.0, 1.0]
                    ]
                    )

# set measurement noise covariance matrix
true_R = np.array( [[100000.0, 0.0],
                    [0.0,      1.0],
                    ])

