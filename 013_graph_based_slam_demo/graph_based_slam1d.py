# Original source code snippet is from
# https://atsushisakai.github.io/PythonRobotics/modules/slam/graph_slam/graph_slam.html
#
# Augmented with more comments
# and adapted here by
# Prof. Dr. Jürgen Brauer, www.juergenbrauer.org
#
# The demo shows a very simple 1D robot.
# 1D means, that it can only move to the left or right.


import numpy as np
np.set_printoptions(precision=5, suppress=True)
np.random.seed(0)

import matplotlib.pyplot as plt
import copy
import math
import itertools


# we only simulate 3 steps
N = 3

# there is a single landmark at location 3
landmark = 3

# simulated true locations at time step 0,1,2
# simulated odometry readings at time step 0,1,2
# simulated observations of landmark at time time 0,1,2
odom = np.empty((N,1))
obs = np.empty((N,1))
x_true = np.empty((N,1))
x_true[0], odom[0], obs[0] = 0.0, 0.0, 2.9
x_true[1], odom[1], obs[1] = 1.0, 1.5, 2.0
x_true[2], odom[2], obs[2] = 2.0, 2.4, 1.0


R = 0.2
Q = 0.2
graphics_radius = 0.1

hxDR = copy.deepcopy(odom)

# plot the landmark
plt.plot(landmark,0, '*k', markersize=30)
# for all three time steps 0,1,2 ...
for i in range(N):

    # ... plot the assumed location of the robot according to odometry
    plt.plot(odom[i], 0, '.', markersize=50, alpha=0.8, color='steelblue')
    #plt.plot([odom[i], odom[i] + graphics_radius], [0,0], 'r')             
    plt.text(odom[i], 0.02, f"odom{i}", fontsize=10, rotation=90)

    # ... plot the observed (noisy) location of the single landmark
    # at that time step    
    plt.plot(odom[i]+obs[i], 0,'.', markersize=25, color='brown')
    plt.text(odom[i]+obs[i], 0.04, f"m{i}", fontsize=10, rotation=90)

    # ... plot the true location of the robot in green
    plt.plot(x_true[i], 0,'.g', markersize=20)
    plt.text(x_true[i], -0.02, f"true{i}", fontsize=10, rotation=90)

plt.title("\n\nA 1D robot example\n"
          "green: true locations\n"
          "blue: locations due to odometry\n"
          "brown: observed landmark locations")
plt.grid()
plt.show()


# Defined as a function to facilitate iteration
def get_H_b(odom, obs):
    """
    Create the information matrix and information vector. This implementation is
    based on the concept of virtual measurement i.e. the observations of the
    landmarks are converted into constraints (edges) between the nodes that
    have observed this landmark.
    """
    measure_constraints = {}
    omegas = {}
    zids = list(itertools.combinations(range(N),2))
    print( f"zids={zids}" )

    # prepare empty information matrix
    H = np.zeros((N,N))

    # prepare empty information vector
    b = np.zeros((N,1))

    # for all pose pairs at time points (t1,t2)...
    for (t1, t2) in zids:

        # get location at time t1 due to odometry
        x1 = odom[t1]
        # get location at time t2 due to odometry
        x2 = odom[t2]
        # get landmark location observed at time t1
        z1 = obs[t1]
        # get landmark location observed at time t2
        z2 = obs[t2]

        # adding virtual measurement constraint

        # landmark should be due to time step t1 at x1+z1
        # landmark should be due to time step t2 at x2+z2
        # so (x2+z2)-(x1+z1)=x2-x1-z1+z2 should be zero
        # to minimize this difference, we will compute an offset
        # dx=H^{-1} @ b
        # note: @ is equivalent to np.matmul()
        measure_constraints[(t1,t2)] = (x2-x1-z1+z2)
        omegas[(t1,t2)] = (1 / (2*Q))

        # populate system's information matrix and vector
        H[t1,t1] += omegas[(t1,t2)]
        H[t2,t2] += omegas[(t1,t2)]
        H[t2,t1] -= omegas[(t1,t2)]
        H[t1,t2] -= omegas[(t1,t2)]

        b[t1] += omegas[(t1,t2)] * measure_constraints[(t1,t2)]
        b[t2] -= omegas[(t1,t2)] * measure_constraints[(t1,t2)]

    return H, b


print("Running graphSLAM ...")
H, b = get_H_b(odom, obs)
print("The determinant of H: ", np.linalg.det(H))
H[0,0] += 1 # np.inf ?
print("The determinant of H after anchoring constraint: ", np.linalg.det(H))

for i in range(5):
    H, b = get_H_b(odom, obs)
    H[(0,0)] += 1

    # Recover the posterior over the path
    dx = np.linalg.inv(H) @ b

    # note: dx is a 3D correction vector (3 time steps)
    print( f"dx={dx}" )

    # add correction vector to current estimates of robot's pose
    # note: odom is already a 3D vector (3 time steps)
    odom += dx
    # repeat till convergence

print("Odometry values after optimzation: \n", odom)

plt.figure()
plt.plot(x_true, np.zeros(x_true.shape), '.', markersize=20, label='Ground truth', color="green")
plt.plot(odom, np.zeros(x_true.shape), '.', markersize=20, label='Optimized odometry', color="red")
plt.plot(hxDR, np.zeros(x_true.shape), '.', markersize=20, label='Odom', color='steelblue')
plt.legend()
plt.grid()
plt.show()