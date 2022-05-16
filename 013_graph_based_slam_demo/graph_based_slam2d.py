# Original source code snippet is from
# https://atsushisakai.github.io/PythonRobotics/modules/slam/graph_slam/graph_slam.html
#
# Augmented with more comments
# and adapted here by
# Prof. Dr. Jürgen Brauer, www.juergenbrauer.org
#
# The demo shows a very simple 2D robot.
# Note: I think this demo does not work.


import numpy as np
np.set_printoptions(precision=5, suppress=True)
np.random.seed(0)

import matplotlib.pyplot as plt
import copy
import math
import itertools


def calc_input():
    """
    Defines speed and yaw_rate
    and returns both as a control vectur u
    """
    v = 2.0  # [m/s]
    yaw_rate = 0.2  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(xTrue, xd, u, RFID):
    """
    Generates a new true state xTrue,
    a simulated noisy measurement z
    """
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 4))

    # for all landmarks ...
    for i in range(len(RFID[:, 0])):

        # what is the true distance to the landmark?
        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)

        # what is the true relative and absolute angle to the landmark?
        angle = pi_2_pi(math.atan2(dy, dx)) - xTrue[2, 0]        
        phi = pi_2_pi(math.atan2(dy, dx))

        # do we observe the landmark at all?
        if d <= MAX_RANGE:

            # simulate a noisy distance measurement
            dn = d + np.random.randn() * Qsim[0, 0]  # add noise

            # simulate a noisey angle measurement
            angle_noise = np.random.randn() * Qsim[1, 1]
            angle += angle_noise
            phi += angle_noise

            # pack everything in one measurement vector zi
            zi = np.array([dn, angle, phi, i])
            z = np.vstack((z, zi))

    # add noise to control vector u
    ud1 = u[0, 0] + np.random.randn() * Rsim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * Rsim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    # compute dead reckoning estimated state xd
    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = F @ x + B @ u

    return x


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi



######################################
# Step 1: Simulation of robot movement
#         and measurement vectors
######################################

#  Simulation parameter
Qsim = np.diag([0.02, np.deg2rad(0.030)])**2 # error added to range and bearing
Rsim = np.diag([0.5, np.deg2rad(2.0)])**2 # error added to [v, w]

DT = 2.0  # time tick [s]
SIM_TIME = 100.0  # simulation time [s]
MAX_RANGE = 100.0  # maximum observation range

# TODO: Why not use Qsim ?
# Covariance parameter of Graph Based SLAM
C_SIGMA1 = 0.1
C_SIGMA2 = 0.1
C_SIGMA3 = np.deg2rad(1.0)

MAX_ITR = 20  # Maximum iteration during optimization

# consider only 2 landmarks for simplicity
# RFID positions [x, y, yaw]
RFID = np.array([[10.0, -2.0, 0.0],
                 [15.0, 10.0, 0.0],
#                  [3.0, 15.0, 0.0],
#                  [-5.0, 20.0, 0.0],
#                  [-5.0, 5.0, 0.0]
                 ])

# state vector: [x,y,yaw]
STATE_SIZE = 3  

# here we store the true states
xTrue = np.zeros((STATE_SIZE, 1))

# here we store the states estimated by dead reckoning
xDR = np.zeros((STATE_SIZE, 1))

# initialize rotation
xTrue[2] = np.deg2rad(45)
xDR[2] = np.deg2rad(45)

# history initial values
hxTrue = xTrue
hxDR = xTrue

# get a first observation given control vector u=[0,0]
_, z, _, _ = observation(xTrue, xDR, np.array([[0,0]]).T, RFID)

# we store all observation vectors in a list
hz = [z]

# for all time steps to simulate ...
timesteps = 5
for i in range(timesteps):

    # get conrol vector u
    u = calc_input()

    # compute true state xTrue
    # measurement vector z
    # dead reckoning estimated state xDR
    # and noisy control vector ud
    xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

    # augment history of true states
    hxTrue = np.hstack((hxTrue, xTrue))

    # augment history of DR estimated states
    hxDR = np.hstack((hxDR, xDR))

    # augment history of measurement vectors
    hz.append(z)

# visualize

# show all landmarks
plt.plot(RFID[:, 0], RFID[:, 1], "*k", markersize=20)

# show dead reckoning estimated states
plt.plot(hxDR[0, :], hxDR[1, :], '.', markersize=40, alpha=0.8, label='Odom')

# show true states
plt.plot(hxTrue[0, :], hxTrue[1, :], '.', markersize=20, alpha=0.6, label='X_true')

# plot observed landmarks
graphics_radius = 0.3
for i in range(hxDR.shape[1]):
    x = hxDR[0, i]
    y = hxDR[1, i]
    yaw = hxDR[2, i]
    plt.plot([x, x + graphics_radius * np.cos(yaw)],
             [y, y + graphics_radius * np.sin(yaw)], 'r')
    d = hz[i][:, 0]
    angle = hz[i][:, 1]
    plt.plot([x + d * np.cos(angle + yaw)], [y + d * np.sin(angle + yaw)], '.',
             markersize=20, alpha=0.7)
    plt.legend()
plt.grid()
plt.show()



#######################################
# Step 2: show which node combinations
#         will lead to constraints
#######################################

# copy the data to have a consistent naming with the .py file
zlist = copy.deepcopy(hz)
x_opt = copy.deepcopy(hxDR)
xlist = copy.deepcopy(hxDR)
number_of_nodes = x_opt.shape[1]
n = number_of_nodes * STATE_SIZE


# get all the possible combination of the different node
zids = list(itertools.combinations(range(len(zlist)), 2))
print("\n\nAvailable node combinations: \n", zids)

for i in range(xlist.shape[1]):    
    print("Node {} observed landmark with ID {}".format(i, zlist[i][0, 3]))



############################
# Step 3: graph construction
############################

class Edge:

    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))  # information matrix
        self.d1 = 0.0
        self.d2 = 0.0
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.id1 = 0
        self.id2 = 0


# initialize edges list
edges = []

# go through all available node combinations ...
for (t1, t2) in zids:

    # get infos for node 1
    x1, y1, yaw1 = xlist[0, t1], xlist[1, t1], xlist[2, t1]

    # get infos for node 2
    x2, y2, yaw2 = xlist[0, t2], xlist[1, t2], xlist[2, t2]

    # All nodes have valid observation with ID=0, therefore, no data association condition
    iz1 = 0
    iz2 = 0

    # get distance, relative angle and absolute angle for observation of landmark 0
    # from node 1
    d1 = zlist[t1][iz1, 0]
    angle1, phi1 = zlist[t1][iz1, 1], zlist[t1][iz1, 2]

    # get distance, relative angle and absolute angle for observation of landmark 0
    # from node 2
    d2 = zlist[t2][iz2, 0]
    angle2, phi2 = zlist[t2][iz2, 1], zlist[t2][iz2, 2]

    # find angle between observation and horizontal
    tangle1 = pi_2_pi(yaw1 + angle1)
    tangle2 = pi_2_pi(yaw2 + angle2)

    # project the observations
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    # create a new edge
    edge = Edge()
    #print(y1,y2, tmp3, tmp4)

    # calculate the error of the virtual measurement
    # node 1 as seen from node 2 throught the observations 1,2
    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
    edge.e[2, 0] = pi_2_pi(yaw2 - yaw1 - tangle1 + tangle2)

    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    # we have a new constraint
    edges.append(edge)

    print("For nodes",(t1,t2))
    print("Added edge with errors: \n", edge.e)

    # Visualize measurement projections
    plt.plot(RFID[0, 0], RFID[0, 1], "*k", markersize=20)
    plt.plot([x1,x2],[y1,y2], '.', markersize=50, alpha=0.8)
    plt.plot([x1, x1 + graphics_radius * np.cos(yaw1)],
             [y1, y1 + graphics_radius * np.sin(yaw1)], 'r')
    plt.plot([x2, x2 + graphics_radius * np.cos(yaw2)],
             [y2, y2 + graphics_radius * np.sin(yaw2)], 'r')

    plt.plot([x1,x1+tmp1], [y1,y1], label="obs 1 x")
    plt.plot([x2,x2+tmp2], [y2,y2], label="obs 2 x")
    plt.plot([x1,x1], [y1,y1+tmp3], label="obs 1 y")
    plt.plot([x2,x2], [y2,y2+tmp4], label="obs 2 y")
    plt.plot(x1+tmp1, y1+tmp3, 'o')
    plt.plot(x2+tmp2, y2+tmp4, 'o')
plt.legend()
plt.grid()
plt.show()


from scipy.spatial.transform import Rotation as Rot

def calc_rotational_matrix(angle):
    return Rot.from_euler('z', angle).as_matrix()



# Initialize the system information matrix and information vector
H = np.zeros((n, n))
b = np.zeros((n, 1))
x_opt = copy.deepcopy(hxDR)

for edge in edges:
    id1 = edge.id1 * STATE_SIZE
    id2 = edge.id2 * STATE_SIZE

    t1 = edge.yaw1 + edge.angle1
    A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                  [0, -1.0, -edge.d1 * math.cos(t1)],
                  [0, 0, -1.0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                  [0, 1.0, edge.d2 * math.cos(t2)],
                  [0, 0, 1.0]])

    # TODO: use Qsim instead of sigma
    sigma = np.diag([C_SIGMA1, C_SIGMA2, C_SIGMA3])
    Rt1 = calc_rotational_matrix(tangle1)
    Rt2 = calc_rotational_matrix(tangle2)
    edge.omega = np.linalg.inv(Rt1 @ sigma @ Rt1.T + Rt2 @ sigma @ Rt2.T)

    # Fill in entries in H and b
    H[id1:id1 + STATE_SIZE, id1:id1 + STATE_SIZE] += A.T @ edge.omega @ A
    H[id1:id1 + STATE_SIZE, id2:id2 + STATE_SIZE] += A.T @ edge.omega @ B
    H[id2:id2 + STATE_SIZE, id1:id1 + STATE_SIZE] += B.T @ edge.omega @ A
    H[id2:id2 + STATE_SIZE, id2:id2 + STATE_SIZE] += B.T @ edge.omega @ B

    b[id1:id1 + STATE_SIZE] += (A.T @ edge.omega @ edge.e)
    b[id2:id2 + STATE_SIZE] += (B.T @ edge.omega @ edge.e)


print("The determinant of H: ", np.linalg.det(H))
plt.figure()
plt.subplot(1,2,1)
plt.imshow(H, extent=[0, n, 0, n])
plt.subplot(1,2,2)
plt.imshow(b, extent=[0, 1, 0, n])
plt.show()

# Fix the origin, multiply by large number gives same results but better visualization
H[0:STATE_SIZE, 0:STATE_SIZE] += np.identity(STATE_SIZE)
print("The determinant of H after origin constraint: ", np.linalg.det(H))
plt.figure()
plt.subplot(1,2,1)
plt.imshow(H, extent=[0, n, 0, n])
plt.subplot(1,2,2)
plt.imshow(b, extent=[0, 1, 0, n])
plt.show()


# Find the solution (first iteration)
for iter_nr in range(5):
    dx = - np.linalg.inv(H) @ b
    for i in range(number_of_nodes):
        x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]
    print("dx: \n",dx)
    print("ground truth: \n ",hxTrue)
    print("Odom: \n", hxDR)
    print("SLAM: \n", x_opt)

    # performance will improve with more iterations, nodes and landmarks.
    print("\n")
    print( f"graphSLAM localization error: {np.sum((x_opt - hxTrue) ** 2):.6f}" )
    print( f"Odom localization error: {np.sum((hxDR - hxTrue) ** 2):.6f}" )