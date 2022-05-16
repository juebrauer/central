# Original source code snippet is from
# https://atsushisakai.github.io/PythonRobotics/modules/slam/graph_slam/graph_slam.html
#
# Augmented with more comments
# and adapted here by
# Prof. Dr. Jürgen Brauer, www.juergenbrauer.org
#
# The demo shows a very simple robot that drives
# in a circle.


import copy
import itertools
import math

import matplotlib.pyplot as plt
import numpy as np
np.random.seed(42)
from scipy.spatial.transform import Rotation as Rot

# measurement noise matrix
Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2

# process noise matrix
R_sim = np.diag([0.1, np.deg2rad(10.0)]) ** 2

DT = 2.0  # time tick [s]
SIM_TIME = 120.0  # simulation time [s]
MAX_RANGE = 30.0  # maximum observation range
STATE_SIZE = 3  # State size [x,y,yaw]

# Covariance parameter of Graph Based SLAM
C_SIGMA1 = 0.1
C_SIGMA2 = 0.1
C_SIGMA3 = np.deg2rad(1.0)

MAX_ITR = 20  # Maximum iteration for graph optimization

show_graph_d_time = 20.0  # [s]
show_animation = True


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


def cal_observation_sigma():
    sigma = np.zeros((3, 3))
    sigma[0, 0] = C_SIGMA1 ** 2
    sigma[1, 1] = C_SIGMA2 ** 2
    sigma[2, 2] = C_SIGMA3 ** 2

    return sigma


def calc_rotational_matrix(angle):
    return Rot.from_euler('z', angle).as_matrix()


def calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
              angle1, d2, angle2, t1, t2):
    """
    Generates a single edge in the pose graph
    """
    edge = Edge()

    tangle1 = pi_2_pi(yaw1 + angle1)
    tangle2 = pi_2_pi(yaw2 + angle2)
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    # the error has 3 components:

    # x2+tmp2 - (x1+tmp1) should be zero!
    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2

    # y2+tmp3 - (y1+tmp4) should be zero!
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4

    # why is this set to zero?
    edge.e[2, 0] = 0

    Rt1 = calc_rotational_matrix(tangle1)
    Rt2 = calc_rotational_matrix(tangle2)

    sig1 = cal_observation_sigma()
    sig2 = cal_observation_sigma()

    edge.omega = np.linalg.inv(Rt1 @ sig1 @ Rt1.T + Rt2 @ sig2 @ Rt2.T)

    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    return edge


def calc_edges(x_list, z_list):
    edges = []
    cost = 0.0
    z_ids = list(itertools.combinations(range(len(z_list)), 2))

    # for all available node combinations ...
    for (t1, t2) in z_ids:

        # get node t1 (estimated robot pose at time t1)
        x1, y1, yaw1 = x_list[0, t1], x_list[1, t1], x_list[2, t1]

        # get node t2 (estimated robot pose at time t2)
        x2, y2, yaw2 = x_list[0, t2], x_list[1, t2], x_list[2, t2]

        # do both nodes have observations stored?
        if z_list[t1] is None or z_list[t2] is None:
            continue  # No observation

        # for all observations of node 1 ...
        for iz1 in range(len(z_list[t1][:, 0])):

            # for all observations of node 2 ...
            for iz2 in range(len(z_list[t2][:, 0])):

                # did both nodes observe the same landmark?
                # the landmark id is stored for each node 
                # in column 4 (index 3):
                if z_list[t1][iz1, 3] == z_list[t2][iz2, 3]:

                    # yes, both the nodes have observed the same landmark!

                    # get observed distance and angle to the landmark from node 1
                    d1 = z_list[t1][iz1, 0]
                    angle1, phi1 = z_list[t1][iz1, 1], z_list[t1][iz1, 2]

                    # get observed distance and angle to the landmark from node 2
                    d2 = z_list[t2][iz2, 0]
                    angle2, phi2 = z_list[t2][iz2, 1], z_list[t2][iz2, 2]

                    # generate a single edge
                    edge = calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
                                     angle1, d2, angle2, t1, t2)

                    # store the edge
                    edges.append(edge)
                    cost += (edge.e.T @ edge.omega @ edge.e)[0, 0]

    print("cost:", cost, ",n_edge:", len(edges))

    # return the pose graph
    return edges


def calc_jacobian(edge):
    t1 = edge.yaw1 + edge.angle1
    A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                  [0, -1.0, -edge.d1 * math.cos(t1)],
                  [0, 0, 0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                  [0, 1.0, edge.d2 * math.cos(t2)],
                  [0, 0, 0]])

    return A, B


def fill_H_and_b(H, b, edge):
    A, B = calc_jacobian(edge)

    id1 = edge.id1 * STATE_SIZE
    id2 = edge.id2 * STATE_SIZE

    H[id1:id1 + STATE_SIZE, id1:id1 + STATE_SIZE] += A.T @ edge.omega @ A
    H[id1:id1 + STATE_SIZE, id2:id2 + STATE_SIZE] += A.T @ edge.omega @ B
    H[id2:id2 + STATE_SIZE, id1:id1 + STATE_SIZE] += B.T @ edge.omega @ A
    H[id2:id2 + STATE_SIZE, id2:id2 + STATE_SIZE] += B.T @ edge.omega @ B

    b[id1:id1 + STATE_SIZE] += (A.T @ edge.omega @ edge.e)
    b[id2:id2 + STATE_SIZE] += (B.T @ edge.omega @ edge.e)

    return H, b


def graph_based_slam(x_init, hz):

    print("\n\nstart graph based slam")

    # make a copy of all measurements so ar
    z_list = copy.deepcopy(hz)

    # make a copy of the start estimates
    # for the robot poses
    x_opt = copy.deepcopy(x_init)
    print("xopt=",x_opt)
    
    # how many states are there?
    nt = x_opt.shape[1]

    # nr of nodes * 3
    n = nt * STATE_SIZE

    for itr in range(MAX_ITR):

        # graph-construction: generate edges
        edges = calc_edges(x_opt, z_list)

        # compute information matrix and
        # information vector
        H = np.zeros((n, n))
        b = np.zeros((n, 1))
        for edge in edges:
            H, b = fill_H_and_b(H, b, edge)

        # to fix origin
        H[0:STATE_SIZE, 0:STATE_SIZE] += np.identity(STATE_SIZE)

        # optimization happens here!
        dx = - np.linalg.inv(H) @ b
        for i in range(nt):
            x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]

        # convergence?
        diff = dx.T @ dx
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if diff < 1.0e-5:
            break

    # return the optimized states
    return x_opt


def calc_input():
    v = 2.0  # [m/s]
    yaw_rate = 0.2  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(xTrue, xd, u, RFID):

    # compute next true pose
    xTrue = motion_model(xTrue, u)

    # start with an empty measurement vector
    z = np.zeros((0, 4))

    # for all landmarks to simulate
    for i in range(len(RFID[:, 0])):

        # get true distance d from robot to landmark
        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)

        # compute true relative angle to landmark
        angle = pi_2_pi(math.atan2(dy, dx)) - xTrue[2, 0]

        # compute true absolute angle to landmark
        phi = pi_2_pi(math.atan2(dy, dx))

        # do we observe the landmark at all?
        if d <= MAX_RANGE:

            # yes! landmarks is in range

            # simulate a noisy distance measurement
            dn = d + np.random.randn() * Q_sim[0, 0]  # add noise

            # simulate a noisy angle measurement
            angle_noise = np.random.randn() * Q_sim[1, 1]
            angle += angle_noise
            phi += angle_noise

            # pack all into a single measurement vector z_i
            zi = np.array([dn, angle, phi, i])

            # pack this new measurement vector into overall measurement marix z
            z = np.vstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1]
    ud = np.array([[ud1, ud2]]).T

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



def show_trajectories(time, RFID, hxTrue, hxDR, x_opt):

    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    # plot all landmarks in black
    plt.plot(RFID[:, 0], RFID[:, 1], "*k")

    # plot true robot locations in blue
    plt.plot(hxTrue[0, :].flatten(),
                hxTrue[1, :].flatten(), "-b")

    # plot locations estimated by dead reckoning in black
    plt.plot(hxDR[0, :].flatten(),
                hxDR[1, :].flatten(), "-k")

    # plot graph based SLAM optimized locations in red
    plt.plot(x_opt[0, :].flatten(),
                x_opt[1, :].flatten(), "-r")

    plt.axis("equal")
    plt.grid(True)
    plt.title("Time" + str(time)[0:5])
    

def main():

    time = 0.0

    # landmark locations [x, y, yaw]
    RFID = np.array([[10.0, -2.0, 0.0],
                     [15.0, 10.0, 0.0],
                     [3.0, 15.0, 0.0]
                    ])
    """                
    RFID = np.array([[10.0, -2.0, 0.0],
                     [15.0, 10.0, 0.0],
                     [3.0, 15.0, 0.0],
                     [8.,8.,8.]
                     ])
    """

    # state vector is 3-dimensional [x y yaw]

    # here we store the true state
    xTrue = np.zeros((STATE_SIZE, 1))

    # here we store the state computed by dead reckoning
    xDR = np.zeros((STATE_SIZE, 1))

    # history list of all true states
    hxTrue = []

    # history list of all dead reckoning computed states
    hxDR = []

    # history of all measurement vectors
    hz = []

    d_time = 0.0
    init = False

    # continue with simulation?
    while SIM_TIME >= time:

        if not init:
            hxTrue = xTrue
            hxDR = xTrue
            init = True
        else:
            # augment history Numpy arrays of
            # true and dead reckoning computed states
            hxTrue = np.hstack((hxTrue, xTrue))
            hxDR = np.hstack((hxDR, xDR))
            
        # absolute time goes by
        time += DT

        # time since least visualization and optimization update
        d_time += DT

        # simulate control vector u
        u = calc_input()

        # simulate a single measurement
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        # store new measurement in history of all measurements
        hz.append(z)

        # from time to time 
        if d_time >= show_graph_d_time:

            # optimize the nodes
            x_opt = graph_based_slam(hxDR, hz)

            # reset optimization timer
            d_time = 0.0

            if show_animation:

                show_trajectories(time, RFID, hxTrue, hxDR, x_opt)
                plt.pause(1.0)

        # end-if show trajectories from time to time

    # end-while simulation time is not over

    # show final trajectories
    print("\nSimulation finished!")
    show_trajectories(time, RFID, hxTrue, hxDR, x_opt)
    plt.show()

main()
