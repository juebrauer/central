"""
EKF (Extended Kalman Filter) SLAM Demo

This demo code is from
https://atsushisakai.github.io/PythonRobotics/modules/slam/ekf_slam/ekf_slam.html

I mainly augmented it with further comments.
"""

import math
import matplotlib.pyplot as plt
import numpy as np

SHOW_EST_STATE_VECTOR = False
SHOW_WHICH_LANDMARKS_ARE_CURRENTLY_DETECTED = True

# EKF state covariance
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2

# simulation parameter: process and measurement noise covariance matrices
Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2
print( f"Q_sim={Q_sim}" )
print( f"R_sim={R_sim}" )

# simulation time tick [s]
DT = 0.1

# total time to simulate [s]
SIM_TIME = 150.0

# maximum observation range for landmarks
# try 20, 15
MAX_RANGE = 17.5 

# threshold of Mahalanobis distance for data association.
M_DIST_TH = 2.0  

# state size for the robot is 3: [x,y,yaw]
STATE_SIZE = 3  

# state size for landmarks is 2: [x,y]
LM_SIZE = 2  

show_animation = True


def ekf_slam(xEst, PEst, u, z):

    # 1st step: prediction step
    S = STATE_SIZE
    G, Fx = jacob_motion(xEst[0:S], u)
    xEst[0:S] = motion_model(xEst[0:S], u)
    PEst[0:S, 0:S] = G.T @ PEst[0:S, 0:S] @ G + Fx.T @ Cx @ Fx
    initP = np.eye(2)


    # 2nd step: measurement correction step
    # for each observed landmark ...
    for iz in range(len(z[:, 0])):
        
        min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])

        nLM = calc_n_lm(xEst)

        # did we observe a new landmark?
        if min_id == nLM:
            
            # yes! we have observed a new landmark!
            if SHOW_EST_STATE_VECTOR:
                print( f"\nNew landmark detected! {nLM} landmarks so far." )
                print( f"State vector before: \n{xEst}" )

            # extend state and covariance matrix
            # given the measurement matrix,
            # we compute the estimated location of this new landmark
            xAug = np.vstack((xEst, calc_landmark_position(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))

            # augmented state matrix and covariance matrix become current matrices
            xEst = xAug
            PEst = PAug
            
            if SHOW_EST_STATE_VECTOR:
                print( f"State vector after: \n{xEst}" )
        
        # get the assumed location of the landmark
        # from the state vector
        lm = get_landmark_position_from_state(xEst, min_id)

        # update state vector estimate
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)
        K = (PEst @ H.T) @ np.linalg.inv(S)
        xEst = xEst + (K @ y)
        PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

    # constrain rotation angle to [0,2pi]
    xEst[2] = pi_2_pi(xEst[2])

    return xEst, PEst


def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(xTrue, xd, u, RFID):

    # compute new true position and orientation of robot
    xTrue = motion_model(xTrue, u)

    # prepare measurement matrix z
    # for each perceived landmark we will store on measurement row vector
    z = np.zeros((0, 3))

    # for all landmarks ...
    for i in range(len(RFID[:, 0])):

        # ... compute actual relative coordinates (dx,dy)
        #     between robot and this landmark
        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]

        # ... compute distance and relative orientation to this landmark
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])

        # is this landmark too far away to be perceptible?
        if d <= MAX_RANGE:

            # no! its within the range!

            # simulate measurement noise regarding perceived relative distance
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5

            # and 
            # simulate measurement noise regarding perceived relative orientation
            angle_n = angle + np.random.randn() * Q_sim[1, 1] ** 0.5

            # that's the new measurement vector: [relative distance, relative angle, landmark ID]
            zi = np.array([dn, angle_n, i])

            # for each landmark we store one measurement vector in z
            z = np.vstack((z, zi))

    # simulate dead reckoning state and control vector
    ud = np.array([[
        u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5,
        u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5]]).T
    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    return x


def calc_n_lm(x):
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_lm(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0, 0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)

    G = np.eye(STATE_SIZE) + Fx.T @ jF @ Fx

    return G, Fx,


def calc_landmark_position(x, z):
    """
    Given the current state vector x,
    especially the location estimate (x[0,0], x[1,0]),
    compute the landmark location (zp[0,0], z[1,0]),
    given the measured relative landmark location
    z[0]: measured distance to the landmark
    z[1]: measured relative angle to the landmark
    """
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

    return zp


def get_landmark_position_from_state(x, ind):
    """
    Given the state vector x,
    retrieve the <ind>-th landmark location from the state vector x
    """
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_landmark_id(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """

    nLM = calc_n_lm(xAug)

    min_dist = []

    for i in range(nLM):
        lm = get_landmark_position_from_state(xAug, i)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        min_dist.append(y.T @ np.linalg.inv(S) @ y)

    min_dist.append(M_DIST_TH)  # new landmark

    min_id = min_dist.index(min(min_dist))

    return min_id


def calc_innovation(lm, xEst, PEst, z, LMid):
    delta = lm - xEst[0:2]
    q = (delta.T @ delta)[0, 0]
    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
    y = (z - zp).T
    y[1] = pi_2_pi(y[1])
    H = jacob_h(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + Cx[0:2, 0:2]

    return y, S, H


def jacob_h(q, delta, x, i):
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = G @ F

    return H


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():    
    
    time = 0.0

    # define landmark (RFID) positions [x, y]
    RFID = np.array([[10.0, -2.0],
                     [15.0, 10.0],
                     [3.0, 15.0],
                     [-5.0, 20.0]])

    # prepare NumPy arrays to store ...
    
    # ... EKF estimated state vector [x y yaw]'    
    xEst = np.zeros((STATE_SIZE, 1))

    # ... true state vector [x y yaw]'
    xTrue = np.zeros((STATE_SIZE, 1))

    # ... estimated uncertainty about the estimated state
    PEst = np.eye(STATE_SIZE)

    # ... Dead Reckoning estimated state
    xDR = np.zeros((STATE_SIZE, 1))

    # history of EKF estimated, true and DR estimated states
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    # continue simulation?
    while SIM_TIME >= time:

        # time goes by
        time += DT

        # get control signal [speed, yaw rate]
        u = calc_input()

        # get
        # XTrue: true robot pose [x,y,yaw]
        # z: matrix of noisy landmark measurements
        # xDR: dead-reckoning estimated robot pose [x,y,yaw]
        # ud: control vector with noise
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        if SHOW_WHICH_LANDMARKS_ARE_CURRENTLY_DETECTED :
            print( f"Measurement matrix={z[:,2]}" )


        # EKF SLAM gives us two estimates:
        # - an estimated robot pose xEst
        # - an estimation about the uncertainy of this estimate
        xEst, PEst = ekf_slam(xEst, PEst, ud, z)

        x_state = xEst[0:STATE_SIZE]

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            # show all true landmarks locations as black stars
            plt.plot(RFID[:, 0], RFID[:, 1], "*k")

            # show all EKF estimated locations as red dots
            plt.plot(xEst[0], xEst[1], ".r")

            # plot estimated landmark locations as "x"
            for i in range(calc_n_lm(xEst)):
                plt.plot(xEst[STATE_SIZE + i * 2],
                         xEst[STATE_SIZE + i * 2 + 1], "xg")

            # plot true robot locations in blue
            plt.plot(hxTrue[0, :],
                     hxTrue[1, :], "-b")

            # plot dead-reckoning estimated robot locations in black
            plt.plot(hxDR[0, :],
                     hxDR[1, :], "-k")

            # plot EKF estimated robot locations in red
            plt.plot(hxEst[0, :],
                     hxEst[1, :], "-r")

            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


main()
