import numpy as np
import matplotlib.pyplot as plt
import params as P
import kalmanfilter

simu_time = 0.0

true_x = np.array( [P.start_pos, P.start_speed] )

true_xs = []
zs = []
est_xs = []

initial_P = np.array( [[10.0, 0.0],
                       [ 0.0, 1.0]])
kf = kalmanfilter.kalman_filter( initial_x = np.array( [P.start_pos, P.start_speed] ),
                                 initial_P = initial_P,
                                 F = P.true_F,
                                 B = P.true_B,
                                 Q = P.true_Q,
                                 H = P.true_H,
                                 R = P.true_R)
               
while simu_time < P.max_simu_time:

    process_noise = np.random.multivariate_normal( np.array([0,0]), P.true_Q )
    true_x = P.true_F @ true_x + P.true_B @ P.u + process_noise
    true_xs.append( true_x )

    kf.predict( P.u )

    measurement_noise = np.random.multivariate_normal( np.array([0,0]), P.true_R)
    z = P.true_H @ true_x + measurement_noise
    zs.append( z )

    kf.correct_prediction_using_measurement( z )
    est_xs.append( kf.x )

    simu_time += P.dt



plt.figure(figsize=(15,10))

plt.subplot(4,1, 1)
plt.title("Position")
plt.xlabel("time [s]")
plt.ylabel("position [m]")
plt.plot( [x[0] for x in true_xs], label="true position", color="black" )
plt.plot( [z[0] for z in zs], label="measured position", color="red")
plt.plot( [est_x[0] for est_x in est_xs], label="KF estimated position", color="blue")
plt.legend()

plt.subplot(4,1, 2)
plt.title("Speed")
plt.xlabel("time [s]")
plt.ylabel("speed [m/s]")
plt.plot( [x[1] for x in true_xs], label="true speed", color="black" )
plt.plot( [z[1] for z in zs], label="measured speed", color="red")
plt.plot( [est_x[1] for est_x in est_xs], label="KF estimated speed", color="blue")
plt.legend()

plt.subplot(4,1, 3)
kf_pos_errors   = [est_x[0]-x[0] for x, est_x in zip(true_xs, est_xs)]
meas_pos_errors = [z[0]    -x[0] for x, z     in zip(true_xs, zs)]
plt.title("Position Errors per time step")
plt.xlabel("time [s]")
plt.ylabel("error [m]")
plt.plot( kf_pos_errors,   color="blue", label="KF pos error" )
plt.plot( meas_pos_errors, color="red",  label="Measurement pos error" )
plt.legend()

plt.subplot(4,1, 4)
kf_speed_errors   = [est_x[1]-x[1] for x, est_x in zip(true_xs, est_xs)]
meas_speed_errors = [z[1]    -x[1] for x, z     in zip(true_xs, zs)]
plt.title("Speed Errors per time step")
plt.xlabel("time [s]")
plt.ylabel("error [m/s]")
plt.plot( kf_speed_errors,   color="blue", label="KF speed error" )
plt.plot( meas_speed_errors, color="red",  label="Measurement speed error" )
plt.legend()


plt.subplots_adjust(hspace=3)

plt.show()


