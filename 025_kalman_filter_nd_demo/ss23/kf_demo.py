from params import *
import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import *


simu_time = 0.0

initial_x = np.array( [start_pos, start_speed] )
initial_P = np.array( [[10.0, 0.0],
                       [0.0, 1.0]])

true_x = initial_x.copy()

true_xs = []
zs      = []
kf_xs   = []

kf = kalman_filter(initial_x=initial_x,
                   initial_P=initial_P,
                   F=true_F,
                   B=true_B,
                   Q=true_Q,
                   H=true_H,
                   R=true_R)

while simu_time < max_simu_time:

    # simulate state transition with process noise
    true_x = true_F @ true_x + true_B @ u
    process_noise = np.random.multivariate_normal(
                        np.array([0,0]), true_Q)
    true_x += process_noise

    # simulate noisy measurements
    measurement_noise = np.random.multivariate_normal(
                            np.array([0,0]), true_R)
    z = true_H @ true_x + measurement_noise

    kf.predict(u)
    kf.correct_prediction_using_measurement(z)

    kf_xs.append( kf.x ) 

    true_xs.append( true_x )
    zs.append( z )
    simu_time += dt

plt.figure( figsize=(15,10) )

# plot true, measured position of car
plt.subplot(4,1, 1)
plt.title("Position")
plt.xlabel("time [s]")
plt.ylabel("position [m]")
plt.plot( [x[0] for x in true_xs], "red",   label="true" )
plt.plot( [z[0] for z in zs],      "green", label="measured" )
plt.plot( [x[0] for x in kf_xs],   "blue",  label="KF estimate" )
plt.legend()

# plot true, measured speed of car
plt.subplot(4,1, 2)
plt.title("Speed")
plt.xlabel("time [s]")
plt.ylabel("speed [m/s]")
plt.plot( [x[1] for x in true_xs], "red",   label="true" )
plt.plot( [z[1] for z in zs],      "green", label="measured" )
plt.plot( [x[1] for x in kf_xs],   "blue",  label="KF estimate" )
plt.legend()

# plot error of measurement, KF for position
meas_pos_errors = [x[0]-z[0] for x,z in zip(true_xs, zs)]
kf_pos_errors   = [x[0]-x_kf[0] for x,x_kf in zip(true_xs, kf_xs)]
MAE_meas_pos = np.mean(np.abs(meas_pos_errors))
MAE_kf_pos   = np.mean(np.abs(kf_pos_errors))
plt.subplot(4,1, 3)
plt.title("Position Error")
plt.xlabel("time [s]")
plt.ylabel("position error [m]")
plt.plot( meas_pos_errors, "green", label=f"measurement error (MAE={MAE_meas_pos:.2f})" )
plt.plot( kf_pos_errors,   "blue",  label=f"KF estimate error (MAE={MAE_kf_pos:.2f})"   )
plt.legend()

# plot error of measurement, KF for speed
meas_speed_errors = [x[1]-z[1] for x,z in zip(true_xs, zs)]
kf_speed_errors   = [x[1]-x_kf[1] for x,x_kf in zip(true_xs, kf_xs)]
MAE_meas_speed = np.mean(np.abs(meas_speed_errors))
MAE_kf_speed   = np.mean(np.abs(kf_speed_errors))
plt.subplot(4,1, 4)
plt.title("Speed Error")
plt.xlabel("time [s]")
plt.ylabel("speed error [m/s]")
plt.plot( meas_speed_errors, "green", label=f"measurement error (MAE={MAE_meas_speed:.2f})" )
plt.plot( kf_speed_errors,   "blue",  label=f"KF estimate error (MAE={MAE_kf_speed:.2f})"   )
plt.legend()

plt.subplots_adjust(hspace=1)

plt.show()

print("Simulation finished!")