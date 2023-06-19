from params import *
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from extended_kalman_filter import *

def f(x):
    """
    This is the non-linear true state
    transtion function: f:R^3 -> R^3
    f(x) = f(x0,x1,x2)
         =  (x0+1, sin(x0), cos(x0))
         =  (f1(x),  f2(x), f3(x))
    """
    new_counter = x[0]+1
    new_pos     = np.sin(new_counter)
    new_speed   = np.cos(new_counter)
    new_x = np.array([new_counter,
                      new_pos,
                      new_speed])
    return new_x

def Jacobi_f(x):
    x0,x1,x2 = x[0],x[1],x[2]
    Jf = np.array([[1,0,0],
                   [np.cos(x0),0,0],
                   [-np.sin(x0),0,0]])
    return Jf
    
def h(x):
    """
    This is the non-linear true measurement function
    h(x) = h(x0,x1,x2) = (x0^2, x1^2, x2^2)
    """
    return x**2

def Jacobi_h(x):
    Jh = np.array([[2*x[0],      0, 0],
                   [0     , 2*x[1], 0],
                   [0     ,      0, 2*x[2]]
                  ])
    return Jh



simu_time = 0.0

initial_x = np.array( [0,0,0] )
initial_P = np.array( [[0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0],
                       [0.0, 0.0, 0.0]])

true_x = initial_x.copy()

true_xs = []
zs      = []
kf_xs   = []
state_uncertainties = [] 

kf = extended_kalman_filter(initial_x=initial_x,
                            initial_P=initial_P,
                            Q=true_Q,
                            R=true_R,
                            est_f = f,
                            est_h = h,
                            Jacobi_f = Jacobi_f,
                            Jacobi_h = Jacobi_h)

while simu_time < max_simu_time:

    # simulate state transition with process noise
    true_x = f(true_x)
    process_noise = np.random.multivariate_normal(
                        np.array([0,0,0]), true_Q)
    true_x += process_noise

    # simulate noisy measurements
    measurement_noise = np.random.multivariate_normal(
                            np.array([0,0,0]), true_R)
    z = h(true_x) + measurement_noise

    kf.predict()
    if simu_time != 0 and simu_time % measurement_each_nth_step == 0:
        kf.correct_prediction_using_measurement(z)

    kf_xs.append( kf.x ) 

    true_xs.append( true_x )
    zs.append( z )
    print(f"simu_time: {simu_time}:\n{kf.P}")
    state_uncertainties.append( kf.get_scalar_measure_of_uncertainty_about_state() ) 
    simu_time += dt

plt.figure( figsize=(15,10) )


plt.subplot(5,1, 1)
plt.title("State 1")
plt.xlabel("time [s]")
plt.plot( [x[1] for x in true_xs], "red",   label="true" )
plt.plot( [z[1] for z in zs],      "green", label="measured" )
plt.plot( [x[1] for x in kf_xs],   "blue",  label="EKF estimate" )
plt.legend()

plt.subplot(5,1, 2)
plt.title("State 2")
plt.xlabel("time [s]")
plt.plot( [x[2] for x in true_xs], "red",   label="true" )
plt.plot( [z[2] for z in zs],      "green", label="measured" )
plt.plot( [x[2] for x in kf_xs],   "blue",  label="EKF estimate" )
plt.legend()

# plot error of measurement, KF for position
D = 2
z_errors = [x[D]-z[D] for x,z in zip(true_xs, zs)]
ekf_errors   = [x[D]-x_kf[D] for x,x_kf in zip(true_xs, kf_xs)]
MAE_z   = np.mean(np.abs(z_errors))
MAE_ekf = np.mean(np.abs(ekf_errors))
plt.subplot(5,1, 3)
plt.title(f"Error for state {D}")
plt.xlabel("time [s]")
plt.plot( z_errors, "green", label=f"measurement error (MAE={MAE_z:.2f})" )
plt.plot( ekf_errors,   "blue",  label=f"KF estimate error (MAE={MAE_ekf:.2f})"   )
plt.legend()

# plot uncertainties about estimated states over time
plt.subplot(5,1, 5)
plt.title("State uncertainty")
plt.xlabel("time [s]")
plt.ylabel("state uncertainty")
plt.plot(state_uncertainties, "black")

plt.subplots_adjust(hspace=1)

plt.show()

print("Simulation finished!")