"""
A demo for the n-dimensional Extended Kalman-Filter (EKF).

This filter can estimate n-dimensional states.
By fusing predictions and measurements over time,
the EKF can come up with a better state estimate
than just "believing" noisy measurements directly.

by Prof. Dr. Juergen Brauer
www.juergen.brauer.org
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import extended_kalman_filter
from params import *


def main():

    def f(x):
        """
        This is the non-linear true state transition function
        f:R^3 -> R^3

        f(x) = f(x0,x1,x2) = (x0+1, sin(x0), cos(x0))
                           = (f1(x), f2(x), f3(x))
        """
        new_counter = x[0] + 1
        new_pos     = np.sin(new_counter)
        new_speed   = np.cos(new_counter)
        new_x       = np.array( [new_counter, new_pos, new_speed] )
        #print( f"new_x={new_x}" )
        return new_x


    def Jacobi_f(x):
        """
        This is just a linear approximation of f
        using the Jacobi matrix of f
        """
        Jf = np.array([[ 1,            0, 0],
                       [ np.cos(x[0]), 0, 0],
                       [-np.sin(x[0]), 0, 0]
                      ])
        return Jf


    def h(x):
        """
        This is the non-linear true measurement function
        """
        return x**2


    def Jacobi_h(x):
        """
        This is just a linear approximation of h
        using the Jacobi matrix of h
        """
        Jh = np.array([[ 2*x[0],      0,     0],
                       [      0, 2*x[1],     0],
                       [      0,      0, 2*x[2]]
                      ])
        return Jh


    

    print("Extended Kalman Filter demo start.")
    
    # 1.initialize simulation time    
    simu_time = 0.0

    # 2. prepare list to save all true states, measurements    
    true_xs = []
    zs = []
    pred_xs = []
    state_uncertainties = []


    # 3. create a Kalman filter object
    #
    # the Kalman filter needs to know
    # - initial estimate of state x
    # - initial estimate of uncertainty P about this state
    # - estimate for the transition matrix F
    # - estimate for the control matrix B
    # - estimate for the process noise Q
    # - estimate for the measurement noise R
    initial_x = np.array( [start_counter, start_pos, start_speed] )
    initial_P = np.array( [[0.0,  0.0, 0.0],
                           [0.0 , 0.0, 0.0],
                           [0.0 , 0.0, 0.0]
                          ]
                        )
    ekf = extended_kalman_filter.extended_kalman_filter(
            initial_x=initial_x,
            initial_P=initial_P,            
            Q=true_Q,            
            R=true_R,
            est_f=f,
            est_h=h,
            Jacobi_f=Jacobi_f,
            Jacobi_h=Jacobi_h)


    # 4. simulation loop
    true_x = initial_x.copy()
    while simu_time < max_simu_time:

        # 4.1 change state according to non-linear
        #     state transition function f
        true_x = f(true_x)

        # 4.2 add process noise to true state
        process_noise = np.random.multivariate_normal(np.array([0,0,0]), true_Q)
        #print(process_noise)
        true_x += process_noise

        # 4.3 Kalman filter predict step
        ekf.predict()
        
        # 4.4 simulate a noisy measurement
        measurement_noise = np.random.multivariate_normal(np.array([0,0,0]), true_R)
        #print(measurement_noise)
        z = h(true_x) + measurement_noise

        # 4.5 Kalman filter state estimate correction step?
        if simu_time != 0 and simu_time % measurement_each_nth_step == 0:
            ekf.correct_prediction_using_measurement(z)
            print( f"simu_time={simu_time} --> measurement correction step" )
        
        # 4.6 generate a list of historical values of
        #     - true states
        #     - measurements
        #     - predicted states
        #     - uncertainties about the predicted states
        true_xs.append( true_x )
        zs.append( z )
        pred_xs.append( ekf.x )
        uncertainty = ekf.get_scalar_measure_of_uncertainty_about_state()
        state_uncertainties.append( uncertainty )

        # 4.7 time goes by ...
        simu_time += dt

    # 5. prepare a figure
    plt.figure(figsize=(15,10))
    
    # 6. plot true, measured, predicted state1
    plt.subplot(4,1,1)
    plt.title("State 1")
    plt.xlabel("time [s]")
    plt.plot( [z[1] for z in zs], 'green', label="measured" )
    plt.plot( [x[1] for x in true_xs], 'red', label="true" )
    plt.plot( [x[1] for x in pred_xs], 'blue', label="EKF estimate" )
    plt.legend()

    # 7. plot true, measured, predicted state2
    plt.subplot(4,1,2)
    plt.title("State 2")
    plt.xlabel("time [s]")
    plt.plot( [z[2] for z in zs], 'green', label="measured" )
    plt.plot( [x[2] for x in true_xs], 'red', label="true" )
    plt.plot( [x[2] for x in pred_xs], 'blue', label="EKF estimate" )
    plt.legend()

    
    # 8. plot state uncertainty over time
    plt.subplot(4,1,3)
    plt.title("State uncertainty")
    plt.xlabel("time [s]")
    plt.ylabel("state uncertainty")    
    plt.plot( state_uncertainties, 'black')    
    
    # 9. plot error of KF state estimate vs. measurement
    # select a state dimension here (D=0 or D=1):
    D = 1
    ekf_errors  = [ est[D]-actual[D] for actual,est  in zip(true_xs, pred_xs)]
    z_errors = [meas[D]-actual[D] for actual,meas in zip(true_xs, zs)]
    plt.subplot(4,1,4)
    plt.title(f"Errors per time step for state dim #{D}")
    plt.xlabel("time [s]")
    plt.ylabel("error")    
    avg_error_ekf = np.mean(np.abs(ekf_errors))
    plt.plot( ekf_errors,
              'blue',
              label=f"EKF estimate error, avg={avg_error_ekf:.3f}")
    avg_error_meas = np.mean(np.abs(z_errors))
    plt.plot( z_errors,
              'green',              
              label=f"measurement error,  avg={avg_error_meas:.3f}" )
    plt.legend()

    # 10. add more vertical space between subplots
    plt.subplots_adjust(hspace=1)

    # 11. show figure
    plt.show()

    print("Extended Kalman Filter demo end.")
    

main()
