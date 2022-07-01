"""
A demo for the n-dimensional Kalman-Filter (KF).

This filter can estimate n-dimensional states.
By fusing predictions and measurements over time,
the KF can come up with a better state estimate
than just "believing" noisy measurements directly.

by Prof. Dr. Juergen Brauer
www.juergen.brauer.org
"""

import numpy as np
import matplotlib.pyplot as plt
import extended_kalman_filter
from params import *


def main():

    def f(x):
        """
        This is the non-linear true state transition function
        f:R^3 --> R^3

        f(x) = f(x0,x1,x2) = (x0+1, sin(x0), cos(x0))
                           = (f1(x), f2(x), f3(x))
        """

        new_state0 = x[0] + 1
        new_state1 = np.sin(x[0])
        new_state2 = np.cos(x[0])
        new_x = np.array( [new_state0, new_state1, new_state2] )
        return new_x


    def est_f(x):
        """
        This is the non-linear true state transition function
        f:R^3 --> R^3

        f(x) = f(x0,x1,x2) = (x0+1, sin(x0), cos(x0))
                           = (f1(x), f2(x), f3(x))
        """

        new_state0 = x[0] + 1
        new_state1 = 2*np.cos(x[0])
        new_state2 = 0.5*np.sin(x[0])
        new_x = np.array( [new_state0, new_state1, new_state2] )
        return new_x


    def Jacobi_f(x):

        """
        Jf = | df1/dx0, df1/dx1, df1/dx2  |
             | df2/dx0, df2/dx1, df2/dx2  |
             | df3/dx0, df3/dx1, df3/dx2  |
           = | d(x0+1)/dx0,       d(x0+1)/dx1,    d(x0+1)/dx2  |
             | d(sin(x0))/dx0, d(sin(x0))/dx1, d(sin(x0))/dx2  |
             | d(cos(x0))/dx0, d(cos(x0))/dx1, d(cos(x0))/dx2  |
        """

        Jf = np.array( [[            1, 0, 0],
                        [np.cos(x[0]),  0, 0],
                        [-np.sin(x[0]), 0, 0]
                       ])
        return Jf


    def h(x):
        """
        This is the non-linear true measurement function
        h:R^3 --> R^3

        h(x) = h(x0,x1,x2) = (x[0]^2, x[1]^2, x[2]^2)
                           = (h1(x), h2(x), h3(x))
        """
        return x**2


    def Jacobi_h(x):

        """
        Jh = | dh1/dx0, dh1/dx1, dh1/dx2  |
             | dh2/dx0, dh2/dx1, dh2/dx2  |
             | dh3/dx0, dh3/dx1, dh3/dx2  |
        """

        Jh = np.array([[2*x[0],       0,      0],
                       [     0,  2*x[1],      0],
                       [     0,       0, 2*x[2]]
                      ])
        return Jh


    # 0. make sure that from one experiment to the other
    #    the process and measurement noise random vectors
    #    are the same
    #np.random.seed(42)

    
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
    
    initial_x = np.array( [start_state0, start_state1, start_state2] )

    initial_P = np.array( [[0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0]
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
         Jacobi_h=Jacobi_h
        )
    


    # 4. simulation loop
    true_x = initial_x.copy()
    while simu_time < max_simu_time:

        # 4.1 change state according to non-linear function
        true_x = f(true_x)
    
        # 4.2 add process noise to true state
        process_noise = np.random.multivariate_normal(np.array([0,0,0]), true_Q)
        #print( f"simu_time={simu_time}, process_noise={process_noise}" )
        true_x += process_noise

        # 4.3 Kalman filter predict step
        ekf.predict()        
        
        # 4.4 simulate a noisy measurement
        measurement_noise = np.random.multivariate_normal(np.array([0,0,0]), true_R)
        #print(measurement_noise)
        z = h(true_x) + measurement_noise

        # 4.5 Kalman filter state estimate correction step?       
        if simu_time != 0 and simu_time % measurement_correct_each_nth_step == 0:
            ekf.correct_prediction_using_measurement(z)
            #print( f"simu_time={simu_time} --> measurement correction step" )
     

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
    
    # 6. plot true, measured, predicted positions
    plt.subplot(4,1,1)
    plt.title("State 0")
    plt.xlabel("time [s]")
    #plt.plot( [z[0] for z in zs], 'green', label="measured" )
    plt.plot( [x[0] for x in true_xs], 'red', label="true" )
    plt.plot( [x[0] for x in pred_xs], 'blue', label="EKF estimate" )
    plt.legend()


    # 7. plot true, measured speeds
    plt.subplot(4,1,2)
    plt.title("State 1")
    plt.xlabel("time [s]")
    plt.plot( [z[1] for z in zs], 'green', label="measured" )
    plt.plot( [x[1] for x in true_xs], 'red', label="true" )
    plt.plot( [x[1] for x in pred_xs], 'blue', label="EKF estimate" )
    plt.legend()

    # 8. plot state uncertainty over time
    plt.subplot(4,1,3)
    plt.title("State uncertainty")
    plt.xlabel("time [s]")
    plt.ylabel("state uncertainty")    
    plt.plot( state_uncertainties, 'black')    

    # 9. plot error of KF state estimate vs. measurement    
    ekf_state1_errors   = [ est[1]-actual[1] for actual,est  in zip(true_xs, pred_xs)]
    meas_state1_errors  = [meas[1]-actual[1] for actual,meas in zip(true_xs, zs)]
    plt.subplot(4,1,4)
    plt.title("Errors per time step")
    plt.xlabel("time [s]")
    plt.ylabel("error")

    ekf_mean_state1_error = np.mean(np.abs(ekf_state1_errors))    
    plt.plot( ekf_state1_errors,  'blue',
              label=f"EKF state1 error: mean={ekf_mean_state1_error:.3f}" )

    meas_mean_state1_error = np.mean(np.abs(meas_state1_errors)) 
    plt.plot( meas_state1_errors, 'green',
              label=f"measurement state1 error: mean={meas_mean_state1_error:.3f}" )
    plt.legend()


    # 10. add more vertical space between subplots
    plt.subplots_adjust(hspace=1)

    # 11. show figure
    plt.show()
    

main()
