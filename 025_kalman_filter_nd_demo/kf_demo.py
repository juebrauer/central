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
import kalman_filter
from params import *


def main():
    
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
    initial_x = np.array( [start_pos, start_speed] )
    initial_P = np.array( [[10.0, 0.0],
                           [0.0 , 1.0]
                          ]
                        )
    kf = kalman_filter.kalman_filter(initial_x=initial_x,
                                     initial_P=initial_P,
                                     F=true_F,
                                     B=true_B,
                                     Q=true_Q,
                                     H=true_H,
                                     R=true_R
                                     )


    # 4. simulation loop
    true_x = initial_x.copy()
    while simu_time < max_simu_time:

        # 4.1 change state according to linear equation x=Fx+Bu
        #true_x = true_F @ true_x
        true_x = true_F @ true_x + true_B @ u

        # 4.2 add process noise to true state
        process_noise = np.random.multivariate_normal(np.array([0,0]), true_Q)
        #print(process_noise)
        true_x += process_noise

        # 4.3 Kalman filter predict step
        kf.predict(u)        
        
        # 4.4 simulate a noisy measurement
        measurement_noise = np.random.multivariate_normal(np.array([0,0]), true_R)
        #print(measurement_noise)
        z = true_H @ true_x + measurement_noise

        # 4.5 Kalman filter state estimate correction step?
        if simu_time != 0 and simu_time % measurement_each_nth_step == 0:
            kf.correct_prediction_using_measurement(z)
            #print( f"simu_time={simu_time} --> measurement correction step" )
        
        # 4.6 generate a list of historical values of
        #     - true states
        #     - measurements
        #     - predicted states
        #     - uncertainties about the predicted states
        true_xs.append( true_x )
        zs.append( z )
        pred_xs.append( kf.x )
        uncertainty = kf.get_scalar_measure_of_uncertainty_about_state()
        state_uncertainties.append( uncertainty )

        # 4.7 time goes by ...
        simu_time += dt

    # 5. prepare a figure
    plt.figure(figsize=(15,10))
    
    # 6. plot true, measured, predicted positions
    plt.subplot(4,1,1)
    plt.title("Position")
    plt.xlabel("time [s]")
    plt.ylabel("position [m]")
    plt.plot( [z[0] for z in zs], 'green', label="measured" )
    plt.plot( [x[0] for x in true_xs], 'red', label="true" )
    plt.plot( [x[0] for x in pred_xs], 'blue', label="KF estimate" )
    plt.legend()

    # 7. plot true, measured speeds
    plt.subplot(4,1,2)
    plt.title("Speed")
    plt.xlabel("time [s]")
    plt.ylabel("speed [m/s]")    
    plt.plot( [z[1] for z in zs], 'green', label="measured" )
    plt.plot( [x[1] for x in true_xs], 'red', label="true" )
    plt.plot( [x[1] for x in pred_xs], 'blue', label="KF estimate" )
    plt.legend()

    # 8. plot state uncertainty over time
    plt.subplot(4,1,3)
    plt.title("State uncertainty")
    plt.xlabel("time [s]")
    plt.ylabel("state uncertainty")    
    plt.plot( state_uncertainties, 'black')    

    # 9. plot error of KF state estimate vs. measurement    
    kf_pos_errors   = [ est[0]-actual[0] for actual,est  in zip(true_xs, pred_xs)]
    meas_pos_errors = [meas[0]-actual[0] for actual,meas in zip(true_xs, zs)]
    plt.subplot(4,1,4)
    plt.title("Errors per time step")
    plt.xlabel("time [s]")
    plt.ylabel("error")    
    plt.plot( kf_pos_errors,   'blue',  label="KF pos error")
    plt.plot( meas_pos_errors, 'green', label="measurement pos error")
    plt.legend()

    # 10. add more vertical space between subplots
    plt.subplots_adjust(hspace=1)

    # 11. show figure
    plt.show()
    

main()
