/// file: kalman_filter_1d.h
///
/// by Prof. Dr. Juergen Brauer, www.juergenbrauer.org


#pragma once

class kalman_filter_1d
{
  public:

               kalman_filter_1d(double init_mu,
                                double init_sigma,
                                double process_noise,
                                double measurement_noise);

      void     predict(double u);

      void     correct_by_measurement(double z);

      double   get_current_state_estimate();

      double   get_current_uncertainty();


  private:

      double   mu;                        // current estimated state with highest probability
      double   sigma;                     // uncertainty about the estimated state

      double   process_noise;             // how much (Gaussian) randomness is there during a state transition?
      double   measurement_noise;         // how much (Gaussian) randomness is there during a measurement?

};
