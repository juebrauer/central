/// file: kalman_filter_1d.cpp
///
/// by Prof. Dr. Juergen Brauer, www.juergenbrauer.org
///
/// --- 
/// for a good explanation of (a special case) of the
/// 1D Kalman filter case, see Sebastian Thrun's
/// videos:
///
/// https://www.youtube.com/watch?v=X7YggdDnLaw 
/// (prediction step)
/// and
/// https://www.youtube.com/watch?v=d8UrbKKlGxI 
/// (correction-by-measurement step)
///

#include "kalman_filter_1d.h"


kalman_filter_1d::kalman_filter_1d(double init_mu,
                                   double init_sigma,
                                   double process_noise,
                                   double measurement_noise)
{
  // store estimated start state mu
  mu = init_mu;

  // store estimated start uncertainty sigma
  sigma = init_sigma;

  // store specified state transition noise value
  this->process_noise = process_noise;

  // store specified measurement noise value
  this->measurement_noise = measurement_noise;

}



void kalman_filter_1d::predict(double u)
{
  mu = mu + u;
  sigma = sigma + process_noise;
}


void kalman_filter_1d::correct_by_measurement(double z)
{
  mu = (measurement_noise * mu + sigma * z) / (measurement_noise + sigma);

  sigma = 1.0 / (1.0/measurement_noise + 1.0/sigma);
}


double kalman_filter_1d::get_current_state_estimate()
{
  return mu;
}

double kalman_filter_1d::get_current_uncertainty()
{
  return sigma;
}