/// file: mvnrnd.cpp
///
/// Implementation for a class
/// that represents a random number generator
/// that can generate n-dimensional
/// random vectors y that follow
/// a specified multivariate normal distribution, i.e.,
/// y ~ N(mu,S)
///    where mu is a n-dimensional mean vector
///          S is a n x n covariance matrix
///
/// ---
/// by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

#include "mvnrnd.h"

#include "math_tools.h" // for computing the Cholesky decomposition C of S, i.e., such that CC^T = S

mvnrnd::mvnrnd(Mat mu, Mat S)
{
  this->mu = mu;
  this->S  = S;

  this->C = cholesky_decomposition(S);
}



/// returns an n-dimensional random vector
/// distributed according to a multivariate normal distribution
/// with mean mu and covariance matrix S
///
/// See
/// https://www.quora.com/How-can-I-generate-two-dependent-random-variables-follow-the-standard-normal-distribution-which-has-the-correlation-value-0-5-in-C++-11
Mat mvnrnd::get_next_random_vector()
{
  int n = mu.rows;

  // 1. generate a random vector Z of uncorrelated variables
  Mat Z = Mat(n, 1, CV_32F);
  std::normal_distribution<float> distribution(0.0, 1.0);
  for (int i = 1; i <= n; i++)
  {
    float rnd_number = distribution(generator);
    Z.at<float>(i - 1, 0) = rnd_number;
  }

  // 2. now map that vector of uncorrelated variables to one
  //    of correlated variables
  Mat Y = mu + C*Z;

  // 3. return that random vector
  return Y;

} // get_next_random_vector
