/// file: mvnrnd.h
///
/// Class declaration for a class
/// that represents a random number generator
/// that can generate n-dimensional
/// random vectors y that follow
/// a specified multivariate normal distribution, i.e.,
/// y ~ N(mu,S)
///    where mu is a n-dimensional mean vector
///          S  is a n x n covariance matrix
///
/// by Prof. Dr. Juergen Brauer

#pragma once                 // include this file not several times

#include "opencv2/core.hpp"  // for cv::Mat

#include <random>            // for 1D Gaussian random number generator std::normal_distribution<double> 

using namespace cv;



class mvnrnd
{
  public:

            mvnrnd(Mat mu, Mat S);

      Mat   get_next_random_vector();


  private:
          
      Mat                             mu;                // mean vector of the n-dimensional normal distribution
      Mat                             S;                 // covariance matrix of the n-dimensional normal distribution
      Mat                             C;                 // Cholesky decomposition of S, i.e, CC^T = S
      std::default_random_engine      generator;         // random number generator
      
}; // class mvnrnd