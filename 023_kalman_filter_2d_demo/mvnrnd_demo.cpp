#include <iostream>
#include <fstream>

#include "mvnrnd.h"



int main()
{
    std::cout << "mvnrnd_demo started\n";

    cv::Mat mu    = (Mat_<float>(2, 1) << 5,7 );   
    cv::Mat SIGMA = (Mat_<float>(2, 2) <<   10.0,  0.0,
                                            0.0,   1.0);

    mvnrnd r = mvnrnd(mu, SIGMA);

    std::ofstream f( "random_vectors.txt" );

    uint N=10000;
    for (uint i=0; i<N; i++)
    {
        Mat rnd_vec = r.get_next_random_vector();
        f << "(" << rnd_vec.at<float>(0,0) << "," << rnd_vec.at<float>(1,0) << ")" << std::endl;
    }

    f.close();

    std::cout << "mvnrnd_demo finished\n";
}

