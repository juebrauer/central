/// The spaceship class
///
/// implements a simulated spaceship of
/// the aliens that has the ability to
/// split into several parts, where the
/// parts can move independently from each other!
///
/// ---
/// by Prof. Dr. J�rgen Brauer, www.juergenbrauer.org

#pragma once

#include <opencv2/core.hpp> // for Point
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <vector>

#include "params.h"

using namespace std;
using namespace cv;

class spaceship
{  

public:

    struct part_info
    {
      Point location;
      Point move_vec;
    };

                               spaceship(int nr_of_parts, Size world_size);

    void                       move();

    void                       draw_yourself_into_this_image(Mat& img);

    vector<part_info*>         get_part_info_vector();

private:

  int                          nr_of_parts;
  Size                         world_size;
  vector<part_info*>           part_infos;

  Point                        get_rnd_move_vec();

}; // class spaceship
