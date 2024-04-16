#include "Robot.h"

#include <opencv2/opencv.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#include "params.h"
#include "visualization.h"


Robot::Robot(string         name,
             int            radius,
             Point2d        start_pos,
             double         start_orientation,
             vector<double> sensor_angles,
             vector<double> sensor_distances,
             bool           noisy_sensors)
{
  this->name             = name;
  this->radius           = radius,
  this->pos              = start_pos;
  this->orientation      = start_orientation;
  this->sensor_angles    = sensor_angles;  
  this->sensor_distances = sensor_distances;
  this->noisy_sensors    = noisy_sensors;

  nr_sensors             = (int)sensor_angles.size();
  sensor_values          = new double[nr_sensors];  

  

  position_belief_map    = nullptr;  
}


double Robot::get_radius()
{
  return radius;
}

Point2d Robot::get_position()
{
  return pos;
}

double Robot::get_orientation()
{
  return orientation;
}

int Robot::get_nr_sensors()
{
  return nr_sensors;
}


double* Robot::get_sensor_values()
{
  return sensor_values;
}

vector<double> Robot::get_sensor_angles()
{
  return sensor_angles;
}

vector<double> Robot::get_sensor_distances()
{
  return sensor_distances;
}


///
/// For a given world <world> and a robot position <position>
/// compute the sensor value for each of the robot's distance
/// sensors and store them in the array <sensor_values>
///
/// If <simulate_noisy_sensors> we will add some random
/// uniformly distributed noise on these sensor values.
///

void Robot::compute_sensor_values(Mat& world,
                                  Point position,
                                  double* sensor_values,
                                  bool simulate_noisy_sensors)
{
  // 1. for each distance sensor
  for (int sensor_nr = 0; sensor_nr < nr_sensors; sensor_nr++)
  {
    // 1.1 get (x,y) coords of hypothetical robot position
    double x = position.x;
    double y = position.y;

    // 1.2 get sensor orientation relative to robots orientation
    double sensor_angle = sensor_angles[sensor_nr];

    // 1.3 map robot angle + sensor_angle to a direction vector
    double sensor_dx = cos(orientation + sensor_angle);
    double sensor_dy = sin(orientation + sensor_angle);

    // 1.4 compute sensor start position
    double sensor_startx = x + sensor_dx * radius;
    double sensor_starty = y + sensor_dy * radius;

    // now move from sensor start position into sensor direction
    // till we reach the maximum distance or hit an obstacle in the world (white pixel)

    // 1.6 get maximum sensor distance
    double sensor_max_dist = sensor_distances[sensor_nr];

    // 1.7 test step by step whether the next pixel is a black pixel == free space
    int step;
    for (step = 0; step < sensor_max_dist; step++)
    {
      // get next pixel location on sensor ray
      double sx = sensor_startx + step*sensor_dx;
      double sy = sensor_starty + step*sensor_dy;

      // invalid coordinates?
      if ((sx >= world.cols) || (sy >= world.rows) || (sx < 0) || (sy < 0))
         break;

      // get value of world pixel at the sensor ray position (sx,sy)
      Vec3b pixel_color = world.at<Vec3b>((int)sy, (int)sx);

      // is it black or white?
      if ((pixel_color.val[0] == 0) && (pixel_color.val[1] == 0) && (pixel_color.val[2] == 0))
      {
        // black pixel, so continue
        continue;
      }
      else
      {
        // white pixel: sensor ray reached a white pixel
        break;
      }

    } // for (move along sensor line)

    // 1.8 store final sensor value (= maximum distance or distance till we have found a non-black pixel)
    double final_sensor_value = (double)step;
    if (simulate_noisy_sensors)
    {
       int RANGE = (int) (sensor_max_dist / 10);
       double rnd_val = (double)(-RANGE / 2 + rand() % RANGE);
       final_sensor_value += rnd_val;
    }
    sensor_values[sensor_nr] = final_sensor_value;

    // 1.9 output sensor value for debugging
    //printf("sensor #%d: %d\n", sensor_nr, step);

  } // for (sensor_nr)

} // compute_sensor_values


///
/// For a given world <world> and a hypothetical robot position <position>
/// and a list of current real sensor values <real_sensor_values>
/// we compute the sensor values we would get (without any noise)
/// at the robot position <position>.
///
/// Already DURING the computation of these sensor values
/// we compute the sum of differences between these hypothetical sensor
/// values and the given real sensor values <real_sensor_values> and
/// if the sum reaches the threshold <max_allowed_difference>,
/// we stop the computation of the hypothetical sensor values
///
/// --> i.e., we then say: No! We cannot be at this hypothetical position
///     <position> since the sensor values we would get here are too
///     different compared to the actual real sensor values
///     <real_sensor_values>
///

bool Robot::check_hypothetical_position(Mat& world,
                                        Point position,
                                        double* real_sensor_values,
                                        double max_allowed_difference  
                                        )
{
   int sum_differences = 0;

   // 1. for each distance sensor
   for (int sensor_nr = 0; sensor_nr < nr_sensors; sensor_nr++)
   {
      // 1.1 get (x,y) coords of hypothetical robot position
      double x = position.x;
      double y = position.y;

      // 1.2 get sensor orientation relative to robots orientation
      double sensor_angle = sensor_angles[sensor_nr];

      // 1.3 map robot angle + sensor_angle to a direction vector
      double sensor_dx = cos(orientation + sensor_angle);
      double sensor_dy = sin(orientation + sensor_angle);

      // 1.4 compute sensor start position
      double sensor_startx = x + sensor_dx * radius;
      double sensor_starty = y + sensor_dy * radius;

      // now move from sensor start position into sensor direction
      // till we reach the maximum distance or hit an obstacle in the world (white pixel)

      // 1.6 get maximum sensor distance
      double sensor_max_dist = sensor_distances[sensor_nr];

      // 1.7 test step by step whether the next pixel is a black pixel == free space
      int step;
      for (step = 0; step < sensor_max_dist; step++)
      {
         // get next pixel location on sensor ray
         double sx = sensor_startx + step*sensor_dx;
         double sy = sensor_starty + step*sensor_dy;

         // invalid coordinates?
         if ((sx >= world.cols) || (sy >= world.rows) || (sx < 0) || (sy < 0))
            break;

         // get value of world pixel at the sensor ray position (sx,sy)
         Vec3b pixel_color = world.at<Vec3b>((int)sy, (int)sx);

         // is it black or white?
         if ((pixel_color.val[0] == 0) && (pixel_color.val[1] == 0) && (pixel_color.val[2] == 0))
         {
            // black pixel, so continue
            continue;
         }
         else
         {
            // white pixel: sensor ray reached a white pixel
            break;
         }

      } // for (move along sensor line)

      // 1.8 store final sensor value (= maximum distance or distance till we have found a non-black pixel)
      int final_sensor_value = step;

      int diff_to_real_sensor_value =
         abs(final_sensor_value - (int)(real_sensor_values[sensor_nr]));

      sum_differences += diff_to_real_sensor_value;

      if (sum_differences > max_allowed_difference)
         return false;

   } // for (sensor_nr)


   // Yes, that hypothetical position yields "similar"
   // sensor values compared to the current real sensor values
   return true;

} // check_hypothetical_position



bool Robot::test_wall_bump(Mat world)
{
   for (double angle = 0.0; angle < 2 * M_PI; angle += 0.1)
   {
      // compute world coordinates (x,y) of check point
      int x = (int) (pos.x + cos(angle) * radius);
      int y = (int) (pos.y + sin(angle) * radius);

      // is there a white pixel at (x,y)?
      Vec3b pixel_color = world.at<Vec3b>(y, x);
      if ((pixel_color.val[0] == 0) && (pixel_color.val[1] == 0) && (pixel_color.val[2] == 0))
      {
         // black pixel, so continue         
      }
      else
      {
         // white pixel: robot's outer radius has hit a wall!
         return true;
      }
   }

   // no outer point of the robot hit a wall point
   return false;

} // test_wall_bump



void Robot::create_position_belief_map(int W, int H)
{
   pbm_width  = W;
   pbm_height = H;

   // allocate memory for a 2D double array
   position_belief_map = new double*[pbm_height];
   for (int y = 0; y < pbm_height; y++)
   {
      position_belief_map[y] = new double[pbm_width];
   }

   reset_position_belief_map();

} // create_position_belief_map



void Robot::reset_position_belief_map()
{
   for (int y = 0; y < pbm_height; y++)
   {
      for (int x = 0; x < pbm_width; x++)
      {
         position_belief_map[y][x] = 0.0;

      } // for (x)
   } // for (y)

} // reset_position_belief_map



double Robot::get_sensor_vec_distance(int N, double* vec1, double* vec2)
{
   double dist = 0.0;
   for (int i = 0; i < N; i++)
   {
      dist += abs(vec1[i] - vec2[i]);
   }
   return dist;
    
} // get_sensor_vec_distance



void Robot::increase_belief_for_position(int x, int y, double inc)
{
   int Neigh = 2;
   for (int dy = -Neigh; dy <= Neigh; dy++)
   {
      for (int dx = -Neigh; dx <= Neigh; dx++)
      {
         int fx = x + dx;
         int fy = y + dy;
         if ((fx < 0) || (fy < 0) || (fx >= pbm_width) || (fy >= pbm_height))
            continue;

         position_belief_map[fy][fx] += inc;         
      }
   }  

} // increase_belief_for_position



void Robot::update_position_belief_map(Mat world)
{
   double* hypothetical_sensor_values = new double[nr_sensors];

   
   int W = world.cols;
   int H = world.rows;

   // forget hints for old positions...
   for (int y = 0; y < H; y++)
   {
      for (int x = 0; x < W; x++)
      {
          //if (position_belief_map[y][x] > 0.0)
          {
              //position_belief_map[y][x] -=0.005; // linear decay
              position_belief_map[y][x] *= BELIEF_DECAY_FACTOR; // exponential decay
              if (position_belief_map[y][x] < 0.01)
                  position_belief_map[y][x] = 0.0;
          }
      }
   }

   // "MATCHING" happens here!

   // check for each hypothetical world position
   // which sensor readings the robot would have
   // and compare it with the actual sensor readings
   const int METHOD = 2;
   for (int y = 0; y < H; y+=PARAM_MATCH_STEP)
   {
      for (int x = 0; x < W; x+=PARAM_MATCH_STEP)
      {
         // SLOW: compute total sensor value vector
         //       for each position considered
         if (METHOD == 1)
         {
            compute_sensor_values(world, Point(x, y), hypothetical_sensor_values, false);
            double dist = get_sensor_vec_distance(nr_sensors,
               hypothetical_sensor_values,
               sensor_values);
            if (dist < MAX_DIFFERENCE_SUM)
               increase_belief_for_position(x, y, 0.02);           
         }

         // FASTER: stop&exit computation of hypothetical sensor value vector
         //         if summed difference of hypothetical sensor value vector
         //         and actual sensor value vector is already too high
         //         --> early rejecting of position candidates that will yield
         //             too different sensor readings
         if (METHOD == 2)
         {
            bool position_possible =
               check_hypothetical_position(world, Point(x, y), sensor_values, MAX_DIFFERENCE_SUM);
            if (position_possible)
               increase_belief_for_position(x, y, 0.02); 
         }
         
      } // for (x)
   } // for (y)

   delete[] hypothetical_sensor_values;

} // update_position_belief_map



void Robot::update_visualization_of_position_belief_map(Mat world)
{
   visu_position_belief_map = world.clone();
   
   // map each current position belief value
   // to a color and set the pixel in the visualization
   for (int y = 0; y < pbm_height; y++)
   {
      for (int x = 0; x < pbm_width; x++)
      {
         Vec3b pixel_color = world.at<Vec3b>(y, x);
         if ((pixel_color.val[0] == 255) && (pixel_color.val[1] == 255) && (pixel_color.val[2] == 255))
         {
            // white pixel == wall, so continue
            continue;
         }

         // get belief value for position (x,y)
         double belief = position_belief_map[y][x];

         // map that value to a color
         Vec3b col;

         if (belief > 0)
         {
            col = map_prob_to_color((float)belief);
            visu_position_belief_map.at<Vec3b>(y, x) = col;
         }
         
         /*
         if (belief > 0)
            col = Vec3b(0, 0, 255);
         else
            col = Vec3b(0, 0, 0);
         */

         
      }
   }

   if (RESIZE_BELIEF_MAP_VISU)
      resize(visu_position_belief_map, visu_position_belief_map, Size(), 0.75, 0.75);

} // update_visualization_of_position_belief_map




void Robot::update(Mat world)
{
   // "SCAN" happens here!   
   // 1. compute new (distance) sensor values
   compute_sensor_values(world, pos, sensor_values, noisy_sensors);
   

   // 2. do we already have created a localization belief 2D array?
   if (position_belief_map == nullptr)
      create_position_belief_map( world.cols, world.rows );
     
   // 3. update belief of robot's position on the map (world)
   update_position_belief_map( world );


   // 4. navigation behavior

   // save old position
   Point2d old_pos = pos;

   // get sensor values
   double sensor_0 = sensor_values[0];
   double sensor_1 = sensor_values[1];
  
   const double one_radian = +M_PI / 180.0;
   
   // Behavior #1:
  // near to a wall?
   if ((sensor_0 < 10) || (sensor_1 < 10))
   {
      // turn left or right?
      if (sensor_0 < sensor_1)
         turn(M_PI / 16 + rand() % 50 * one_radian );
      else
         turn(-M_PI / 16);
   }
   else
      move(1);


   // 5. check for robot bumping into walls?
   if (0)
   {
      bool bumped = test_wall_bump(world);
      if (bumped)
      {
         printf("B");
         pos = old_pos;
      }
   }


   // 6. generate new visualization of position belief map ...
   update_visualization_of_position_belief_map(world);

   // 7. ... and show the visualization of the position belief map
   imshow("position belief map", visu_position_belief_map);

} // update


void Robot::move(double pixel)
{
  // get (x,y) coords of current robot position
  double x = pos.x;
  double y = pos.y;

  // map angle to a direction vector
  double dx = cos(orientation);
  double dy = sin(orientation);

  // add direction vector to current position to compute new robot position
  x += dx * pixel;
  y += dy * pixel;

  // store new position
  pos = Point2d(x, y);

} // move



void Robot::turn(double angle)
{
  orientation += angle;

} // turn


void Robot::set_wake_up_location(Point p)
{
   pos = p;
}


double Robot::get_distance_to_target()
{
   // compute direction vector to the target location
   double dirx = wake_up_location.x - pos.x;
   double diry = wake_up_location.y - pos.y;
   double distance_to_target = sqrt(dirx*dirx + diry*diry);

   return distance_to_target;
}

