#include "Robot.h"

#include <opencv2/opencv.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#include "params.h"




Robot::Robot(string         name,
             int            radius,
             Point2d        start_pos,
             double         start_orientation,
             vector<double> sensor_angles,
             vector<double> sensor_distances)
{
  this->name             = name;
  this->radius           = radius,
  this->pos              = start_pos;
  this->orientation      = start_orientation;
  this->sensor_angles    = sensor_angles;
  this->sensor_distances = sensor_distances;
  nr_sensors              = (int) sensor_angles.size();
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


vector<double> Robot::get_sensor_values()
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


void Robot::compute_sensor_values(Mat world)
{
  // 1. clear old sensor values
  sensor_values.clear();

  // 2. for each distance sensor
  for (int sensor_nr = 0; sensor_nr < nr_sensors; sensor_nr++)
  {
    // 2.1 get (x,y) coords of current robot position
    double x = pos.x;
    double y = pos.y;

    // 2.2 get sensor orientation relative to robots orientation
    double sensor_angle = sensor_angles[sensor_nr];

    // 2.3 map robot angle + sensor_angle to a direction vector
    double sensor_dx = cos(orientation + sensor_angle);
    double sensor_dy = sin(orientation + sensor_angle);

    // 2.4 compute sensor start position
    double sensor_startx = x + sensor_dx * radius;
    double sensor_starty = y + sensor_dy * radius;

    // 2.5 now move from sensor start position into sensor direction
    //     till we reach the maximum distance or hit an obstacle in the world (white pixel)

    // 2.6 get maximum sensor distance
    double sensor_max_dist = sensor_distances[sensor_nr];

    // 2.7 test step by step whether the next pixel is a black pixel == free space
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

    // 2.8 store final sensor value (= maximum distance or distance till we have found a non-black pixel)
    sensor_values.push_back(step);

    // 2.9 output sensor value for debugging
    //printf("sensor #%d: %d\n", sensor_nr, step);

  } // for (sensor_nr)

} // compute_sensor_values



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



void Robot::update(Mat world)
{
  compute_sensor_values(world);

  // save old position
  Point2d old_pos = pos; 

  // do we already planned a route?
  if (!route_planned)
  {
    route_in_world.clear();
    plan_route_to_target(world, WAVE_FRONT_ALGORITHM_GRID_CELL_SIZE);
    if (!route_planned)
        return;
    next_milestone_nr = 0;
    current_behavior_mode = TURN_TO_GOAL;
    next_milestone_point = route_in_world[next_milestone_nr];
  }

  // get sensor values     
  const double one_radian = +M_PI / 180.0;


  ///
  /// Behavior: Turn to goal
  ///
  if (current_behavior_mode == TURN_TO_GOAL)
  {
     
     // compute direction vector to the next milestone location
     double dirx = next_milestone_point.x - pos.x;
     double diry = next_milestone_point.y - pos.y;

     // compute goal angle
     double angle_goal = atan2(diry, dirx);
     
     printf("angle_goal=%.1f, robot_angle=%.1f\n", angle_goal, orientation);

     // Get the (minimum) difference between the two angles
     // i) (robot) orientation and ii) angle_goal
     // see
     // http://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
     //
     // note: angle_diff is not the absolute difference, but a signed difference
     //       i.e., it still preserves the information in which direction to turn!
     double angle_diff = atan2(sin(orientation - angle_goal), cos(orientation - angle_goal));
     

     // is the goal angle and the current orientation
     // of the robot similar enough?
     if (abs(angle_diff) <= 2.0*one_radian)
     {
        current_behavior_mode = GO_STRAIGHT_TO_GOAL;
     }
     else
     {
        // compute turn direction
        double turn_angle;
        if (angle_diff<0)
           turn_angle = +one_radian;
        else
           turn_angle = -one_radian;

        // command the robot to turn
        turn(turn_angle);
     }
     

     /*
     double angle_diff = atan2(pos.y, pos.x) -
                         atan2(next_milestone_point.y, next_milestone_point.x);

     // is the goal angle and the current orientation
     // of the robot similar enough?
     if (abs(angle_diff) <= 2.0*one_radian)
     {
        current_behavior_mode = GO_STRAIGHT_TO_GOAL;
     }
     else
     {
        // compute turn direction
        double turn_angle;
        if (angle_diff<0)
           turn_angle = +one_radian;
        else
           turn_angle = -one_radian;

        // command the robot to turn
        turn(turn_angle);
     }
     */
  }


  ///
  /// Behavior: Go straight to goal
  ///
  if (current_behavior_mode == GO_STRAIGHT_TO_GOAL)
  {
     // get distance to next milestone point
     double dx = next_milestone_point.x - pos.x;
     double dy = next_milestone_point.y - pos.y;
     double dist = sqrt(dx*dx + dy*dy);
     if (dist <= 2)
     {
        current_behavior_mode = TURN_TO_GOAL;
        next_milestone_nr++;
        if (next_milestone_nr == route_in_world.size())
        {
           route_planned = false; // restart!           
        }
        else
        {
           next_milestone_point = route_in_world[next_milestone_nr];
        }
     }
     else
        move(1);     
  }
   

  // did the robot bump into a wall?
  if (0)
  {
     bool bumped = test_wall_bump(world);
     if (bumped)
     {
        printf("B");
        pos = old_pos;
     }
  }

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


void Robot::set_target_location(Point p)
{
   target_location = p;
   route_planned = false;
}


double Robot::get_distance_to_target()
{
   // compute direction vector to the target location
   double dirx = target_location.x - pos.x;
   double diry = target_location.y - pos.y;
   double distance_to_target = sqrt(dirx*dirx + diry*diry);

   return distance_to_target;
}


/// method: plan_route_to_target()
///
/// compute a route in world coordinates
/// to get the robot from its current location (pos.x, pos.y)
/// to the desired target location (target_location.x, target_location.y)
///
/// the final route in world coordinates is stored in the vector
/// vector<Point> route_in_world
///
void Robot::plan_route_to_target(Mat world, int grid_cell_size)
{
   vector<Point> neighbors;
   neighbors.push_back(Point(-1, 0));
   neighbors.push_back(Point( 0, 1));
   neighbors.push_back(Point(+1, 0));
   neighbors.push_back(Point( 0,-1));
   //neighbors.push_back(Point(-1,-1));
   //neighbors.push_back(Point(+1,+1));
   //neighbors.push_back(Point(-1,+1));
   //neighbors.push_back(Point(+1,-1));

   // 1. compute grid dimensions
   grid_width  = (world.cols / grid_cell_size) + 1;
   grid_height = (world.rows / grid_cell_size) + 1;


   // 2. allocate memory for grid
   grid = new int*[grid_height];
   for (int gy = 0; gy < grid_height; gy++)
      grid[gy] = new int[grid_width];


   // 3. initialize all grid cells with 0
   for (int gy = 0; gy < grid_height; gy++)
      for (int gx = 0; gx < grid_width; gx++)
         grid[gy][gx] = 0;
   

   // 4. mark grid cells which are occupied with 1's
   for (int y = 0; y < world.rows; y++)
   {
      for (int x = 0; x < world.cols; x++)
      {
         int gx = x / grid_cell_size;
         int gy = y / grid_cell_size;
         
         // is pixel black or white?
         Vec3b pixel_color = world.at<Vec3b>(y, x);
         if ((pixel_color.val[0] == 0) &&
             (pixel_color.val[1] == 0) &&
             (pixel_color.val[2] == 0))
         {
            // black pixel, so continue
            continue;
         }
         else
         {
            // white pixel, so mark grid cell as occupied
            grid[gy][gx] = 1;
         }         
      } // for (x)
   } // for (y)

   
   // 5. compute robot's position on grid ...
   int robot_gridx = (int) (pos.x / grid_cell_size);
   int robot_gridy = (int) (pos.y / grid_cell_size);

   // ... and target grid position
   int target_gridx = target_location.x / grid_cell_size;
   int target_gridy = target_location.y / grid_cell_size;


   // 6. now let the wave propagate from the
   //    target to the start position of the robot
   int current_wave_step_nr = 2;
   grid[target_gridy][target_gridx] = current_wave_step_nr;
   bool finished = false;
   route_on_grid.clear();
   bool wave_could_not_propagate_further;
   
   while (!finished)
   {
      printf("current_wave_step_nr=%d\n", current_wave_step_nr);
      wave_could_not_propagate_further = true;

      // 6.1 propagate wave one step further
      for (int gy = 0; gy < grid_height; gy++)
      {
         for (int gx = 0; gx < grid_width; gx++)
         {
            // is the cell a cell to propagate further?
            if (grid[gy][gx] == current_wave_step_nr)
            {
               for (uint i = 0; i < neighbors.size(); i++)
               {
                  // get neighbor offset vector
                  Point neighbor = neighbors[i];

                  // compute final position
                  int fx = gx + neighbor.x;
                  int fy = gy + neighbor.y;

                  // is final position (fx,fy) in the grid?
                  if ((fx < 0) || (fy < 0) || (fx >= grid_width) || (fy >= grid_height))
                     continue;

                  // is the neighbor cell free?
                  if (grid[fy][fx] == 0)
                  {
                     // yes! so propagate wave to there!
                     grid[fy][fx] = current_wave_step_nr + 1;
                     wave_could_not_propagate_further = false;

                     // did we reach the start cell?
                     if ((fx == robot_gridx) && (fy == robot_gridy))
                     {
                        // yes! so we have found a path from the
                        // target backwards to the start goal
                        finished = true;
                     }
                  }

               } // for (all neighbors)

            } // if
         } // for (gx)
      } // for (gy)

      // 6.2 visualize grid
      Mat grid_visu = get_grid_visu(20, grid_cell_size);
      imshow("grid", grid_visu);
      waitKey(0);

      // the goal seems to be in a region of the 2D world
      // that is not reachable!
      if (wave_could_not_propagate_further)
      {
          printf("Wavefront propagation stopped before "
                 "reaching the target location!\n");
          
          // stop planing!
          return;
      }

      // 6.3 goto next wave step nr      
      current_wave_step_nr++;

   } // while wave still can propagate...

   printf("Found path from target backwards to start position!\n");

   // 7. now extract a path by following the 
   //    grid cells backwards from <current_wave_step_nr> to 2   
   Point current_cell(robot_gridx, robot_gridy);
   route_in_world.clear();
   for (int wave_step_nr = current_wave_step_nr-1;
        wave_step_nr >= 2;
        wave_step_nr--)
   {
      // search a cell neighbored to the current cell
      // that has the correct wave_step_nr
      for (uint i = 0; i < neighbors.size(); i++)
      {
         // get neighbor offset vector
         Point neighbor = neighbors[i];

         // compute final position
         Point n = current_cell + neighbor;

         // is neighbor cell n in the grid?
         if ((n.x < 0) || (n.y < 0) || (n.x >= grid_width) || (n.y >= grid_height))
            continue;

         // has the neighbor cell the correct wave_step_nr?
         if (grid[n.y][n.x] == wave_step_nr)
         {
            current_cell = n;

            route_on_grid.push_back(current_cell);

            int world_x = n.x * grid_cell_size;
            int world_y = n.y * grid_cell_size;
            route_in_world.push_back(Point(world_x, world_y));

            break; // only one neighbor with the current wave_step_nr is needed
         }
      } // for (all neighbors)
   } // for (all wave steps to go back)
   route_in_world.push_back(target_location);

   // 8. visualize grid with wave step numbers and final route
   Mat grid_visu = get_grid_visu(20, grid_cell_size);
   imshow("grid", grid_visu);
   waitKey(0);
      

   // 9. remember that we already have planned
   //    the route to the target when calling
   //    update() in the future
   route_planned = true;


   // 10. release memory
   for (int gy = 0; gy < grid_height; gy++)
      delete[] grid[gy];
   delete[] grid;

} // plan_route_to_target



Mat Robot::get_grid_visu(int visu_cell_size,
                         int grid_cell_size)
{
   // 1. prepare grid visualization image
   Mat grid_visu(grid_height*visu_cell_size, grid_width*visu_cell_size, CV_8UC3);


   // 2. target cell in grid world?
   int target_gridx = target_location.x / grid_cell_size;
   int target_gridy = target_location.y / grid_cell_size;
  

   // 3. start cell in grid world?
   int robot_gridx = (int)(pos.x / grid_cell_size);
   int robot_gridy = (int)(pos.y / grid_cell_size);
  

   // 4. for each grid cell to draw ...   
   for (int gy = 0; gy < grid_height; gy++)
   {
      for (int gx = 0; gx < grid_width; gx++)
      {
         Vec3b cell_color;
         if (grid[gy][gx] == 1)
         {
            // cell is occupied
            cell_color = Vec3b(0, 0, 255);
         }
         else
         {
            // cell is free space
            cell_color = Vec3b(0,0,0);
         }

         // special cells?
         if ((gx == target_gridx) && (gy == target_gridy))
            cell_color = Vec3b(0, 255, 0); // green

         if ((gx == robot_gridx) && (gy == robot_gridy))
            cell_color = Vec3b(255, 0, 0); // blue

         // show white border around cell
         int x = gx*visu_cell_size;
         int y = gy*visu_cell_size;
         rectangle(grid_visu, Rect(x, y, visu_cell_size, visu_cell_size),
                   CV_RGB(255,255,255), 1);

         // fill cell with cell color
         rectangle(grid_visu, Rect(x+1, y+1, visu_cell_size-1, visu_cell_size-1),
                   cell_color, -1);

         // show wave step nr in cell
         char txt[100];
         Vec3b col = Vec3b(255, 255, 255);
         if ((grid[gy][gx] != 0) && (grid[gy][gx] != 1))
            col = Vec3b(0, 255, 0);
         sprintf(txt, "%d", grid[gy][gx]);
         putText(grid_visu,
            txt,
            Point(x + 1, y + 15),
            FONT_HERSHEY_SIMPLEX, 0.4, // font face and scale
            col, // white
            1); // line thickness and type
      }
   }  

   // 5. draw grid route from start to target
   //    - if we have already found one
   for (uint i = 0; i < route_on_grid.size(); i++)
   {
      int gx = route_on_grid[i].x;
      int gy = route_on_grid[i].y;

      int x = gx*visu_cell_size;
      int y = gy*visu_cell_size;

      rectangle(grid_visu, Rect(x, y, visu_cell_size, visu_cell_size),
                CV_RGB(255,255,0), 2);
   }


   // 6. return the visualization image
   return grid_visu;

} // get_grid_visu


vector<Point> Robot::get_planned_route_in_world_coords()
{
   return route_in_world;
}