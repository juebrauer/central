/// Robot navigation WITH a map
///
/// The goal of this exercise is to let students think
/// about own ideas to bring a robot from a point A
/// in the world to a point B WITH the help of a map.
///
/// The user can select a target point in the 2D world
/// and the robot will try to reach this point with
/// using a map. In this implementation the robot will
/// use the "Wavefront" algorithm on a discretized grid
/// version of the world in order to plan a route from
/// its current location to the target location.
///
///
/// ---
/// by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#define SAVE_IMAGES 0
#define SAVE_FOLDER "~/tmp"

#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>

#include "Robot.h"



using namespace cv;
using namespace std;

bool exit_simulation;
Point current_mouse_pos;
Point selected_target_pos;

void mouse_callback_func(int event, int x, int y, int flags, void* userdata)
{
   current_mouse_pos = Point2i(x, y);
   if (event == EVENT_LBUTTONDOWN) {
      selected_target_pos = current_mouse_pos;
      return;
   }
}

void simulate_one_episode(Mat world,
                          Robot& r1,
                          vector<double> sensor_angles,
                          vector<double> sensor_distances)
{
   printf("Robot will try to reach the target location (%d, %d)\n",
           selected_target_pos.x, selected_target_pos.y);

   int simulation_step = 0;
   Mat image;
   while (!exit_simulation)
   {
      // 1. initialize image with world image
      world.copyTo(image);


      // 2. move the robot according to its specific behavior
      r1.update(world);


      // 3. show planned route
      vector<Point> planned_route = r1.get_planned_route_in_world_coords();
      if (planned_route.size() > 0)
      {
         Point last_pos = planned_route[0];
         for (uint i = 1; i < planned_route.size(); i++)
         {
            Point next_pos = planned_route[i];
            line(image, last_pos, next_pos, CV_RGB(255, 255, 0), 1);
            circle(image, next_pos, 5, CV_RGB(255, 255, 0), -1);
            last_pos = next_pos;
         }
      }
      else
      {
          // no route found / planned!
          break;
      }


      // 4. show the robot's position as a circle and its orientation by a line
      Point2d pos = r1.get_position();
      double  ori = r1.get_orientation();
      double  dx = cos(ori);
      double  dy = sin(ori);
      double  r = r1.get_radius();
      circle(image, pos, (int)r, CV_RGB(255, 0, 0), 1);
      line(image, pos, pos + Point2d(dx*r, dy*r), CV_RGB(0, 255, 0), 1);


      // 5. compute all the sensor values
      vector<double> sensor_values = r1.get_sensor_values();


      // 6. for each sensor draw a sensor ray
      for (int sensor_nr = 0; sensor_nr < r1.get_nr_sensors(); sensor_nr++)
      {
         double sensor_value = sensor_values[sensor_nr];

         // get (x,y) coords of current robot position
         double x = pos.x;
         double y = pos.y;

         // get sensor orientation relative to robots orientation
         double sensor_angle = sensor_angles[sensor_nr];

         // map robot angle + sensor_angle to a direction vector
         double sensor_dx = cos(r1.get_orientation() + sensor_angle);
         double sensor_dy = sin(r1.get_orientation() + sensor_angle);

         // compute sensor start position
         double sensor_startx = x + sensor_dx * r1.get_radius();
         double sensor_starty = y + sensor_dy * r1.get_radius();

         // compute sensor ray end position
         double sensor_endx = sensor_startx + sensor_dx * sensor_value;
         double sensor_endy = sensor_starty + sensor_dy * sensor_value;

         // draw sensor ray line
         line(image, Point((int)sensor_startx, (int)sensor_starty), Point((int)sensor_endx, (int)sensor_endy), CV_RGB(255, 255, 0), 1);

      } // for (draw all sensor rays)


      // 7. show simulation step nr
      char txt[100];
      sprintf(txt, "simulation step %d", simulation_step);
      putText(image,
         txt,
         Point(20, 50),
         FONT_HERSHEY_SIMPLEX, 0.7, // font face and scale
         CV_RGB(255, 0, 0), // white
         1); // line thickness and type


      // 8. show target location as green circle
      circle(image, selected_target_pos, 5, CV_RGB(0,255,0), -1);
      

      // 9. show world with robot, sensor rays and additional textual information
      imshow("Robot Simulator", image);


      // 10. save image?
      if (SAVE_IMAGES)
      {
         char fname[500];
         sprintf(fname, "%s\\img%05d.png", SAVE_FOLDER, simulation_step);
         imwrite(fname, image);
      }


      // 11. wait for a key
      char c = (char)waitKey(10);


      // 12. ESC pressed?
      if (c == 27)
         exit_simulation = true;

      // 13. did the robot reach the goal?
      if (norm((Point)(r1.get_position()) - selected_target_pos) <= 1.25*r1.get_radius())
      {
         printf("Robot reached target location!\n\n\n");
         exit_simulation = true;         
      }


      // 14. one step simulated more
      simulation_step++;

   } // while (simulation shall continue)

} // simulate_one_episode


Point select_new_target_location(Mat world, Robot r)
{
   printf("Please select a new target location to go to ...\n");

   // 1. set up a mouse callback function
   namedWindow("Robot Simulator", 1);
   setMouseCallback("Robot Simulator", mouse_callback_func, nullptr);


   // 2. wait for user to clock on a location in the world image
   Mat image;
   selected_target_pos = Point(-1,-1);
   int w = world.cols-1;
   int h = world.rows-1;
   int blink_counter = -100;
   while (selected_target_pos.x == -1)
   {
      world.copyTo(image);

      // show robot pos by blue blinking circle
      circle(image, r.get_position(), 10, CV_RGB(0,0,255), blink_counter++<0?-1:5);
      if (blink_counter>100)
         blink_counter=-100;

      int x = current_mouse_pos.x;
      int y = current_mouse_pos.y;

      line(image, Point(0,y), Point(w,y), CV_RGB(0, 255, 0), 1);
      line(image, Point(x,0), Point(x,h), CV_RGB(0, 255, 0), 1);

      imshow("Robot Simulator", image);
      waitKey(1);
   }

   printf("User selected target location (%d, %d)\n",
          selected_target_pos.x, selected_target_pos.y);
   
   return selected_target_pos;

} // select_new_target_location


int main()
{
  // 1. load image of world: change this to the folder where you store world1.png!
  //    white pixels mean: no drivable space
  //    black pixels mean: drivable space
  string img_filename = "world1.png";
  Mat world = imread(img_filename);


  // 2. check whether world dimensions are valid
  if ((world.cols == 0) && (world.rows == 0))
  {
    cout << "Error! Could not read the image file: " << img_filename << endl;
    return -1;
  }


  // 3. get world dimensions
  int WORLD_WIDTH  = world.cols;
  int WORLD_HEIGHT = world.rows;


  // 4. prepare an image (where we will draw the robot and the world)
  Mat image(WORLD_HEIGHT, WORLD_WIDTH, CV_8UC3);


  // 5. create a robot
  vector<double> sensor_angles, sensor_distances;
  sensor_angles.push_back(0.0);
  sensor_angles.push_back(-M_PI / 3);
  sensor_angles.push_back(-M_PI / 6);
  sensor_angles.push_back(-M_PI / 8);
  sensor_angles.push_back(+M_PI / 8);
  sensor_angles.push_back(+M_PI / 6);
  sensor_angles.push_back(+M_PI / 3);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  Robot r1("R2D2", 10, Point(WORLD_HEIGHT/2, WORLD_WIDTH/2), M_PI/4, sensor_angles, sensor_distances);


  // 6. let user select target locations
  //    then try to get the robot there
  exit_simulation = false;
  while (!exit_simulation)
  {
      Point target_location = select_new_target_location(world, r1);

      r1.set_target_location( target_location );

      simulate_one_episode(world, r1, sensor_angles, sensor_distances);

      exit_simulation = false;
  }  

} // main