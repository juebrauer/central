/// Demo: Robot localization GIVEN a map
///
/// Task:
/// -----
/// The goal of this demo is to show students how
/// a robot that wakes up (is switched on)
/// and does not have a localization sensor can localize.
///
/// The robot has to localize itself on the map just using
/// its distance sensor information, an orientation sensor
/// and a map of the world.
///
///
/// Key idea:
/// ---------
/// The key idea followed in this situation is to compare
/// the sensor readings we would get at hypothetical
/// positions of the robot in the map with the real sensor
/// readings ("scan-matching" approach)
///
///
/// Issue #1: Robustness against sensor noise
/// -----------------------------------------
/// However, if sensor readings are noisy we need to introduce
/// some tolerance, i.e., will still consider a position p
/// in the map as a candidate, even if the actual sensor readings
/// we would get at this hypothetical position are slightly
/// different from the actual current noisy sensor readings.
///
/// 
/// Issue #2: Speed
/// ---------------
/// "Trick" a) for acceleration:
/// In order to accelerate the search we do not check
/// all positions in the map.
/// Since sensor readings are noisy anyway, we therefore
/// consider neighboured positions p' of a position candidate p
/// as possible candidates as well.
/// 
/// "Trick" b) for acceleration:
/// We further use an approach where we reject position candidates
/// at early time steps, when we see that the sensor readings are
/// already too different to be a promising candidate.
/// --> see method   bool Robot::check_hypothetical_position()
///
///
/// Experiment with the following parameters in params.h:
/// -----------------------------------------------------
/// --> MAX_DIFFERENCE_SUM = 10,30
/// --> SIMULATE_NOISY_DISTANCE_SENSORS =false,true
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
#include "params.h"



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
   printf("Robot is put to the target location (%d, %d)\n",
           selected_target_pos.x, selected_target_pos.y);

   int simulation_step = 0;
   Mat image;
   bool exit_episode = false;
   bool pause_simulation = false;
   while (!exit_episode)
   {
      if (!pause_simulation) {
         // 1. initialize image with world image
         world.copyTo(image);


         // 2. move the robot according to its specific behavior
         r1.update(world);


         // 3. show the robot's position as a circle and its orientation by a line
         Point2d pos = r1.get_position();
         double  ori = r1.get_orientation();
         double  dx = cos(ori);
         double  dy = sin(ori);
         double  r = r1.get_radius();
         circle(image, pos, (int)r, CV_RGB(255, 0, 0), 2);
         line(image, pos, pos + Point2d(dx*r, dy*r), CV_RGB(0, 255, 0), 1);


         // 4. compute all the sensor values
         double* sensor_values = r1.get_sensor_values();


         // 5. for each sensor draw a sensor ray
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


         // 6. show simulation step nr
         char txt[100];
         sprintf(txt, "step %d", simulation_step);
         putText(image,
            txt,
            Point(20, 50),
            FONT_HERSHEY_SIMPLEX, 0.7, // font face and scale
            CV_RGB(255, 0, 0), // white
            1); // line thickness and type


         // 7. show world with robot, sensor rays and additional textual information
         imshow("Robot Simulator", image);


         // 8. save image?
         if (SAVE_IMAGES)
         {
            char fname[500];
            sprintf(fname, "%s/img%05d.png", SAVE_FOLDER, simulation_step);
            imwrite(fname, image);
         }

         // 9. one step simulated more
         simulation_step++;

      } // if pause_simulation


      // 10. wait for a key
      char c = (char)waitKey(10);


      // 11. ESC pressed? --> exit simulation
      if (c == 27)
         exit_episode = true;

      // 12. SPACE pressed? --> toggle pause simulation
      if (c==32)
         pause_simulation = !pause_simulation;
      

      
   } // while (episode shall continue)

} // simulate_one_episode



Point select_new_wake_up_location(Mat world, Robot r)
{
   printf("Where shall the robot wake up?\n");

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

   printf("User selected new robot wake up location (%d, %d)\n",
          selected_target_pos.x, selected_target_pos.y);
   
   return selected_target_pos;

} // select_new_wake_up_location



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
  sensor_angles.push_back(-M_PI / 4);
  sensor_angles.push_back(+M_PI / 4);  
  sensor_angles.push_back(-M_PI / 2);  
  sensor_angles.push_back(+M_PI / 2);  
  //sensor_angles.push_back(-M_PI / 8);
  //sensor_angles.push_back(+M_PI / 8);
  

  sensor_distances.push_back(200);
  sensor_distances.push_back(200);    
  sensor_distances.push_back(200);  
  sensor_distances.push_back(200);  
  //sensor_distances.push_back(200);
  //sensor_distances.push_back(200);
  

  Robot r1("R2D2",
           10,
           Point(WORLD_HEIGHT/2, WORLD_WIDTH/2),
           -M_PI,
           sensor_angles,
           sensor_distances,
           SIMULATE_NOISY_DISTANCE_SENSORS);


  // 6. let user select wake up location,
  //    then put robot there.
  //    The robot then starts moving from there
  //    and tries to localize itself on the map.
  exit_simulation = false;
  while (!exit_simulation)
  {
      Point wake_up_location = select_new_wake_up_location(world, r1);

      r1.set_wake_up_location(wake_up_location);

      simulate_one_episode(world, r1, sensor_angles, sensor_distances);

      r1.reset_position_belief_map();

      exit_simulation = false;
  }  

} // main