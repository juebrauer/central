#pragma once

#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class Robot
{ 
  public:


    //////// generic robot methods:

                    Robot(string             name,
                          int                radius,
                          Point2d            start_pos,
                          double             start_orientation,
                          vector<double>     sensor_angles,
                          vector<double>     sensor_distances,
                          bool               noisy_sensors);

    void            compute_sensor_values(Mat& world,
                                          Point position,
                                          double* sensor_values,
                                          bool simulate_noisy_sensors);

    void            update(Mat world);

    Point2d         get_position();

    double          get_orientation();

    double          get_radius();

    int             get_nr_sensors();

    double*         get_sensor_values();

    vector<double>  get_sensor_angles();

    vector<double>  get_sensor_distances();

    void            move(double pixel);

    void            turn(double angle);

    bool            test_wall_bump(Mat world);

    double          get_distance_to_target();


    //////// methods related to localization:

    void            set_wake_up_location(Point p);

    void            create_position_belief_map(int W, int H);

    void            reset_position_belief_map();

    void            update_position_belief_map(Mat world);
    
    void            update_visualization_of_position_belief_map(Mat world);

    double          get_sensor_vec_distance(int N, double* vec1, double* vec2);
    
    bool            check_hypothetical_position(Mat& world,
                                                Point position,
                                                double* real_sensor_values,
                                                double max_allowed_difference);

    void            increase_belief_for_position(int x, int y, double inc);

    ////////



private:

   string          name;
   double          radius;
   Point2d         pos;
   double          orientation;
   vector<double>  sensor_angles;
   vector<double>  sensor_distances;
   int             nr_sensors;
   double*         sensor_values;
   bool            noisy_sensors;
   

   //////// new variables

   Point           wake_up_location;
   int             pbm_width;
   int             pbm_height;
   double**        position_belief_map;
   Mat             visu_position_belief_map;

   ////////


}; // class Robot
