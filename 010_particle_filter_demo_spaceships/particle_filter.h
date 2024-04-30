/// particle_filter.h
///
/// A straightforward implementation
/// of the particle filter idea
///
/// A particle filter is a sample based approach
/// for recursive Bayesian filtering
/// The particles are a population based discrete
/// representation of a probability density function.
///
/// The filter recursively updates
///
///   - the particle locations according to a 
///     probabilistic motion model
///     (prediction update step)
///
///   - recomputes importance weights for each particle
///     (measurement update step)
///
///   - resamples the particles according to the current
///     pdf represented by the importance weights
///
/// ---
/// by Prof. Dr. J�rgen Brauer, www.juergenbrauer.org

#pragma once

#include "params.h"




#include <vector>         // for storing all the particles in a vector
#include "random_tools.h" // for get_rnd_from_interval()

#include <opencv2/core.hpp>



using namespace cv;
using namespace std;


///
/// for each particle we store its location in state space
/// and an importance weight
struct particle
{
  float* state;
  float  weight;
};



///
/// base class for motion & measurement update models
///
class particle_filter_update_model
{
public:

  particle_filter_update_model()
  {
  }

  ///
  /// update location or imporant weight of the specified particle
  ///
  virtual void update_particle(particle* p)
  {
    // default: no motion at all / no importance weight change
    // implement your motion model + perception model in an own subclass
  }

}; // particle_filter_update_model



///
/// represents a probability distribution using
/// a set of discrete particles
///
class particle_filter
{

public:
                         particle_filter(int population_size,
                                         int state_space_dimension);

                         ~particle_filter();

   void                  set_userdata(void* ptr_user_data);

   void                  reset_positions_and_weights();

   void                  set_param_ranges(int param_nr, float min_value, float max_value);

   void                  set_random_start_state_for_specified_particle(particle* p);

   void                  set_random_start_states_for_all_particles();

   void                  set_prediction_model(particle_filter_update_model* m);

   void                  set_perception_model(particle_filter_update_model* m);

   particle*             sample_one_particle_according_to_importance_weight_pdf();

   void                  update(vector<Mat>& measurements);
  

   bool									     start_locations_initalized;

   particle_filter_update_model*		  your_prediction_model;

   particle_filter_update_model*		  your_perception_model;

   std::vector<particle*>	           all_particles;

   int									     population_size;
 
   int									     state_space_dimension;

   float*								     min_values;

   float*								     max_values;

   float*							        range_sizes;

   float*							        segmentation_of_unit_interval;	

   int									     nr_update_steps_done;

   particle*						        particle_with_highest_weight;

   void*								        ptr_user_data;


}; // particle_filter