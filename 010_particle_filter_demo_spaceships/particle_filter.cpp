#include "particle_filter.h"
#include <omp.h>

///
///  generate a particle filter with
///  given nr of particles
///  and state space dimension
///
particle_filter::particle_filter(int population_size, int state_space_dimension)
{
  // 1. save infos about nr of particles to generate
  //    & dimension of the state space
  this->population_size       = population_size;
  this->state_space_dimension = state_space_dimension;


  // 2. create the desired number of particles
  for (int i = 0; i<population_size; i++)
  {
    // 2.1 create new particle object
    particle* p = new particle();

    // 2.2 create vector for storing the particles location
    //     in state space
    p->state = new float[state_space_dimension];

    // 2.3 set initial weight of this particle
    p->weight = 1.0f / (float)population_size;

    // 2.4 store pointer to this particle
    all_particles.push_back(p);
  }


  // 3. prepare arrays for minimum and maximum coordinates
  //    of each state space dimension
  min_values  = new float[state_space_dimension];
  max_values  = new float[state_space_dimension];
  range_sizes = new float[state_space_dimension];


  // 4. we have no motion and perception model yet  
  your_prediction_model = nullptr;
  your_perception_model = nullptr;


  // 5. start locations of particles not yet set!
  start_locations_initalized = false;


  // 6. no update steps done so far
  nr_update_steps_done = 0;


  // 7. helper data structure for Monte Carlo step
  segmentation_of_unit_interval = new float[population_size + 1];

} // constructor of class particle_filter


///
/// free memory allocated when destroying
/// this particle filter object
///
particle_filter::~particle_filter()
{
  // 1. give back memory allocated for each particle
  for (int i = 0; i<population_size; i++)
  {
    // 1.1 get next particle
    particle* p = all_particles[i];

    // 1.2 give back memory allocated to store location
    delete[] p->state;

    // 1.3 free memory allocated for object (members)
    delete p;
  }


  // 2. give back memory to store minimum and maximum
  //    values per state space dimension
  delete[] min_values;
  delete[] max_values;
  delete[] range_sizes;

  // 3. give back memory used for helper array
  delete[] segmentation_of_unit_interval;

} // destructor of class particle_filter


///
/// method to set pointer to user data
/// needed to access in motion or perception model
///
void particle_filter::set_userdata(void* ptr_user_data)
{
  this->ptr_user_data = ptr_user_data;
}



///
/// reset positions & weights of all particles
/// to start conditions
///
void particle_filter::reset_positions_and_weights()
{
  // 1. reset positions
  set_random_start_states_for_all_particles();


  // 2. reset weights
  for (int i = 0; i<population_size; i++)
  {
    // 2.1 get next particle
    particle* p = all_particles[i];

    // 2.2 reset weight
    p->weight = 1.0f / (float)population_size;
  }

} // reset_positions_and_weights



///
/// should be used by the user to specify in which range [min_value,max_value]
/// the <param_nr>-th parameter of the state space lives
///
void particle_filter::set_param_ranges(int param_nr, float min_value, float max_value)
{
  min_values[param_nr]  = min_value;
  max_values[param_nr]  = max_value;
  range_sizes[param_nr] = max_value - min_value;

} // set_param_ranges


///
/// for the specified particle we guess a random start location
///
void particle_filter::set_random_start_state_for_specified_particle(particle* p)
{
  for (int d = 0; d<state_space_dimension; d++)
  {
    float rnd_start_value = get_rnd_from_interval(min_values[d], max_values[d]);
    p->state[d] = rnd_start_value;
  }

} // set_random_start_state_for_specified_particle



///
/// set initial location in state space for all particles
///
void particle_filter::set_random_start_states_for_all_particles()
{
  for (int i = 0; i<population_size; i++)
  {
    particle* p = all_particles[i];

    set_random_start_state_for_specified_particle(p);
  }

} // set_random_start_states_for_all_particles


///
/// inform the particle filter which motion model we want to use
///
void particle_filter::set_prediction_model(particle_filter_update_model* m)
{
  this->your_prediction_model = m;

} // set_prediction_model



///
/// inform the particle filter which perception model we want to use
///
void particle_filter::set_perception_model(particle_filter_update_model* m)
{
  this->your_perception_model = m;

} // set_prediction_model


///
/// returns a copy of an existing particle
/// the particle to be copied is chosen according to
/// a probability that is proportional to its importance weight
///
particle* particle_filter::sample_one_particle_according_to_importance_weight_pdf()
{
  // 1. guess a random number from [0,1]
  float rndVal = get_rnd_from_interval(0, 1);

  // 2. to which particle does the interval segment belong
  //    in which this random number lies?
  int idx = -1;
  for (unsigned int i = 0; i <= all_particles.size() - 1; i++)
  {
    // 2.1 get next segment of partition of unit interval
    float a = segmentation_of_unit_interval[i];
    float b = segmentation_of_unit_interval[i + 1];

    // 2.2 does the rndVal lie in the interval [a,b] of [0,1] that belongs to particle i?
    if ((rndVal >= a) && (rndVal <= b))
    {
      // yes, it does!
      idx = i;
      break;
    } // if

  } // for
  if (idx == -1)
    idx = (int)all_particles.size() - 1;

  // 3. particle with index <idx> has won! we will resample this particle for the next iteration!
  particle* winner_particle = all_particles[idx];


  // 4. return a _similar_ 'copy' of that particle

  // 4.1 generate new copy particle
  particle* particle_copy = new particle();
  particle_copy->state = new float[state_space_dimension];

  // 4.2 copy location of that particle in state space
  for (int d = 0; d<state_space_dimension; d++)
    particle_copy->state[d] = winner_particle->state[d];

  // 4.3 copy shall be similar, not 100% identical
  for (int d = 0; d<state_space_dimension; d++)
  {
    float value_range = 0.01f * range_sizes[d];
    particle_copy->state[d] += get_rnd_from_interval(-value_range,+value_range);
  }
   
  // 4.4 weight is reset to 1/N
  particle_copy->weight = 1.0f / (float)population_size;


  // 5. return particle copy
  return particle_copy;

} // sample_one_particle_according_to_importance_weight_pdf



///
/// one particle filter update step
///
void particle_filter::update()
{
  // 1. did the user specify a motion and a perception update model?
  if (your_prediction_model == nullptr)
    return;
  if (your_perception_model == nullptr)
    return;


  // 2. set initial particle locations?
  if (!start_locations_initalized)
  {
    set_random_start_states_for_all_particles();
    start_locations_initalized = true;
  }


  // 3. update each particle
  // #pragma omp parallel for
  for (unsigned int i = 0; i<all_particles.size(); i++)
  {
    // 3.1 get next particle
    particle* p = all_particles[i];

    // 3.2 move that particle according to prediction
    if (DO_PREDICTION_STEP)
       your_prediction_model->update_particle(p);

    // 3.3 move that particle according to measurement
    if (DO_MEASUREMENT_CORRECTION_STEP)
      your_perception_model->update_particle(p);    

    // 3.4 make sure, particles do not leave state space!
    for (int d = 0; d<state_space_dimension; d++)
    {
      if (p->state[d] < min_values[d]) p->state[d] = min_values[d];
      if (p->state[d] > max_values[d]) p->state[d] = max_values[d];
    }
  } // for (all particles to be updated)


  // 4. normalize importance weights
    
  // 4.1 compute sum of all weights
  float sum_weights = 0.0f;
  for (unsigned int i = 0; i<all_particles.size(); i++)
  {
    particle* p = all_particles[i];
    sum_weights += p->weight;
  }

  // 4.2 normalize each particle weight
  for (unsigned int i = 0; i<all_particles.size(); i++)
  {    
    particle* p = all_particles[i];
    p->weight /= sum_weights;
  }


  // 5. resample complete particle population based on
  //    current importance weights of current particles?
  if (RESAMPLING)
  { 
    // 5.1 compute division of unit interval [0,1]
    //     such that each particle gets a piece of that interval
    //     where the length of the interval is proportional to its importance weight
    float next_border = 0.0f;
    segmentation_of_unit_interval[0] = 0.0f;
    for (int i = 0; i < population_size; i++)
    {
      // get next particle
      particle* p = all_particles[i];

      // compute next border
      next_border += p->weight;

      // compute next border in unit interval
      segmentation_of_unit_interval[i + 1] = next_border;
    }

    // 5.2 generate new particle population
    std::vector<particle*> new_population;
    for (unsigned int i = 0; i < all_particles.size(); i++)
    {
      particle* p = sample_one_particle_according_to_importance_weight_pdf();
      new_population.push_back(p);
    }

    // 5.3 delete old population
    for (unsigned int i = 0; i < all_particles.size(); i++)
    {
      particle* p = all_particles[i];
      delete[] p->state;
      delete p;
    }

    // 5.4 set new sample population as current population
    all_particles = new_population;
  }


  // 6. find particle with highest weight / highest prob
  particle_with_highest_weight = nullptr;
  for (unsigned int i = 0; i<all_particles.size(); i++)
  {
    particle* p = all_particles[i];
    if ((particle_with_highest_weight == nullptr) || (p->weight > particle_with_highest_weight->weight))
    {
      particle_with_highest_weight = p;
    }
  }


  // 7. one particle filter update step done more!
  nr_update_steps_done++;

} // update