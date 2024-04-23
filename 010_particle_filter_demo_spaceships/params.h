#pragma once

// DEMO order:
//
// 1. show demo with code/parameters as is
//
// 2. Importance of particle diversity:
//    deactivate USE_NOISE_IN_PREDICTION_STEP
//    observe how important noise is in the prediction
//    step to keep diversity
//
// 3. Tracking in the case of heavy noise
//    --> Importance of resampling:
//    (i)   set NR_OF_COMPLETELY_WRONG_MEASUREMENTS from
//          1 to 20 --> observe demo
//    (ii)  set NR_OF_COMPLETELY_WRONG_MEASUREMENTS from
//          1 to 50 --> observe demo
//    (iii) set RESAMPLING on --> observe demo
//    (iv)  set POPULATION_SIZE from 1000 to 2000 --> observe
//
//    This is an impressive behavior of the particle filter and a
//    prototypical example of how sensor fusion over time using many
//    noisy measurements can help to estimate state vectors!
//   
//    Even in situations where we have two times more wrong than correct
//    (= : noisy, but near to real spaceship parts) measurements,
//    the particle filter algorithm allows us to estimate the spaceship
//    locations!
//
//    An important question is : why is this possible?
//    It is the interplay between the prediction and the
//    correction-by-measurement step and the more consistent behavior
//    of measurements near to real spaceship parts:
//    if a particle is near to one of the 20 random measurements,
//    it might follow it, but in the next step, it will follow with
//    a high chance another measurement.
//    In contrast, particles near to �real� (even if they are noisy)
//    measurements, will have a high chance to follow this spaceship
//    part, since a new noisy measurement will appear near to the
//    spaceship part in the next step. And the prediction step helps
//    to stay in the neighborhood of the spaceship part even if it
//    moves away, since we move the particle in the prediction step into
//    the direction of the assumed movement.
//
// 4. Particle relocation experiments
//    --> Importance of resampling
//    (i) set TEST_RELOCATION_OF_PARTICLES true
//        and RESAMPLING false
//   (ii) set TEST_RELOCATION_OF_PARTICLES true
//        and RESAMPLING true
//

// use OpenMP for speed up?
//#define USE_OPENMP

// wait for key press after each simulation step?
#define STEP_BY_STEP 1


// important for keeping up diversity in population
#define USE_NOISE_IN_PREDICTION_STEP true


// how many measurements do we want to generate for one
// spacehship part?
#define NR_MEASUREMENTS_PER_SPACESHIP_PART 1

// if switched on, we will not only generate
// noisy measurements near to spaceship parts,
// but also add completely wrong measurements
#define SIMULATE_COMPLETELY_WRONG_MEASUREMENTS true

// how many of these "completely wrong measurements"
// to add?
#define NR_OF_COMPLETELY_WRONG_MEASUREMENTS 10


// how many particles do we want to use to model
// the probability density function (pdf)?
#define POPULATION_SIZE 10000


// how many parts has the spacehship?
#define NR_SPACESHIP_PARTS 5


// teleport spaceships?
#define SPACE_SHIPS_CAN_TELEPORT false

// size of a single spaceship square
#define SPACESHIP_PART_SIZE_IN_PIXELS 30


// how fast shall a particle move to its nearest
// measurement?
#define MOVE_TO_MEASUREMENT_SPEED 0.05f



// do prediction step?
#define DO_PREDICTION_STEP true

// do measurement correction step?
#define DO_MEASUREMENT_CORRECTION_STEP true


// test how fast the particles can "warp" to a new
// location in state space?
#define TEST_RELOCATION_OF_PARTICLES false


// decides whether to use or not to use resampling
// of particles
#define RESAMPLING false


// speed-up computation of continuous density
// using a LUT (look-up table) for exp() function values
#define USE_LUT_FOR_EXP true // note! does not work with KDE bandwidths < 1

// speed-up computation of continuous density
// function by computing density only at each n-th position
#define SPEED_UP_KDE_STEPSIZE 2


// always compute & show particle clusters?
#define ALWAYS_CLUSTER_PARTICLES 0


// tolerance for cluster building
#define TOLERANCE_CLUSTER_BUILDING 5

#define CYLINDER_KERNEL_RADIUS 50

#define CONVERGENCE_THRESHOLD_MEAN_SHIFT_VEC_LEN 1
