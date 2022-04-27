#pragma once

#include <stdlib.h>

///
/// draw a random number from an interval [a,b]
/// uniformly
///
static float get_rnd_from_interval(float a, float b)
{
  // compute range length, e.g. a=-1, b=2 --> range = 3
  float range = b - a;

  // get random number from [0,1]
  float rnd_val1 = (float)rand() / (float)RAND_MAX;

  // e.g. random number in [0,3]
  float rnd_val_in_range = rnd_val1 * range;

  // e.g. random number in [-1,2]
  float final_rnd_val = a + rnd_val_in_range;

  return final_rnd_val;

} // get_rnd_from_interval