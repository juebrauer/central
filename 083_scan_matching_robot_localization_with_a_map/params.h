#pragma once

// Experiment order:
// (MAX_DIFFERENCE_SUM 10, SIMULATE_NOISY_DISTANCE_SENSORS false)
// (MAX_DIFFERENCE_SUM 10, SIMULATE_NOISY_DISTANCE_SENSORS true)
// (MAX_DIFFERENCE_SUM 30, SIMULATE_NOISY_DISTANCE_SENSORS true)
// fewer sensors

// Important threshold for a
// Scan-Match algorithm:
// Up to which "difference" do we consider
// two scans as similar?
#define MAX_DIFFERENCE_SUM 10

#define BELIEF_DECAY_FACTOR 0.95

#define PARAM_MATCH_STEP 5

#define SIMULATE_NOISY_DISTANCE_SENSORS true
#define RESIZE_BELIEF_MAP_VISU false