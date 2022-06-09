#pragma once

// try (10,10), (20,10), (10,20)
#define NOISE_PROCESS     10.0
#define NOISE_MEASUREMENT 10.0


#define VISUALIZATION true

#define IMG_WIDTH  800

#define IMG_HEIGHT 800

#define LINE_WIDTH 3

#define COL_TIME_AXIS      CV_RGB(255,255,255)   // WHITE
#define COL_GT_POS         CV_RGB(255,255,255)   // WHITE
#define COL_NAIVE_EST_POS  CV_RGB(255,255,0)     // YELLOW
#define COL_MEASUREMENT    CV_RGB(255,0,0)       // RED
#define COL_KF_EST_POS     CV_RGB(0,255,0)       // GREEN
#define COL_KF_UNCERTAINTY CV_RGB(0,255,255)     // CYAN
