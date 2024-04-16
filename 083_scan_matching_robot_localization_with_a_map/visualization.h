#pragma once


#include "opencv2/opencv.hpp"

using namespace cv;


// maps a value in [0,1] to a color
// using a heat map coding:
// blue --> white --> yellow --> green --> red
Vec3b map_prob_to_color(float prob)
{
   // 1a. define heat map from cold to hot:
   //    white -> blue -> cyan -> green -> yellow -> red
   /*
   const int NUM_Colors = 6;
   const float HeatMapColors[NUM_Colors][3] =
   { { 255,255,255 },{ 0,0,255 },{ 0,255,255 },{ 0,255,0 },{ 255,255,0 },{ 255,0,0 } };
   */

   // 1b. define heat map from cold to hot:
   //    white -> blue -> cyan -> green -> yellow -> red   
   const int NUM_Colors = 3;
   const float HeatMapColors[NUM_Colors][3] =
      { { 0,255,0 },{ 255,255,0 },{ 255,0,0 } };
   
   // 2. make sure, prob is not larger than 1.0
   if (prob > 1.0)
      prob = 1.0;

   // 3. compute bottom and top color in heat map
   prob *= (NUM_Colors - 1);
   int   idx1 = (int)floor(prob);
   int   idx2 = idx1 + 1;
   float fract = prob - (float)idx1; // where are we between the two colors?

   // 4. compute some intermediate color between HeatMapColors[idx2] and HeatMapColors[idx1]                                    
   float R = (HeatMapColors[idx2][0] - HeatMapColors[idx1][0])*fract + HeatMapColors[idx1][0];
   float G = (HeatMapColors[idx2][1] - HeatMapColors[idx1][1])*fract + HeatMapColors[idx1][1];
   float B = (HeatMapColors[idx2][2] - HeatMapColors[idx1][2])*fract + HeatMapColors[idx1][2];

   // 5. set (R,G,B) values in a color / Vec3b object
   Vec3b col;
   col.val[0] = (int)B;
   col.val[1] = (int)G;
   col.val[2] = (int)R;

   // 6. your heat map color is ready!
   return col;

} // map_prob_to_color
