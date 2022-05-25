/// file: mdp_simulator.cpp
/// 
/// my solution for the following exercise:
///
/// Mobile Robots - Exercise:
/// Markov Decision Process(MDP) and Value Iteration simulator
///
/// In the lecture you have seen a simple example of a
/// Markov Decision Process : a 4x3 grid world.
/// We further discussed a simple algorithm that computes
/// a utility for each state of this grid world
/// - Value Iteration - which can be used to directly infer
/// the optimal policy for a given grid world /
/// Markov Decision Problem.
///
/// Write a simple simulator that reads in a grid world
/// description from a text file. For each grid cell the
/// text file shall specify whether we can move to the
/// cell or not and which reward we will get if we move to
/// this cell. Also specify "epsiode end" cells in the text
/// file, i.e.in the example in the lecture these were the
/// special fields "goal" (with reward + 1) and "trap"
/// (with reward - 1) which terminated the current episode.
///
/// After reading in the description of the grid world,
/// the simulator shall execute the value iteration algorithm
/// till convergence. Let the user observe the utility for each
/// grid cell after each Value Iteration update step.
///
/// Experiment with different rewards for the grid cells and
/// develop an intuition how value iteration leads to utilities
/// of the grid cells(states) such that we can use them for
/// finding the optimal policy.
///
/// After utlity iteration has converged: visualize the optimal
/// action for each state, i.e., visualize the optimal policy
/// for the grid world.
///
/// ---
/// by Prof. Dr. Juergen Brauer, www.juergenbrauer.org


#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#define SAVE_IMAGES 0
#define SAVE_FOLDER "~/tmp"

#include <iostream>
#include <math.h>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

enum cell_types {FREE, WALL, ABSORBING};
enum actions {LEFT, TOP, RIGHT, DOWN};

double      DISCOUNTING_FACTOR;
int         width, height;
int**       cell_types;
double**    reward_function;
double**    old_utility_function;
double**    utility_function;
int**       optimal_policy;

void read_gridworld_file(string fname)
{
   // 1. open file
   printf("reading in grid world from file %s\n", fname.c_str());
   std::ifstream f( fname );

   // 2. read discounting factor
   f >> DISCOUNTING_FACTOR;
   printf("\tdiscounting factor = %.2f\n", DISCOUNTING_FACTOR);
      
   // 3. read grid dimensions
   f >> width >> height;
   printf("\tgrid world has dimensions %d x %d\n", width, height);
   
   // 4. read cell types
   //    0: free cell
   //    1: wall
   //    2: absorbing state (state will terminated episode)
   cell_types = new int*[height];
   for (int y = 0; y < height; y++)
      cell_types[y] = new int[width];

   printf("\n\tCell types:\n");
   for (int y = 0; y < height; y++)
   {
      printf("\t");
      for (int x = 0; x < width; x++)
      {
         f >> cell_types[y][x];
         printf("%d ", cell_types[y][x]);
      }
      printf("\n");
   }

   // 5. read in reward function r:S->R
   reward_function = new double*[height];
   for (int y = 0; y < height; y++)
      reward_function[y] = new double[width];

   printf("\n\tRewards per state:\n");
   for (int y = 0; y < height; y++)
   {
      printf("\t");
      for (int x = 0; x < width; x++)
      {
         f >> reward_function[y][x];
         printf("%.1f ", reward_function[y][x]);
      }
      printf("\n");
   }

   // 6. prepare utility per state array
   //    utility function U:S->R
   utility_function = new double*[height];
   old_utility_function = new double*[height];
   for (int y = 0; y < height; y++)
   {
      utility_function[y]     = new double[width];
      old_utility_function[y] = new double[width];
      for (int x = 0; x < width; x++)
      {
         utility_function[y][x] = 0.0;
         //utility_function[y][x] = (rand()%200/100.0) - 1.0;
      }
   }

   // 7. set utility of absorbing states to
   //    their rewards
   for (int y = 0; y < height; y++)
   {
      for (int x = 0; x < width; x++)
      {
         if (cell_types[y][x] == ABSORBING)
            utility_function[y][x] = reward_function[y][x];
      }
   }

   // 8. prepare array for optimal policy
   //    and initialize it with some action
   optimal_policy = new int*[height];
   for (int y = 0; y < height; y++)
   {
      optimal_policy[y] = new int[width];
      for (int x = 0; x < width; x++)
      {
         optimal_policy[y][x] = LEFT;
      }
   }
   
} // read_gridworld_file



string action_to_string(int a)
{
   switch (a)
   {
      case LEFT : return "LEFT";
      case TOP  : return "UP";
      case RIGHT: return "RIGHT";
      case DOWN : return "DOWN";
      default   : return "unknown action";
   }
} // action_to_string 



void show_gridworld(int cellsize=150)
{
   int img_width  = width*cellsize;
   int img_height = height*cellsize;
   Mat visu(img_height, img_width, CV_8UC3);
   visu = 0;

   for (int gy = 0; gy < height; gy++)
   {
      for (int gx = 0; gx < width; gx++)
      {
         Vec3b col;
         switch (cell_types[gy][gx])
         {
            case FREE:      col = Vec3b(255, 255, 255); break; // free --> white                               
            case WALL:      col = Vec3b(0, 0, 0); break;       // wall --> black
            case ABSORBING: col = Vec3b(0, 255, 0); break;     // terminate --> green
         }

         int x1 = gx*cellsize;
         int y1 = gy*cellsize;
         rectangle(visu, Rect(x1 + 1, y1 + 1, cellsize - 2, cellsize - 2),
            col, -1);

         char txt[100];
         double font_size = ((double)cellsize/150.0)*0.7;
         int text_size = 1;
         if (cellsize > 100)
            text_size = 2;

         sprintf(txt, "r=%.2f", reward_function[gy][gx]);
         if (reward_function[gy][gx] != 0)
            col = Vec3b(0, 0, 255); // red
         else
            col = Vec3b(0, 0, 0);
         putText(visu,
            txt,
            Point(x1 + 5, y1 + 20),
            FONT_HERSHEY_SIMPLEX, font_size, // font face and scale
            col,
            text_size); // line thickness and type

         sprintf(txt, "U=%.5f", utility_function[gy][gx]);
         putText(visu,
            txt,
            Point(x1 + 5, y1 + 40),
            FONT_HERSHEY_SIMPLEX, font_size, // font face and scale
            CV_RGB(0, 0, 0), // white
            text_size); // line thickness and type

         if (cell_types[gy][gx] == FREE)
         {
            int a = optimal_policy[gy][gx];
            sprintf(txt, "A=%s", action_to_string(a).c_str());
            putText(visu,
               txt,
               Point(x1 + 5, y1 + 60),
               FONT_HERSHEY_SIMPLEX, font_size, // font face and scale
               CV_RGB(0, 0, 0), // white
               text_size); // line thickness and type
         }

      } // for (gx)
   } // for (gy)

   imshow("grid world", visu);
   waitKey(0);

} // show_gridworld


///
/// for a given start position and an offset
/// vector return the target position
/// 
Point2i get_successor(Point2i start, Point2i offset)
{
   // 1. compute successor grid cell position s
   Point2i s = start + offset;

   // 2. is the successor state cell position
   //    within the grid world dimensions?
   if ((s.x < 0)      || (s.y < 0) ||
       (s.x >= width) || (s.y >= height))
      return start; // no, successor cell position is invalid

   // 3. yes, successor state cell position is valid
   //    is it a wall?
   if (cell_types[s.y][s.x] == WALL)
      return start; // yes, s is a wall. So we will keep at start position

   // 4. successor state cell position s is
   //    within grid world dimensions AND is not a wall
   return s;

} // get_successor



///
/// for a given start position and action a
/// compute a list of possible successor states
/// with their corresponding probabilities
/// (note: we assume a STOCHASTIC world!)
///
/// note: the same state s can appear more than once
///       in the list!
///
void get_list_of_possible_successor_states(Point2i start,
                                           int a,
                                           vector<Point2i>& successors,
                                           vector<double>& probs)
{
   successors.clear();
   probs.clear();

   // absorbing state?
   if (cell_types[start.y][start.x] == ABSORBING)
   {
      // we cannot get out of an absorbing state!
      successors.push_back(start);
      probs.push_back(1.0);
      return;
   }

   switch (a)
   {
      case actions::LEFT:
         successors.push_back(get_successor(start, Point2i(-1, 0)));
         probs.push_back(0.8);
         successors.push_back(get_successor(start, Point2i(0, +1)));
         probs.push_back(0.1);
         successors.push_back(get_successor(start, Point2i(0, -1)));
         probs.push_back(0.1);
         break;

      case actions::TOP:
         successors.push_back(get_successor(start, Point2i(0, -1)));
         probs.push_back(0.8);
         successors.push_back(get_successor(start, Point2i(-1, 0)));
         probs.push_back(0.1);
         successors.push_back(get_successor(start, Point2i(+1, 0)));
         probs.push_back(0.1);
         break;

      case actions::RIGHT:
         successors.push_back(get_successor(start, Point2i(+1, 0)));
         probs.push_back(0.8);
         successors.push_back(get_successor(start, Point2i(0, -1)));
         probs.push_back(0.1);
         successors.push_back(get_successor(start, Point2i(0, +1)));
         probs.push_back(0.1);
         break;

      case actions::DOWN:
         successors.push_back(get_successor(start, Point2i(0, +1)));
         probs.push_back(0.8);
         successors.push_back(get_successor(start, Point2i(+1, 0)));
         probs.push_back(0.1);
         successors.push_back(get_successor(start, Point2i(-1, 0)));
         probs.push_back(0.1);
         break;

   } // switch (action)

} // get_list_of_possible_successor_states



void compute_best_action_that_maximizes_expected_future_reward(
         Point2i start,
         int& best_action,
         double& max_future_reward)
{
   // 1. try out all possible actions and find action,
   //    that maximizes the expected reward in the future
   max_future_reward = 0.0;
   for (int a = actions::LEFT; a <= actions::DOWN; a++)
   {
      // 1.1 find out in which states we could land if
      //     we take action a
      vector<Point2i> successors;
      vector<double>  probs;
      get_list_of_possible_successor_states(Point2i(start.x, start.y), a,
         successors, probs);

      // 1.2 what is the expected future reward of this action a?
      double future_reward = 0.0;
      for (uint i = 0; i < successors.size(); i++)
      {
         Point2i s_prime = successors[i];
         double utility_s_prime = old_utility_function[s_prime.y][s_prime.x];
         double prob_to_land_in_s_prime = probs[i];
         future_reward += prob_to_land_in_s_prime * utility_s_prime;
      }

      // 1.3 did we found a "better" action a?
      if ((a == actions::LEFT) || (future_reward>max_future_reward))
      {
         max_future_reward = future_reward;
         best_action = a;
      }

   } // for (all possible actions)

} // compute_best_action_that_maximizes_expected_future_reward



void value_iteration()
{
   // 1. make a copy of the current utility values
   for (int gy = 0; gy < height; gy++)
      for (int gx = 0; gx < width; gx++)
         old_utility_function[gy][gx] = utility_function[gy][gx];

   // 2. for all cells ...
   for (int gy = 0; gy < height; gy++)
   {
      for (int gx = 0; gx < width; gx++)
      {
         // 2.1 is it a free cell?
         //     if not, continue with next cell
         if (cell_types[gy][gx] != 0)
            continue;

         // it is a free cell, so compute utility for this cell now:

         // 2.2 get reward for this cell
         double reward = reward_function[gy][gx];

         // 2.3 get max future reward we can get with best action
         int a;
         double max_future_reward = 0.0;
         compute_best_action_that_maximizes_expected_future_reward(
            Point2i(gx, gy),
            a,
            max_future_reward);

         // 2.4 compute new utility value and store it
         double new_utility = reward + DISCOUNTING_FACTOR * max_future_reward;
         utility_function[gy][gx] = new_utility;

      } // for (gx)
   } // for (gy)

} // value_iteration


void compute_optimal_policy_from_utility_values()
{
   // 1. for all cells ...
   for (int gy = 0; gy < height; gy++)
   {
      for (int gx = 0; gx < width; gx++)
      {
         // 1.1 compute best action we can take
         int a;
         double max_future_reward = 0.0;
         compute_best_action_that_maximizes_expected_future_reward(
            Point2i(gx, gy),
            a,
            max_future_reward);

         // 1.2 store best action
         optimal_policy[gy][gx] = a;
      } // for (gx)
   } // for (gy)

} // compute_optimal_policy_from_utility_values





int main()
{
   srand((unsigned int) time(NULL));

   //read_gridworld_file("grid_worlds/4x3_gridworld.txt");
   read_gridworld_file("grid_worlds/10x10_gridworld.txt");

   bool converged = false;
   int iteration_step = 0;
   while (!converged)
   {
      printf("iteration step: %d\n", iteration_step);

      compute_optimal_policy_from_utility_values();

      //show_gridworld(150);
      show_gridworld(80);

      value_iteration();

      iteration_step++;
   }

   printf("Simulation finished. Press any key to exit.\n");   
}