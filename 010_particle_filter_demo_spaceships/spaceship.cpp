#include "spaceship.h"

#include "params.h"

spaceship::spaceship(int nr_of_parts, Size world_size)
{
  // 1. store infos
  this->nr_of_parts = nr_of_parts;
  this->world_size  = world_size;

  // 2. initialize all parts
  Point next_part_location = Point(world_size.width / 2, world_size.height / 2);
  Point rnd_common_move_vec = get_rnd_move_vec();
  for (int part_nr = 0; part_nr < nr_of_parts; part_nr++)
  {
    // 2.1 create new part info object
    part_info* new_part = new part_info;
    new_part->location = next_part_location;
    new_part->move_vec = rnd_common_move_vec;

    // 2.2 store pointer to that new part info object
    part_infos.push_back( new_part );

    // 2.3 compute next part location
    next_part_location += Point(SPACESHIP_PART_SIZE_IN_PIXELS, 0);
  }

} // constructor of class spaceship



Point spaceship::get_rnd_move_vec()
{
   int rnd_x = rand() % 3 - 1; // -1,0,+1
   int rnd_y = rand() % 3 - 1; // -1,0,+1
   return Point(rnd_x, rnd_y);

} // get_rnd_move_vec



void spaceship::move()
{
  // 1. move all parts according to their move vectors
  for (int part_nr = 0; part_nr < nr_of_parts; part_nr++)
  {    
    // 1.1 move part
    part_infos[part_nr]->location += part_infos[part_nr]->move_vec;

    // 1.2 teleport?
    if ((SPACE_SHIPS_CAN_TELEPORT) && (rand() % 100 == 0))
    {
      part_infos[part_nr]->location.x = rand() % world_size.width;
      part_infos[part_nr]->location.y = rand() % world_size.height;
    }


    // 1.3 make sure, parts do not leave the 2D world
    if (part_infos[part_nr]->location.x - SPACESHIP_PART_SIZE_IN_PIXELS < 0)
    {
      part_infos[part_nr]->location.x = SPACESHIP_PART_SIZE_IN_PIXELS;
      part_infos[part_nr]->move_vec.x = +1;
    }
    if (part_infos[part_nr]->location.y - SPACESHIP_PART_SIZE_IN_PIXELS < 0)
    {
      part_infos[part_nr]->location.y = SPACESHIP_PART_SIZE_IN_PIXELS;
      part_infos[part_nr]->move_vec.y = +1;
    }
    if (part_infos[part_nr]->location.x + SPACESHIP_PART_SIZE_IN_PIXELS >= world_size.width)
    {
      part_infos[part_nr]->location.x = world_size.width - SPACESHIP_PART_SIZE_IN_PIXELS;
      part_infos[part_nr]->move_vec.x = -1;
    }
    if (part_infos[part_nr]->location.y + SPACESHIP_PART_SIZE_IN_PIXELS >= world_size.height)
    {
      part_infos[part_nr]->location.y = world_size.height - SPACESHIP_PART_SIZE_IN_PIXELS;
      part_infos[part_nr]->move_vec.y = -1;
    }

    // 1.4 compute new motion vector for this part?
    if (rand() % 250 == 0)
    {
      part_infos[part_nr]->move_vec = get_rnd_move_vec();
    }
    

  } // for (move all parts)
  


} // move


void spaceship::draw_yourself_into_this_image(Mat& img)
{
  // 1. draw each part individually
  for (int part_nr = 0; part_nr < nr_of_parts; part_nr++)
  {
    // 1.1 draw filled box
    Rect r(part_infos[part_nr]->location.x - SPACESHIP_PART_SIZE_IN_PIXELS/2,
           part_infos[part_nr]->location.y - SPACESHIP_PART_SIZE_IN_PIXELS/2,
           SPACESHIP_PART_SIZE_IN_PIXELS,
           SPACESHIP_PART_SIZE_IN_PIXELS);
    rectangle(img, r, CV_RGB(0, 255, 0), -1);

    // 1.2 draw rectangle around box
    rectangle(img, r, CV_RGB(200, 255, 0), 1);

  } // for (draw each part individually)

} // draw_yourself_into_this_image


vector<spaceship::part_info*> spaceship::get_part_info_vector()
{
  return part_infos;
}