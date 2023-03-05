/*
 * Copyright (C) 2023 Ryan Y. Liu <yueqianliu@outlook.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/maze_runner/maze_runner.c"
 * @author Ryan Y. Liu <yueqianliu@outlook.com>
 * A module for AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module aims to provide a collision free navigation strategy for the Bebop in the Cyberzoo.
 * Cyberzoo == Maze, so bebop == maze runner :p
 */

#include "modules/maze_runner/maze_runner.h"
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[maze_runner->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

float oa_color_count_frac = 0.18f;
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;
int32_t color_count = 0;                     // orange color count from color filter for obstacle detection
int16_t obstacle_free_confidence = 0;        // a measure of how certain we are that the way ahead is safe.
float heading_increment = 5.f;               // heading angle increment [deg]
float maxDistance = 2.25;                    // max waypoint displacement [m]
const int16_t max_trajectory_confidence = 5; // number of consecutive negative object detections to be sure we are obstacle free
static abi_event color_detection_ev;

void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                        int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                        int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                        int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

void maze_runner_init(void)
{
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(MAZE_RUNNER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
}

void maze_runner_loop(void)
{
  // loop frequency set in the module xml
  // only evaluate our state machine if we are flying
  if (!autopilot_in_flight())
  {
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oa_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;

  // VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);

  // update our safe confidence using color threshold
  if (color_count < color_count_threshold)
  {
    obstacle_free_confidence++;
  }
  else
  {
    obstacle_free_confidence -= 2; // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float moveDistance = fminf(maxDistance, 0.2f * obstacle_free_confidence);

  switch (navigation_state)
  {
  case SAFE:
    // Move waypoint forward
    moveWaypointForward(WP_TRAJECTORY, 1.5f * moveDistance);
    if (!InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY)))
    {
      navigation_state = OUT_OF_BOUNDS;
    }
    else if (obstacle_free_confidence == 0)
    {
      navigation_state = OBSTACLE_FOUND;
    }
    else
    {
      moveWaypointForward(WP_GOAL, moveDistance);
    }

    break;
  case OBSTACLE_FOUND:
    // stop
    waypoint_move_here_2d(WP_GOAL);
    waypoint_move_here_2d(WP_TRAJECTORY);

    // randomly select new search direction
    chooseRandomIncrementAvoidance();

    navigation_state = SEARCH_FOR_SAFE_HEADING;

    break;
  case SEARCH_FOR_SAFE_HEADING:
    increase_nav_heading(heading_increment);

    // make sure we have a couple of good readings before declaring the way safe
    if (obstacle_free_confidence >= 2)
    {
      navigation_state = SAFE;
    }
    break;
  case OUT_OF_BOUNDS:
    increase_nav_heading(heading_increment);
    moveWaypointForward(WP_TRAJECTORY, 1.5f);

    if (InsideObstacleZone(WaypointX(WP_TRAJECTORY), WaypointY(WP_TRAJECTORY)))
    {
      // add offset to head back into arena
      increase_nav_heading(heading_increment);

      // reset safe counter
      obstacle_free_confidence = 0;

      // ensure direction is safe before continuing
      navigation_state = SEARCH_FOR_SAFE_HEADING;
    }
    break;
  default:
    break;
  }
  return;
}
