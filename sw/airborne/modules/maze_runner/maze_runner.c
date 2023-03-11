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

/**
 * @file "modules/maze_runner/maze_runner.c"
 * @author Ryan Y. Liu <yueqianliu@outlook.com>
 * A module for AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module aims to provide a collision free navigation strategy for the Bebop in the Cyberzoo.
 * Cyberzoo == Maze, so bebop == maze runner :p
 */

#include "modules/maze_runner/maze_runner.h"

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
static abi_event cv_ev;

void cv_cb(uint8_t __attribute__((unused)) sender_id,
           float left_flow_mag,
           float right_flow_mag,
           float left_flow_eof,
           float right_flow_eof,
           int fps)
{
    VERBOSE_PRINT("[%d, %d], [%d, %d], %d\n", (int)left_flow_mag, (int)right_flow_mag, (int)left_flow_eof, (int)right_flow_eof, fps);
}

void maze_runner_init(void)
{
    AbiBindMsgCV_MAZE_RUNNER(0, &cv_ev, cv_cb);
}

void maze_runner_loop(void)
{
//   loop frequency set in the module xml
//   only evaluate our state machine if we are flying
   if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED)
   {
     return;
   }

   guidance_h_set_body_vel(0.1, 0);
   guidance_h_set_heading_rate(0.1);

  return;
}
