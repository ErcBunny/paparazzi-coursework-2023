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

/** @file "modules/maze_runner/maze_runner.h"
 * @author Ryan Y. Liu <yueqianliu@outlook.com>
 * A module for AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module aims to provide a collision free navigation strategy for the Bebop in the Cyberzoo.
 * Cyberzoo == Maze, so bebop == maze runner :p
 */

#ifndef MAZE_RUNNER_H
#define MAZE_RUNNER_H

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <time.h>
#include <stdio.h>

#define NAV_C // needed to get the nav functions like Inside...

#define MAZE_RUNNER_VERBOSE TRUE
#ifndef MAZE_RUNNER_VISUAL_DETECTION_ID
#define MAZE_RUNNER_VISUAL_DETECTION_ID ABI_BROADCAST
#endif

enum navigation_state_t
{
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS
};

extern float oa_color_count_frac;
extern float heading_increment;

extern void maze_runner_init(void);
extern void maze_runner_loop(void);

uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters);
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
uint8_t increase_nav_heading(float incrementDegrees);
uint8_t chooseRandomIncrementAvoidance(void);

void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                        int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                        int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                        int32_t quality, int16_t __attribute__((unused)) extra);

#endif
