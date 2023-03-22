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
 * @file "modules/computer_vision/cv_maze_runner.h"
 * @author Ryan Y. Liu <yueqianliu@outlook.com>
 */

#ifndef CV_MAZE_RUNNER_H
#define CV_MAZE_RUNNER_H

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/cv.h"
#include "pthread.h"
#include <time.h>
#include <stdio.h>

#include "modules/computer_vision/opencv_maze_runner.h"

#define CV_MAZE_RUNNER_VERBOSE TRUE

#define CB_STATE_FIRST_RUN 0
#define CB_STATE_SECOND_RUN 1
#define CB_STATE_INITIALIZED 2

extern void cv_maze_runner_init(void);

struct image_t *video_cb(struct image_t *img, uint8_t camera_id);
void *frontend_thread(void *args);

#endif