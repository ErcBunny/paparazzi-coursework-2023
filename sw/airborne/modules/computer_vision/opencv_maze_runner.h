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
 * @file "modules/computer_vision/opencv_maze_runner.h"
 * @author Ryan Y. Liu <yueqianliu@outlook.com>
 */

#ifndef OPENCV_MAZE_RUNNER_H
#define OPENCV_MAZE_RUNNER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

    struct opencv_frontend_return_t
    {
        float lmag;
        float rmag;
        float leof;
        float reof;
        int grad_sum;
    };

    void opencv_frontend_init(uint16_t src_h, uint16_t src_w, int of_method);

    void opencv_frontend_run(struct image_t *src_0, struct image_t *src_1);

    struct opencv_frontend_return_t opencv_frontend_return(void);

#ifdef __cplusplus
}
#endif

#endif