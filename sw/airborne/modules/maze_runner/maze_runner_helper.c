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
 * @file "modules/maze_runner/maze_runner_helper.c"
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

/**
 * global variables, only shared within this file
 */
static int stage = 0;

/**
 * implement functions
 */
void ctrl_backend_init(void){}
void ctrl_backend_run(struct cmd_t *cmd, struct EnuCoor_f *goal, struct mav_state_t *mav, struct cv_info_t *cv)
{
    float err_mag = cv->lmag - cv->rmag;
    float err_eof = cv->leof - cv->reof;
    float err_x = goal->x - mav->pos_enu.x;
    float err_y = goal->y - mav->pos_enu.y;
    float mav_heading = mav->rpy_ned.psi;
    float wp_heading = atan2(err_x, err_y);
    if(mav_heading < 0)
    {
        mav_heading += 2 * M_PI;
    }
    if(wp_heading < 0)
    {
        wp_heading += 2 * M_PI;
    }
    float err_wp = wp_heading - mav_heading;
    if(err_wp > M_PI)
    {
        err_wp -= 2 * M_PI;
    }
    else if(err_wp < -M_PI)
    {
        err_wp += 2 * M_PI;
    }
    switch (stage) {
        case 0:
            cmd->body_vel_x = 0;
            cmd->body_vel_y = 0;
            cmd->yaw_rate = 0.5 * err_wp;
            if(fabs(err_wp) < 0.2)
            {
                stage = 1;
            }
            break;
        case 1:
            cmd->body_vel_x = 0.3;
            cmd->body_vel_y = 0;
            cmd->yaw_rate = 0.5 * err_wp;
            if(fabs(err_x) < 0.5 && fabs(err_y) < 0.5)
            {
                stage = 2;
            }
            break;
        case 2:
            cmd->body_vel_x = 0;
            cmd->body_vel_y = 0;
            cmd->yaw_rate = 0;
            if(fabs(err_wp) > 0.2 || fabs(err_x) > 0.5 || fabs(err_y) > 0.5)
            {
                stage = 0;
            }
            break;
    }

}