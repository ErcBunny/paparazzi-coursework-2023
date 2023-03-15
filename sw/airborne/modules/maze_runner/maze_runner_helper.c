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
static struct var_t err_mag, err_eof, err_wp, err_dst, sum_eof;
static float p_wp, d_wp, p_mag, d_mag, p_dst, d_dst, p_eof, d_eof;
static float alpha;
static struct nav_weights_t nav_weights;
static float eof_thresh_0 = 150;
static float eof_thresh_1 = 400;
static struct EnuCoor_f last_goal;

/**
 * implement functions
 */
void ctrl_backend_init(void)
{
    err_mag.x = err_mag.dx = 0;
    err_eof.x = err_eof.dx = 0;
    err_wp.x = err_wp.dx= 0;
    err_dst.x = err_dst.dx = 0;
    sum_eof.x = sum_eof.dx = 0;
    nav_weights.wp = 1;
    nav_weights.mag = 0;
    nav_weights.eof = 0;

    last_goal.x = last_goal.y = 0;

    p_wp = 1;
    d_wp = 0.05;
    alpha = 0.05 / (0 + 0.05);
    p_mag = 0.0001;
    d_mag = 0.00;
    p_dst = 1;
    d_dst = 0.1;
    p_eof = 0.1;
    d_eof = 0.01;
}

void ctrl_backend_run(struct cmd_t *cmd, struct dbg_msg_t *dbg, struct EnuCoor_f *goal, struct mav_state_t *mav, struct cv_info_t *cv)
{
    // xy err
    float err_x = goal->x - mav->pos_enu.x;
    float err_y = goal->y - mav->pos_enu.y;
    float err_dst_new = sqrt(pow(err_x, 2) + pow(err_y, 2));
    err_dst.dx = err_dst_new - err_dst.x;
    err_dst.x = err_dst_new;

    // wp heading err
    float err_wp_new = get_wp_err(err_x, err_y, mav);
    err_wp.dx = err_wp_new - err_wp.x;
    err_wp.x = err_wp_new;

    // left right optflow mag err
    low_pass_filter(&err_mag, cv->lmag - cv->rmag, alpha);

    // left right optflow eof err
    low_pass_filter(&err_eof, cv->leof - cv->reof, alpha);

    // sum optflow eof
    low_pass_filter(&sum_eof, cv->leof + cv->reof, alpha);

    // calculate weights for different nav modes
    sigmoid_nav_weight(&nav_weights, &sum_eof, eof_thresh_0, eof_thresh_1, 20);


    // state machine
    switch (stage) {
        case 0:
            cmd->body_vel_x = 0;
            cmd->body_vel_y = 0;
            cmd->yaw_rate = pd_ctrl(&err_wp, p_wp, d_wp);

            if(fabs(err_wp.x) < 0.1)
            {
                stage = 1;
            }
            break;
        case 1:
            cmd->body_vel_x = 0.4;
            cmd->body_vel_y = 0;
            cmd->yaw_rate = pd_ctrl(&err_wp, p_wp, d_wp);// + pd_ctrl(&err_mag, p_mag, d_mag);
            constrain(&cmd->yaw_rate, -1, 1);

            if(fabs(err_dst.x) < 0.5)
            {
                stage = 2;
            }

            break;
        case 2:
            cmd->body_vel_x = 0;
            cmd->body_vel_y = 0;
            cmd->yaw_rate = 0;

            if(fabs(err_dst.x) > 1 && (last_goal.x != goal->x || last_goal.y != goal->y))
            {
                stage = 0;
            }
            break;
    }
    last_goal.x = goal->x;
    last_goal.y = goal->y;

    // dbg msg
    dbg->fps = cv->fps;
    dbg->lmag = cv->lmag;
    dbg->rmag = cv->rmag;
    dbg->leof = cv->leof;
    dbg->reof = cv->reof;
    dbg->dmag = cv->lmag - cv->rmag;
    dbg->deof = cv->leof - cv->reof;
    dbg->dmag_lpf = err_mag.x;
    dbg->deof_lpf = err_eof.x;
    dbg->eof_sum = cv->leof + cv->reof;
}

float get_wp_err(float err_x, float err_y, struct mav_state_t *mav)
{
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
    return err_wp;
}

void low_pass_filter(struct var_t *var, float input, float alpha)
{
    float new_x = alpha * input + (1 - alpha) * var->x;
    var->dx = new_x - var->x;
    var->x = new_x;
}

void sigmoid_nav_weight(struct nav_weights_t *weights, struct var_t *eof_sum, float eof_sum_thresh_0, float eof_sum_thresh_1, float switch_rate)
{
    weights->wp = 1.0 / (1 + exp(switch_rate * (eof_sum->x - eof_sum_thresh_0)));
    weights->mag = 1.0 / (1 + exp(switch_rate * (eof_sum->x - eof_sum_thresh_1)));
    weights->eof = 1.0 / (1 + exp(-switch_rate * (eof_sum->x - eof_sum_thresh_1)));
}

void constrain(float *x, float min, float max)
{
    if(*x > max)
    {
        *x = max;
    }
    if(*x < min)
    {
        *x = min;
    }
}

float pd_ctrl(struct var_t *var, float p, float d)
{
    return var->x * p + var->dx * d;
}
