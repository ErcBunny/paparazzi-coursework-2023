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
 * @file "modules/maze_runner/maze_runner_backend.c"
 * @author Ryan Y. Liu <yueqianliu@outlook.com>
 * A module for AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module aims to provide a collision free navigation strategy for the Bebop in the Cyberzoo.
 * Cyberzoo == Maze, so bebop == maze runner :p
 */

#include "modules/maze_runner/maze_runner.h"

#define STAND_BY 0
#define TURN_TO_GOAL 1
#define ACCEL_TO_GOAL 2
#define GO_TO_GOAL 3
#define BACK_UP 4
#define TURN_TO_TMP 5
#define ACCEL_TO_TMP 6
#define GO_TO_TMP 7

#define PRINT(string, ...) fprintf(stderr, "[maze_runner->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

/**
 * global variables, only shared within this file
 */
static struct zone_t *geofence;
static struct EnuCoor_f last_goal, tmp_wp;
static struct var_t err_mag, err_eof, err_wp, err_dst, sum_eof;
static float p_wp, d_wp;
static int action;
static int loop_cnt, accel_cnt_thresh, back_cnt_thresh;
static float wp_hit_radius, heading_diff_thresh, seof_thresh, fwd_vel, back_vel;
static float dmag_lpf_tau, deof_lpf_tau, seof_lpf_tau, loop_period;

/**
 * implement functions
 */
void ctrl_backend_init(struct zone_t *zone)
{
    geofence = zone;
    last_goal.x = last_goal.y = 0;

    err_mag.x = err_mag.dx = 0;
    err_eof.x = err_eof.dx = 0;
    err_wp.x = err_wp.dx= 0;
    err_dst.x = err_dst.dx = 0;
    sum_eof.x = sum_eof.dx = 0;

    p_wp = 1;
    d_wp = 0.05;

    action = STAND_BY;
    loop_cnt = 0;
    accel_cnt_thresh = 15;
    back_cnt_thresh = 10;
    wp_hit_radius = 0.5;
    heading_diff_thresh = 0.1;
    seof_thresh = 222;
    fwd_vel = 0.4;
    back_vel = -1;
    dmag_lpf_tau = 1;
    deof_lpf_tau = 1;
    seof_lpf_tau = 0;
    loop_period = 0.02;
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

    // store cv info after lpf
    low_pass_filter(&err_mag, cv->lmag - cv->rmag, loop_period / (dmag_lpf_tau + loop_period));
    low_pass_filter(&err_eof, cv->leof - cv->reof, loop_period / (deof_lpf_tau + loop_period));
    low_pass_filter(&sum_eof, cv->leof + cv->reof, loop_period / (seof_lpf_tau + loop_period));

    // state machine
    switch (action)
    {
        case STAND_BY:
        default:
            set_cmd(cmd, 0, 0, 0);
            if(err_dst.x > wp_hit_radius && (goal->x != last_goal.x || goal->y != last_goal.y)){action = TURN_TO_GOAL;}
            break;
        case TURN_TO_GOAL:
            set_cmd(cmd, 0, 0, pd_ctrl(&err_wp, p_wp, d_wp));
            if(fabs(err_wp.x) < heading_diff_thresh){action = ACCEL_TO_GOAL; loop_cnt = 0;}
            break;
        case ACCEL_TO_GOAL:
            set_cmd(cmd, fwd_vel, 0, pd_ctrl(&err_wp, p_wp, d_wp));
            if(loop_cnt > accel_cnt_thresh){action = GO_TO_GOAL;}
            loop_cnt += 1;
            break;
        case GO_TO_GOAL:
            set_cmd(cmd, fwd_vel, 0, pd_ctrl(&err_wp, p_wp, d_wp));
            if(sum_eof.x > seof_thresh)
            {
                action = BACK_UP;
                loop_cnt = 0;
            }
            if(err_dst.x <= wp_hit_radius){action = STAND_BY;}
            if(goal->x != last_goal.x || goal->y != last_goal.y){action = TURN_TO_GOAL;}
            break;
        case BACK_UP:
            set_cmd(cmd, back_vel, 0, pd_ctrl(&err_wp, p_wp, d_wp));
            if(loop_cnt > back_cnt_thresh){action = STAND_BY;}
            loop_cnt += 1;
            break;
        case TURN_TO_TMP:
            break;
        case ACCEL_TO_TMP:
            break;
        case GO_TO_TMP:
            break;
    }
    last_goal.x = goal->x;
    last_goal.y = goal->y;

    // dbg msg
    dbg->fps = cv->fps;
    dbg->dmag = cv->lmag - cv->rmag;
    dbg->deof = cv->leof - cv->reof;
    dbg->seof = cv->leof + cv->reof;
    dbg->dmag_lpf = err_mag.x;
    dbg->deof_lpf = err_eof.x;
    dbg->seof_lpf = sum_eof.x;
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
    float err = wp_heading - mav_heading;
    if(err > M_PI)
    {
        err -= 2 * M_PI;
    }
    else if(err < -M_PI)
    {
        err += 2 * M_PI;
    }
    return err;
}

void low_pass_filter(struct var_t *var, float input, float a)
{
    float new_x = a * input + (1 - a) * var->x;
    var->dx = new_x - var->x;
    var->x = new_x;
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

void set_cmd(struct cmd_t *cmd, float vx, float vy, float ang_vel)
{
    cmd->yaw_rate = ang_vel;
    cmd->body_vel_x = vx;
    cmd->body_vel_y = vy;
}

bool is_origin_lhs(struct mav_state_t *mav)
{
    float mav_heading = mav->rpy_ned.psi;
    float mav_x = mav->pos_enu.x;
    float mav_y = mav->pos_enu.y;
    float origin_to_mav_heading = atan2(mav_x, mav_y);
    if(origin_to_mav_heading < 0)
    {
        origin_to_mav_heading += 2 * M_PI;
    }
    if(origin_to_mav_heading < 0)
    {
        origin_to_mav_heading += 2 * M_PI;
    }
    float err = origin_to_mav_heading - mav_heading;
    if(err > M_PI)
    {
        err -= 2 * M_PI;
    }
    else if(err < -M_PI)
    {
        err += 2 * M_PI;
    }
    return (err > 0);
}