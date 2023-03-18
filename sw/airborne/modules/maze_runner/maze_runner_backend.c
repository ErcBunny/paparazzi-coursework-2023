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
#define LITTLE_STOP 8

#define PRINT(string, ...) fprintf(stderr, "[maze_runner->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

/**
 * global variables, only shared within this file
 */

// variables with derivative info
static struct var_t flow_delta_mag = {0, 0}, flow_delta_eof = {0, 0}, flow_sum_eof = {0, 0};
static struct var_t goal_heading_err = {0, 0}, goal_dst_err = {0, 0}, tmp_heading_err = {0, 0}, tmp_dst_err = {0, 0};

// flight area and waypoints
static struct zone_t geofence;
static struct EnuCoor_f last_goal = {0, 0, 0}, tmp_wp = {0, 0, 0}, last_tmp_wp[2];

// state machine variables
static int action = STAND_BY;
static int loop_cnt = 0;

// state machine params
static int accel_cnt_thresh, back_cnt_thresh, rest_cnt_thresh;
static float wp_hit_radius, heading_hit_thresh, stop_sum_eof_thresh, turn_delta_mag_thresh;
static float fwd_vel, back_vel;
static float tmp_wp_dst;

// low pass filter param
static float flow_delta_mag_lpf_tau, flow_delta_eof_lpf_tau, flow_sum_eof_lpf_tau, loop_period;

// PD ctrl param
static float pd_ctrl_p, pd_ctrl_d;

/**
 * implement functions
 */
void ctrl_backend_init(
    struct zone_t *zone,
    float acc_thresh, float bac_thresh, float res_thresh,
    float wp_hit_r, float heading_hit, float stop_seof_thresh, float trun_dmag_thresh,
    float v_fwd, float v_back,
    float lpf_t_dmag, float lpf_t_deof, float lpf_t_seof, float period,
    float tmp_wp_dist,
    float pd_p, float pd_d)
{
    memcpy(&geofence, zone, sizeof(*zone));
    last_tmp_wp[0].x = last_tmp_wp[0].y = 0;
    last_tmp_wp[1].x = last_tmp_wp[1].y = 0;

    accel_cnt_thresh = acc_thresh;
    back_cnt_thresh = bac_thresh;
    rest_cnt_thresh = res_thresh;
    wp_hit_radius = wp_hit_r;
    heading_hit_thresh = heading_hit;
    stop_sum_eof_thresh = stop_seof_thresh;
    turn_delta_mag_thresh = trun_dmag_thresh;
    fwd_vel = v_fwd;
    back_vel = v_back;
    flow_delta_mag_lpf_tau = lpf_t_dmag;
    flow_delta_eof_lpf_tau = lpf_t_deof;
    flow_sum_eof_lpf_tau = lpf_t_seof;
    loop_period = period;

    tmp_wp_dst = tmp_wp_dist;

    pd_ctrl_p = pd_p;
    pd_ctrl_d = pd_d;
}

void ctrl_backend_run(
    struct cmd_t *cmd, struct dbg_msg_t *dbg,
    struct EnuCoor_f *goal, struct mav_state_t *mav, struct cv_info_t *cv,
    bool is_guided)
{
    // goal wp xy err
    float goal_err_x = goal->x - mav->pos_enu.x;
    float goal_err_y = goal->y - mav->pos_enu.y;
    float goal_dst_err_new = sqrt(pow(goal_err_x, 2) + pow(goal_err_y, 2));
    goal_dst_err.dx = goal_dst_err_new - goal_dst_err.x;
    goal_dst_err.x = goal_dst_err_new;

    // goal heading err
    float goal_heading_err_new = get_wp_heading_err(goal_err_x, goal_err_y, mav);
    goal_heading_err.dx = goal_heading_err_new - goal_heading_err.x;
    goal_heading_err.x = goal_heading_err_new;

    // tmp wp xy err
    float tmp_err_x = tmp_wp.x - mav->pos_enu.x;
    float tmp_err_y = tmp_wp.y - mav->pos_enu.y;
    float tmp_dst_err_new = sqrt(pow(tmp_err_x, 2) + pow(tmp_err_y, 2));
    tmp_dst_err.dx = tmp_dst_err_new - tmp_dst_err.x;
    tmp_dst_err.x = tmp_dst_err_new;

    // tmp heading err
    float tmp_heading_err_new = get_wp_heading_err(tmp_err_x, tmp_err_y, mav);
    tmp_heading_err.dx = tmp_heading_err_new - tmp_heading_err.x;
    tmp_heading_err.x = tmp_heading_err_new;

    // store cv info after lpf
    low_pass_filter(&flow_delta_mag, cv->lmag - cv->rmag, loop_period / (flow_delta_mag_lpf_tau + loop_period));
    low_pass_filter(&flow_delta_eof, cv->leof - cv->reof, loop_period / (flow_delta_eof_lpf_tau + loop_period));
    low_pass_filter(&flow_sum_eof, cv->leof + cv->reof, loop_period / (flow_sum_eof_lpf_tau + loop_period));

    // state machine
    switch (action)
    {
    case STAND_BY:
    default:
        set_cmd(cmd, 0, 0, 0);
        if (goal_dst_err.x > wp_hit_radius)
        {
            action = TURN_TO_GOAL;
        }
        break;
    case TURN_TO_GOAL:
        set_cmd(cmd, 0, 0, pd_ctrl(&goal_heading_err, pd_ctrl_p, pd_ctrl_d));
        if (fabs(goal_heading_err.x) < heading_hit_thresh)
        {
            action = ACCEL_TO_GOAL;
            loop_cnt = 0;
        }
        break;
    case ACCEL_TO_GOAL:
        set_cmd(cmd, fwd_vel, 0, pd_ctrl(&goal_heading_err, pd_ctrl_p, pd_ctrl_d));
        if (loop_cnt > accel_cnt_thresh)
        {
            action = GO_TO_GOAL;
        }
        if (goal->x != last_goal.x || goal->y != last_goal.y)
        {
            action = TURN_TO_GOAL;
        }
        loop_cnt += 1;
        break;
    case GO_TO_GOAL:
        set_cmd(cmd, fwd_vel, 0, pd_ctrl(&goal_heading_err, pd_ctrl_p, pd_ctrl_d));
        if (flow_sum_eof.x > stop_sum_eof_thresh)
        {
            update_tmp_wp(mav);
            action = BACK_UP;
            loop_cnt = 0;
        }
        if (goal_dst_err.x <= wp_hit_radius)
        {
            action = STAND_BY;
        }
        if (goal->x != last_goal.x || goal->y != last_goal.y)
        {
            action = TURN_TO_GOAL;
        }
        break;
    case BACK_UP:
        if (loop_cnt < back_cnt_thresh)
        {
            set_cmd(cmd, back_vel, 0, 0);
        }
        else if (loop_cnt >= back_cnt_thresh && loop_cnt < back_cnt_thresh + accel_cnt_thresh)
        {
            set_cmd(cmd, 0, 0, 0);
        }
        else
        {
            action = TURN_TO_TMP;
        }
        if (goal->x != last_goal.x || goal->y != last_goal.y)
        {
            action = TURN_TO_GOAL;
        }
        loop_cnt += 1;
        break;
    case TURN_TO_TMP:
        set_cmd(cmd, 0, 0, pd_ctrl(&tmp_heading_err, pd_ctrl_p, pd_ctrl_d));
        if (fabs(tmp_heading_err.x) < heading_hit_thresh)
        {
            action = ACCEL_TO_TMP;
            loop_cnt = 0;
        }
        if (goal->x != last_goal.x || goal->y != last_goal.y)
        {
            action = TURN_TO_GOAL;
        }
        break;
    case ACCEL_TO_TMP:
        set_cmd(cmd, fwd_vel, 0, pd_ctrl(&tmp_heading_err, pd_ctrl_p, pd_ctrl_d));
        if (loop_cnt > accel_cnt_thresh)
        {
            action = GO_TO_TMP;
        }
        if (goal->x != last_goal.x || goal->y != last_goal.y)
        {
            action = TURN_TO_GOAL;
        }
        loop_cnt += 1;
        break;
    case GO_TO_TMP:
        set_cmd(cmd, fwd_vel, 0, 0);
        if (flow_sum_eof.x > stop_sum_eof_thresh)
        {
            update_tmp_wp(mav);
            action = BACK_UP;
            loop_cnt = 0;
        }
        if (tmp_dst_err.x <= wp_hit_radius)
        {
            action = LITTLE_STOP;
            loop_cnt = 0;
        }
        if (goal->x != last_goal.x || goal->y != last_goal.y)
        {
            action = TURN_TO_GOAL;
        }
        break;
    case LITTLE_STOP:
        set_cmd(cmd, 0, 0, 0);
        if (loop_cnt > rest_cnt_thresh)
        {
            action = TURN_TO_GOAL;
        }
        if (goal->x != last_goal.x || goal->y != last_goal.y)
        {
            action = TURN_TO_GOAL;
        }
        loop_cnt += 1;
        break;
    }

    if (!is_guided)
    {
        action = STAND_BY;
        set_cmd(cmd, 0, 0, 0);
    }

    last_goal.x = goal->x;
    last_goal.y = goal->y;

    // dbg msg
    dbg->fps = cv->fps;
    dbg->dmag = cv->lmag - cv->rmag;
    dbg->deof = cv->leof - cv->reof;
    dbg->seof = cv->leof + cv->reof;
    dbg->dmag_lpf = flow_delta_mag.x;
    dbg->deof_lpf = flow_delta_eof.x;
    dbg->seof_lpf = flow_sum_eof.x;
}

float get_wp_heading_err(float err_x, float err_y, struct mav_state_t *mav)
{
    float mav_heading = mav->rpy_ned.psi;
    float wp_heading = atan2(err_x, err_y);
    if (mav_heading < 0)
    {
        mav_heading += 2 * M_PI;
    }
    if (wp_heading < 0)
    {
        wp_heading += 2 * M_PI;
    }
    float err = wp_heading - mav_heading;
    if (err > M_PI)
    {
        err -= 2 * M_PI;
    }
    else if (err < -M_PI)
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
    if (*x > max)
    {
        *x = max;
    }
    if (*x < min)
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

void update_tmp_wp(struct mav_state_t *mav)
{
    // propose tmp wps
    float mav_heading = mav->rpy_ned.psi;
    float mav_x = mav->pos_enu.x;
    float mav_y = mav->pos_enu.y;
    struct EnuCoor_f left_wp, right_wp, back_wp;
    left_wp.x = mav_x - tmp_wp_dst * cos(-mav_heading);
    left_wp.y = mav_y - tmp_wp_dst * sin(-mav_heading);
    right_wp.x = mav_x + tmp_wp_dst * cos(-mav_heading);
    right_wp.y = mav_y + tmp_wp_dst * sin(-mav_heading);
    back_wp.x = mav_x + tmp_wp_dst * sin(-mav_heading);
    back_wp.y = mav_y - tmp_wp_dst * sin(-mav_heading);

    // decide which proposal to use
    if (is_inside_zone(&left_wp) && !is_inside_zone(&right_wp))
    {
        tmp_wp.x = left_wp.x;
        tmp_wp.y = left_wp.y;
    }
    if (!is_inside_zone(&left_wp) && is_inside_zone(&right_wp))
    {
        tmp_wp.x = right_wp.x;
        tmp_wp.y = right_wp.y;
    }
    if (!is_inside_zone(&left_wp) && !is_inside_zone(&right_wp))
    {
        tmp_wp.x = back_wp.x;
        tmp_wp.y = back_wp.y;
    }
    if (is_inside_zone(&left_wp) && is_inside_zone(&right_wp))
    {
        if (sqrt(pow(left_wp.x - last_tmp_wp[0].x, 2) + pow(left_wp.y - last_tmp_wp[0].y, 2)) < wp_hit_radius)
        {
            tmp_wp.x = right_wp.x;
            tmp_wp.y = right_wp.y;
        }
        else if (sqrt(pow(right_wp.x - last_tmp_wp[0].x, 2) + pow(right_wp.y - last_tmp_wp[0].y, 2)) < wp_hit_radius)
        {
            tmp_wp.x = left_wp.x;
            tmp_wp.y = left_wp.y;
        }
        else
        {
            if (fabs(flow_delta_mag.x) > turn_delta_mag_thresh)
            {
                if (flow_delta_mag.x > 0)
                {
                    tmp_wp.x = right_wp.x;
                    tmp_wp.y = right_wp.y;
                }
                else
                {
                    tmp_wp.x = left_wp.x;
                    tmp_wp.y = left_wp.y;
                }
            }
            else
            {
                if (flow_delta_eof.x > 0)
                {
                    tmp_wp.x = right_wp.x;
                    tmp_wp.y = right_wp.y;
                }
                else
                {
                    tmp_wp.x = left_wp.x;
                    tmp_wp.y = left_wp.y;
                }
            }
        }
    }
    last_tmp_wp[0].x = last_tmp_wp[1].x;
    last_tmp_wp[0].y = last_tmp_wp[1].y;
    last_tmp_wp[1].x = tmp_wp.x;
    last_tmp_wp[1].y = tmp_wp.y;
    
}

bool is_inside_zone(struct EnuCoor_f *point)
{
    float x, y, x0, y0, x1, y1, x2, y2;
    x = point->x;
    y = point->y;
    x0 = geofence.corner[0].x;
    y0 = geofence.corner[0].y;
    x1 = geofence.corner[1].x;
    y1 = geofence.corner[1].y;
    x2 = geofence.corner[2].x;
    y2 = geofence.corner[2].y;

    float AB_dot_AM = (x1 - x0) * (x - x0) + (y1 - y0) * (y - y0);
    float AB_dot_AB = (x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0);
    float BC_dot_BM = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
    float BC_dot_BC = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    return (0.0 <= AB_dot_AM && AB_dot_AM <= AB_dot_AB && 0.0 <= BC_dot_BM && BC_dot_BM <= BC_dot_BC);
}
