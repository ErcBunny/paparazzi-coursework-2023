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
#include "modules/datalink/telemetry.h"
#define NAV_C // needed to get the nav functions like Inside...
#include "generated/flight_plan.h"

#define PRINT(string, ...) fprintf(stderr, "[maze_runner->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

/**
 * additional function declarations
 */
void telem_cb(struct transport_tx *trans, struct link_device *dev);

/**
 * global var, shared within this file
 */
static abi_event cv_ev;
static struct cv_info_t cv_info, cv_info_cpy;
static struct dbg_msg_t dbg_msg, dbg_msg_cpy, dbg_msg_ctrl;
static struct cmd_t cmd = {0, 0, 0};
static pthread_mutex_t mtx_cv_info;
static pthread_mutex_t mtx_dbg_msg;

static bool is_auto_gen_wp = false, is_wp_cnt_even = true, first_autogen = true;
static float zone_radius = 0;
static float auto_wp_angle = 0, auto_wp_angle_delta;
static int go_wp_timeout_loop_cnt = 0;
static int go_wp_timeout_loop_cnt_thresh = 0;
static float regen_wp_dst_thresh = 0;
static int regen_wp_loop_interval = 0, regen_loop_cnt = 0;
static struct mav_state_t mav;
static struct EnuCoor_f goal_wp;
static uint8_t goal_wpid = WP_GUIDED_GOAL;
/**
 * implement functions
 */
void cv_cb(uint8_t __attribute__((unused)) sender_id,
           float left_flow_mag,
           float right_flow_mag,
           float left_flow_eof,
           float right_flow_eof,
           int fps)
{
    pthread_mutex_lock(&mtx_cv_info);
    cv_info.lmag = left_flow_mag;
    cv_info.rmag = right_flow_mag;
    cv_info.leof = left_flow_eof;
    cv_info.reof = right_flow_eof;
    cv_info.fps = fps;
    pthread_mutex_unlock(&mtx_cv_info);
}

void telem_cb(struct transport_tx *trans, struct link_device *dev)
{
    pthread_mutex_lock(&mtx_dbg_msg);
    memcpy(&dbg_msg_cpy, &dbg_msg, sizeof(dbg_msg));
    pthread_mutex_unlock(&mtx_dbg_msg);
    pprz_msg_send_MAZE_RUNNER(
        trans, dev, AC_ID,
        &dbg_msg_cpy.fps,
        &dbg_msg_cpy.dmag, &dbg_msg_cpy.deof, &dbg_msg_cpy.seof,
        &dbg_msg_cpy.dmag_lpf, &dbg_msg_cpy.deof_lpf, &dbg_msg_cpy.seof_lpf);
}

void maze_runner_init(void)
{
    struct zone_t zone;

    zone.corner[0].x = waypoint_get_x(WP__OZ1);
    zone.corner[0].y = waypoint_get_y(WP__OZ1);
    zone.corner[0].z = waypoint_get_alt(WP__OZ1);

    zone.corner[1].x = waypoint_get_x(WP__OZ2);
    zone.corner[1].y = waypoint_get_y(WP__OZ2);
    zone.corner[1].z = waypoint_get_alt(WP__OZ2);

    zone.corner[2].x = waypoint_get_x(WP__OZ3);
    zone.corner[2].y = waypoint_get_y(WP__OZ3);
    zone.corner[2].z = waypoint_get_alt(WP__OZ3);

    zone.corner[3].x = waypoint_get_x(WP__OZ4);
    zone.corner[3].y = waypoint_get_y(WP__OZ4);
    zone.corner[3].z = waypoint_get_alt(WP__OZ4);

    zone_radius = fmin(
                      0.5 * sqrt(pow(zone.corner[0].x - zone.corner[1].x, 2) + pow(zone.corner[0].y - zone.corner[1].y, 2)),
                      0.5 * sqrt(pow(zone.corner[1].x - zone.corner[2].x, 2) + pow(zone.corner[1].y - zone.corner[2].y, 2))) -
                  0.5;
    auto_wp_angle_delta = 1.0;
    go_wp_timeout_loop_cnt_thresh = (int)(3 * (2 * zone_radius / 0.4) / 0.02);
    regen_wp_dst_thresh = 0.1;
    regen_wp_loop_interval = 50;

    ctrl_backend_init(
        &zone,
        50, 30, 25,
        0.5, 0.1, 225, 900,
        0.4, -1.0,
        1.0, 1.0, 0.0, 0.02,
        1.0,
        1.0, 0.02);
    AbiBindMsgCV_MAZE_RUNNER(0, &cv_ev, cv_cb);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MAZE_RUNNER, telem_cb);
}

void maze_runner_loop(void)
{
    // control output at the front to ensure constant frequency
    guidance_h_set_body_vel(cmd.body_vel_x, cmd.body_vel_y);
    guidance_h_set_heading_rate(cmd.yaw_rate);

    // fill dbg_msg
    pthread_mutex_lock(&mtx_dbg_msg);
    memcpy(&dbg_msg, &dbg_msg_ctrl, sizeof(dbg_msg_ctrl));
    pthread_mutex_unlock(&mtx_dbg_msg);

    // get copy of shared resource
    pthread_mutex_lock(&mtx_cv_info);
    memcpy(&cv_info_cpy, &cv_info, sizeof(cv_info));
    pthread_mutex_unlock(&mtx_cv_info);

    // get mav state
    mav.pos_enu = *stateGetPositionEnu_f();
    mav.vel_enu = *stateGetSpeedEnu_f();
    mav.rpy_ned = *stateGetNedToBodyEulers_f();
    mav.ang_vel_body = *stateGetBodyRates_f();

    // if auto generate waypoint, move WP_GUIDED_GOAL
    if (is_auto_gen_wp)
    {
        float prev_x = waypoint_get_x(WP_GUIDED_GOAL);
        float prev_y = waypoint_get_y(WP_GUIDED_GOAL);
        struct EnuCoor_f new_wp;
        new_wp.z = waypoint_get_alt(WP_GUIDED_GOAL);

        if (pow(mav.pos_enu.x - prev_x, 2) + pow(mav.pos_enu.y - prev_y, 2) < regen_wp_dst_thresh)
        {
            regen_loop_cnt += 1;
        }
        if (go_wp_timeout_loop_cnt > go_wp_timeout_loop_cnt_thresh || regen_loop_cnt > regen_wp_loop_interval || first_autogen)
        {
            if (is_wp_cnt_even)
            {
                new_wp.x = zone_radius * cos(auto_wp_angle);
                new_wp.y = zone_radius * sin(auto_wp_angle);
            }
            else
            {
                new_wp.x = zone_radius * cos(auto_wp_angle + M_PI);
                new_wp.y = zone_radius * sin(auto_wp_angle + M_PI);
                auto_wp_angle += auto_wp_angle_delta;
                if (auto_wp_angle > 2 * M_PI)
                {
                    auto_wp_angle -= 2 * M_PI;
                }
            }
            waypoint_set_enu(WP_GUIDED_GOAL, &new_wp);
            DOWNLINK_SEND_WP_MOVED_ENU(
                DefaultChannel, DefaultDevice, &goal_wpid,
                &waypoints[WP_GUIDED_GOAL].enu_i.x,
                &waypoints[WP_GUIDED_GOAL].enu_i.y,
                &waypoints[WP_GUIDED_GOAL].enu_i.z);

            regen_loop_cnt = 0;
            go_wp_timeout_loop_cnt = 0;
            is_wp_cnt_even = !is_wp_cnt_even;
            first_autogen = false;
        }
        go_wp_timeout_loop_cnt += 1;
    }
    goal_wp.x = waypoint_get_x(WP_GUIDED_GOAL);
    goal_wp.y = waypoint_get_y(WP_GUIDED_GOAL);
    goal_wp.z = waypoint_get_alt(WP_GUIDED_GOAL);

    // calculate control output, which will be used asap in the next loop
    ctrl_backend_run(
        &cmd, &dbg_msg_ctrl,
        &goal_wp, &mav, &cv_info_cpy,
        guidance_h.mode == GUIDANCE_H_MODE_GUIDED);

    return;
}

void toggle_auto_gen_wp()
{
    // assignment is atomic, no need for lock
    is_auto_gen_wp = !is_auto_gen_wp;
    if (is_auto_gen_wp)
    {
        VERBOSE_PRINT("start using random waypoints\n");
    }
    else
    {
        VERBOSE_PRINT("stpp using random waypoints\n");
    }
}