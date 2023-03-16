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
static struct cmd_t cmd;
static pthread_mutex_t mtx_cv_info;
static pthread_mutex_t mtx_dbg_msg;

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

    ctrl_backend_init(&zone);
    AbiBindMsgCV_MAZE_RUNNER(0, &cv_ev, cv_cb);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_MAZE_RUNNER, telem_cb);
}

void maze_runner_loop(void)
{
    // control output at the front to ensure constant frequency
    if (guidance_h.mode == GUIDANCE_H_MODE_GUIDED)
    {
        guidance_h_set_body_vel(cmd.body_vel_x, cmd.body_vel_y);
        guidance_h_set_heading_rate(cmd.yaw_rate);
    }

    // fill dbg_msg
    pthread_mutex_lock(&mtx_dbg_msg);
    memcpy(&dbg_msg, &dbg_msg_ctrl, sizeof(dbg_msg_ctrl));
    pthread_mutex_unlock(&mtx_dbg_msg);

    // get copy of shared resource
    pthread_mutex_lock(&mtx_cv_info);
    memcpy(&cv_info_cpy, &cv_info, sizeof(cv_info));
    pthread_mutex_unlock(&mtx_cv_info);

    // get mav state
    struct mav_state_t mav;
    mav.pos_enu = *stateGetPositionEnu_f();
    mav.vel_enu = *stateGetSpeedEnu_f();
    mav.rpy_ned = *stateGetNedToBodyEulers_f();
    mav.ang_vel_body = *stateGetBodyRates_f();

    // get goal waypoint
    struct EnuCoor_f goal_wp;
    goal_wp.x = waypoint_get_x(WP_GUIDED_GOAL);
    goal_wp.y = waypoint_get_y(WP_GUIDED_GOAL);
    goal_wp.z = waypoint_get_alt(WP_GUIDED_GOAL);

    // calculate control output, which will be used asap in the next loop
    ctrl_backend_run(&cmd, &dbg_msg_ctrl, &goal_wp, &mav, &cv_info_cpy);

    return;
}
