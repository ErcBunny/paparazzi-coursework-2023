/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/ctrl/target_pos.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Control a rotorcraft to follow at a defined distance from the target
 */

#include "target_pos.h"
#include <math.h>

#include "modules/datalink/telemetry.h"
#include "modules/core/abi.h"

// if the targetpos is meant for cyberzoo ( OUTDOOR FLIGTH -> CYBERZOO False )
#define CYBERZOO

// The timeout when receiving GPS messages from the ground in ms
#ifndef TARGET_POS_TIMEOUT
#define TARGET_POS_TIMEOUT 5000
#endif

// The timeout when recceiving an RTK gps message from the GPS
#ifndef TARGET_RTK_TIMEOUT
#define TARGET_RTK_TIMEOUT 1000
#endif

// landing platform location wrt orientation RTK-gps bar
// when bar is pointing forward on ship, 180 is further back on the ship
#ifndef TARGET_OFFSET_HEADING
#define TARGET_OFFSET_HEADING 180.0
#endif

// landing platform horizontal dist wrt RTK-gps module
#ifndef TARGET_OFFSET_DISTANCE
#define TARGET_OFFSET_DISTANCE 10.0
#endif

// landing platform horizontal dist wrt RTK-gps module
#ifndef TARGET_OFFSET_DISTANCE_CYBERZOO
#define TARGET_OFFSET_DISTANCE_CYBERZOO 1.0
#endif

// landing platform heigth wrt RTK-gps height
// positive is up
#ifndef TARGET_OFFSET_HEIGHT
#define TARGET_OFFSET_HEIGHT 5.0
#endif
//#define TARGET_OFFSET_HEIGHT -1.2

// landing platform heigth wrt RTK-gps height
// positive is up
#ifndef TARGET_OFFSET_HEIGHT_CYBERZOO
#define TARGET_OFFSET_HEIGHT_CYBERZOO 1.0
#endif

// calculate X & Y positions in between recieved gps coordinates
#ifndef TARGET_INTEGRATE_XY
#define TARGET_INTEGRATE_XY true
#endif

// calculate Z position in between recieved gps coordinates
#ifndef TARGET_INTEGRATE_Z
#define TARGET_INTEGRATE_Z true
#endif

#ifndef CYBERZOO
/* Initialize the main structure */
struct target_t target_landing = {
  .pos = {0},
  .offset = {
    .heading = TARGET_OFFSET_HEADING,
    .distance = TARGET_OFFSET_DISTANCE,
    .height = TARGET_OFFSET_HEIGHT,
  },
  .target_pos_timeout = 0,
  .target_pos_timeout_limit = TARGET_POS_TIMEOUT,
  .rtk_timeout = 0,
  .rtk_timeout_limit = TARGET_RTK_TIMEOUT,
  .integrate_xy = TARGET_INTEGRATE_XY,
  .integrate_z = TARGET_INTEGRATE_Z
};
#else 
/* Initialize the main structure */
struct target_t target_landing = {
  .pos = {0},
  .offset = {
    .heading = TARGET_OFFSET_HEADING,
    .distance = TARGET_OFFSET_DISTANCE_CYBERZOO,
    .height = TARGET_OFFSET_HEIGHT_CYBERZOO,
  },
  .target_pos_timeout = 0,
  .target_pos_timeout_limit = TARGET_POS_TIMEOUT,
  .rtk_timeout = 0,
  .rtk_timeout_limit = TARGET_RTK_TIMEOUT,
  .integrate_xy = TARGET_INTEGRATE_XY,
  .integrate_z = TARGET_INTEGRATE_Z
};
#endif

/* Get the Relative postion from the RTK */
extern struct GpsRelposNED gps_relposned;

/* GPS abi callback */
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s);

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
// send the calculated TARGET_POS from the drone to GCS
static void send_target_pos_info(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_TARGET_POS_INFO(trans, dev, AC_ID,
                              &target_landing.pos.lla.lat,
                              &target_landing.pos.lla.lon,
                              &target_landing.pos.lla.alt,
                              &target_landing.pos.ground_speed,
                              &target_landing.pos.climb,
                              &target_landing.pos.course,
                              &target_landing.pos.heading,
                              &target_landing.offset.heading,
                              &target_landing.offset.distance,
                              &target_landing.offset.height,
                              &target_landing.target_pos_timeout);
}
#endif

void target_pos_init(void)
{
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TARGET_POS_INFO, send_target_pos_info);
#endif

  AbiBindMsgGPS(ABI_BROADCAST, &gps_ev, gps_cb);
}

/* Get the GPS lla position */
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  target_landing.gps_lla.lat = gps_s->lla_pos.lat;
  target_landing.gps_lla.lon = gps_s->lla_pos.lon;
  target_landing.gps_lla.alt = gps_s->lla_pos.alt;
}

/**
 * Receive a TARGET_POS message from the ground
 */
void target_parse_target_pos(uint8_t *buf)
{
  if(DL_TARGET_POS_ac_id(buf) != AC_ID)
    return;

  // Save the received values
  target_landing.pos.recv_time = get_sys_time_msec();
  //target_landing.pos.tow = gps_tow_from_sys_ticks(sys_time.nb_tick); // FIXME: need to get from the real GPS
  target_landing.pos.tow = DL_TARGET_POS_itow(buf);; // TODO: test this implementation
  target_landing.pos.lla.lat = DL_TARGET_POS_lat(buf);
  target_landing.pos.lla.lon = DL_TARGET_POS_lon(buf);
  target_landing.pos.lla.alt = DL_TARGET_POS_alt(buf);
  target_landing.pos.ground_speed = DL_TARGET_POS_speed(buf);
  target_landing.pos.climb = DL_TARGET_POS_climb(buf);
  target_landing.pos.course = DL_TARGET_POS_course(buf);
  target_landing.pos.heading = DL_TARGET_POS_heading(buf);
  target_landing.pos.valid = true;
}

/**
 * Receive a TARGET_POS message from the ground

void target_parse_target_pos_roll_compensated(uint8_t *buf)
{
  if(DL_TARGET_POS_ac_id(buf) != AC_ID)
    return;

  // Save the received values
  target_landing.pos.recv_time = get_sys_time_msec();
  target_landing.pos.tow = gps_tow_from_sys_ticks(sys_time.nb_tick); // FIXME: need to get from the real GPS

  // int lat_raw = DL_TARGET_POS_lat(buf);  // UNUSED
  // int lon_raw = DL_TARGET_POS_lon(buf);  // UNUSED
  target_landing.pos.lla.alt = DL_TARGET_POS_alt(buf);
  target_landing.pos.ground_speed = DL_TARGET_POS_speed(buf);
  target_landing.pos.climb = DL_TARGET_POS_climb(buf);
  target_landing.pos.course = DL_TARGET_POS_course(buf);
  target_landing.pos.heading = DL_TARGET_POS_heading(buf);
  target_landing.pos.valid = true;

  

  target_landing.pos.lla.lat = DL_TARGET_POS_lat(buf);
  target_landing.pos.lla.lon = DL_TARGET_POS_lon(buf);
} */

extern struct LlaCoor_i gps_lla;
/**
 * Get the current target position (NED) and heading 
 * RELATIVE TO THE DRONE
 */
bool target_get_pos(struct NedCoor_f *pos, float *heading) {
  /* STEPS for x,y,z
      - LLA postions of ship and drone are converted to NED values w.r.t.
      - x,y,z = NED diff between drone and rtkGPS ship [m]
      - add position shift to x,y,z as long as no new target msg is received
      - add offset from rtkGPS ship to landingside ship
      - x,y,z are now position difference between droneGPS and landingside ship
  */
  // calculate how old the last received msg is
  // In seconds, overflow uint32_t in 49,7 days
  //time_diff = (get_sys_time_msec() - target_landing.pos.recv_time) * 0.001; // FIXME: should be based on TOW of ground gps
  float time_diff = (TOW_now - target_landing.pos.tow) * 0.001; //TODO: TEST
  target_landing.target_pos_timeout = time_diff; // for telemetry

  // /* When we have a valid relative position from the RTK GPS and no timeout update the position */
  // if((gps_relposned.relPosValid != 0) && (gps_relposned.iTOW+target.rtk_timeout) > gps_tow_from_sys_ticks(sys_time.nb_tick)) {
  //   struct NedCoor_f cur_pos = *stateGetPositionNed_f();

  //   // Convert the relative postion to a postion in the NED frame of the UAV
  //   pos->x = cur_pos.x - gps_relposned.relPosN / 100.0f;
  //   pos->y = cur_pos.y - gps_relposned.relPosE / 100.0f;
  //   pos->z = cur_pos.z - gps_relposned.relPosD / 100.0f;
  // }

  /* When we have a valid target_pos message, state ned is initialized and no timeout */
  if(target_pos_valid_no_timeout()) {
    struct NedCoor_i target_pos_cm, drone_pos_cm;

    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target_landing.pos.lla);
    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&drone_pos_cm, &state.ned_origin_i, &target_landing.gps_lla);

    // Convert to floating point (cm to meters)
    pos->x = (target_pos_cm.x - drone_pos_cm.x) * 0.01;
    pos->y = (target_pos_cm.y - drone_pos_cm.y) * 0.01;
    pos->z = (target_pos_cm.z - drone_pos_cm.z) * 0.01;

    // Return the heading
    *heading = target_landing.pos.heading;

    // If we have a velocity measurement try to integrate the x-y position when enabled
    struct NedCoor_f vel = {0};
    bool got_vel = target_get_vel(&vel);
    if(target_landing.integrate_xy && got_vel) {
      pos->x = pos->x + vel.x * time_diff;
      pos->y = pos->y + vel.y * time_diff;
    }

    if(target_landing.integrate_z && got_vel) {
      pos->z = pos->z + vel.z * time_diff;
    }

    // Offset the target
    pos->x += target_landing.offset.distance * cosf((*heading + target_landing.offset.heading)/180.*M_PI);
    pos->y += target_landing.offset.distance * sinf((*heading + target_landing.offset.heading)/180.*M_PI);
    pos->z -= target_landing.offset.height;

    // Compensate Roll of ship

    return true;
  }

  return false;
}

/**
 * Get the current target velocity (NED)
 */
bool target_get_vel(struct NedCoor_f *vel) {

  /* When we have a valid target_pos message, state ned is initialized and no timeout */
  if(target_pos_valid_no_timeout()) {
    // Calculate baed on ground speed and course
    vel->x = target_landing.pos.ground_speed * cosf(target_landing.pos.course/180.*M_PI);
    vel->y = target_landing.pos.ground_speed * sinf(target_landing.pos.course/180.*M_PI);
    vel->z = -target_landing.pos.climb;

    return true;
  }

  return false; //target_pos_set_current_offset
}

/*
 * compensate position for Roll of the ship

bool target_compensate_roll(struct NedCoor_f *vel) {

  if(target_landing.pos.valid && state.ned_initialized_i && (target_landing.pos.recv_time+target_landing.target_pos_timeout) > get_sys_time_msec()) {
    // Calculate baed on ground speed and course
    vel->x = target_landing.pos.ground_speed * cosf(target_landing.pos.course/180.*M_PI);
    vel->y = target_landing.pos.ground_speed * sinf(target_landing.pos.course/180.*M_PI);
    vel->z = -target_landing.pos.climb;

    return true;
  }

  return false; //target_pos_set_current_offset
}
*/

/**
 * Set the current measured distance and heading as offset
 */
bool target_pos_set_current_offset(float unk __attribute__((unused))) {
  if(target_pos_valid_no_timeout()) {
    struct NedCoor_i target_pos_cm; // position of 
    struct NedCoor_f uav_pos = *stateGetPositionNed_f(); // NED position of drone

    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target_landing.pos.lla);

    // Convert to floating point (cm to meters)
    struct NedCoor_f pos;
    pos.x = target_pos_cm.x * 0.01; // [m]
    pos.y = target_pos_cm.y * 0.01; // [m]
    pos.z = target_pos_cm.z * 0.01; // [m]

    target_landing.offset.distance = sqrtf(powf(uav_pos.x - pos.x, 2) + powf(uav_pos.y - pos.y, 2)); // [m] euclidean distance (only horizontal)
    target_landing.offset.height = -(uav_pos.z - pos.z); // [m]
    target_landing.offset.heading = atan2f((uav_pos.y - pos.y), (uav_pos.x - pos.x))*180.0/M_PI - target_landing.pos.heading; // [deg]
    return true;
  }

  return false;
}


/**
 * Set the current measured distance and heading as offset
 */
void target_pos_set_current_offset_here(void) {
  if(target_pos_valid_no_timeout()) {
    struct NedCoor_i target_pos_cm;
    struct NedCoor_f uav_pos = *stateGetPositionNed_f();

    // Convert from LLA to NED using origin from the UAV
    ned_of_lla_point_i(&target_pos_cm, &state.ned_origin_i, &target_landing.pos.lla);

    // Convert to floating point (cm to meters)
    struct NedCoor_f pos;
    pos.x = target_pos_cm.x * 0.01; // [m]
    pos.y = target_pos_cm.y * 0.01; // [m]
    pos.z = target_pos_cm.z * 0.01; // [m]

    target_landing.offset.distance = sqrtf(powf(uav_pos.x - pos.x, 2) + powf(uav_pos.y - pos.y, 2)); // [m] euclidean distance (only horizontal)
    target_landing.offset.height = -(uav_pos.z - pos.z); // [m]
    target_landing.offset.heading = atan2f((uav_pos.y - pos.y), (uav_pos.x - pos.x))*180.0/M_PI - target_landing.pos.heading; // [deg]
  }
}

bool target_pos_valid_no_timeout(void){
  return (target_landing.pos.valid // gps position valid (3D fix)
        && state.ned_initialized_i // NED origin is set
        && (target_landing.pos.recv_time+target_landing.target_pos_timeout_limit) > get_sys_time_msec()); // signal has recently be received
}
