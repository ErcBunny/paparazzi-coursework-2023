/*
 * Copyright (C) 2022 Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
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

/** @file "modules/ctrl/eff_scheduling_nederdrone.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * Interpolation of control effectivenss matrix 
of the Nederdrone.
      
If instead using online adaptation is an option, be sure to 
not use this module at the same time!
 */

#include "modules/ctrl/eff_scheduling_nederdrone.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "state.h"
#define INDI_SCHEDULING_LOWER_BOUND_G1 0.0001

int32_t use_scheduling = 1;

static float g_forward[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL_FWD, STABILIZATION_INDI_G1_PITCH_FWD, STABILIZATION_INDI_G1_YAW_FWD, STABILIZATION_INDI_G1_THRUST_FWD};

static float g_hover[4][INDI_NUM_ACT] = {STABILIZATION_INDI_G1_ROLL, STABILIZATION_INDI_G1_PITCH, STABILIZATION_INDI_G1_YAW, STABILIZATION_INDI_G1_THRUST};

// Functions to schedule switching on and of of tip props on front wing
float sched_ratio_tip_props = 1.0;
// If pitch lower, pitch props gradually switch off till  sched_tip_prop_lower_pitch_limit_deg (1 > sched_ratio_tip_props > 0)
float sched_tip_prop_upper_pitch_limit_deg = -30;
// If pitch lower, pitch props switch fully off (sched_ratio_tip_props goes to 0)
float sched_tip_prop_lower_pitch_limit_deg = -70;
// Setting to not switch off tip props during forward flight
bool sched_tip_props_always_on = false;

void ctrl_eff_scheduling_init(void)
{
  // your init code here
  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    g_hover[0][i] = g_hover[0][i] / INDI_G_SCALING;
    g_hover[1][i] = g_hover[1][i] / INDI_G_SCALING;
    g_hover[2][i] = g_hover[2][i] / INDI_G_SCALING;
    g_hover[3][i] = g_hover[3][i] / INDI_G_SCALING;

    g_forward[0][i] = g_forward[0][i] / INDI_G_SCALING;
    g_forward[1][i] = g_forward[1][i] / INDI_G_SCALING;
    g_forward[2][i] = g_forward[2][i] / INDI_G_SCALING;
    g_forward[3][i] = g_forward[3][i] / INDI_G_SCALING;
  }
}

void ctrl_eff_scheduling_periodic(void)
{
  // your periodic code here.
  // freq = 20.0 Hz

  // Go from transition percentage to ratio
  /*float ratio = FLOAT_OF_BFP(transition_percentage, INT32_PERCENTAGE_FRAC) / 100;*/
  float ratio = 0.0;

  // Ratio is only based on pitch now, as the pitot tube is often not mounted.
  if (use_scheduling == 1) {
    ratio = fabs(stateGetNedToBodyEulers_f()->theta) / M_PI_2;
  } else {
    ratio = 0.0;
  }

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {

    // Roll
    g1g2[0][i] = g_hover[0][i] * (1.0 - ratio) + g_forward[0][i] * ratio;
    //Pitch
    g1g2[1][i] = g_hover[1][i] * (1.0 - ratio) + g_forward[1][i] * ratio;
    //Yaw
    g1g2[2][i] = g_hover[2][i] * (1.0 - ratio) + g_forward[2][i] * ratio;
  }

  // Thrust effectiveness
  float ratio_spec_force = 0.0;
  float airspeed = stateGetAirspeed_f();
  Bound(airspeed, 8.0, 20.0);
  ratio_spec_force = (airspeed-8.0) / 12.0;

  for (uint8_t i = 0; i < INDI_NUM_ACT; i++) {
    // Thrust
    g1g2[3][i] = g_hover[3][i] * (1.0 - ratio_spec_force) + g_forward[3][i] * ratio_spec_force;
  }

  // Tip prop ratio
  float pitch_deg = stateGetNedToBodyEulers_f()->theta / 180. * M_PI;
  float pitch_range_deg = sched_tip_prop_upper_pitch_limit_deg - sched_tip_prop_lower_pitch_limit_deg;
  if (sched_tip_props_always_on) {
    sched_ratio_tip_props = 1.0;
  } else if (pitch_deg > sched_tip_prop_upper_pitch_limit_deg) {
    sched_ratio_tip_props = 1.0;
  } else if (pitch_deg < sched_tip_prop_lower_pitch_limit_deg) {
    sched_ratio_tip_props = 0.0;
  } else {
    float pitch_offset = pitch_deg - sched_tip_prop_lower_pitch_limit_deg;
    sched_ratio_tip_props = pitch_offset / pitch_range_deg;
  }
  Bound(sched_ratio_tip_props, 0.0, 1.0);
}


