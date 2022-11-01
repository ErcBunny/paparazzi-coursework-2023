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

/** @file "modules/wind_tunnel/wind_tunnel_rot_wing.c"
 * @author Dennis van Wijngaarden <D.C.vanWijngaarden@tudelft.nl>
 * This module allows the user to control seperate actuators for example during wind tunnel experiments.
 */

#include "modules/wind_tunnel/wind_tunnel_rot_wing.h"
#include "mcu_periph/sys_time.h"

int16_t actuators_wt[11] = {0,0,0,0,0,0,0,0,0,0,0};
int16_t actuators_slider_wt[11] = {0,0,0,0,0,0,0,0,0,0,0};

bool actuator_is_servo_wt[11] = {0,0,0,0,0,1,1,1,1,1,1};

bool motors_off_wt = false;
bool motor_off_wt[5] = {0,0,0,0,0};

// actuator sweep parameters
uint8_t wt_actuator_sweep_index = 0;
int16_t wt_input_min_cmd = 0;
int16_t wt_input_max_cmd = 0;
float wt_input_steptime = 5;
uint8_t wt_input_n_steps = 10;

struct wt_active_sweep_params {
  uint8_t sweep_index;
  int16_t min_cmd;
  int16_t max_cmd;
  float steptime;
  uint8_t n_steps;
  float total_time;
  float stepsize;
} wt_active_sweep_p;

// status indicators
bool wt_sweep_running = false;
float wt_sweep_timer_start = 0;

void init_wt_rot_wing(void)
{
  // your init code here
  wt_active_sweep_p.sweep_index = wt_actuator_sweep_index;
  wt_active_sweep_p.min_cmd     = wt_input_min_cmd;
  wt_active_sweep_p.max_cmd     = wt_input_max_cmd;
  wt_active_sweep_p.steptime    = wt_input_steptime;
  wt_active_sweep_p.n_steps     = wt_input_n_steps;
  wt_active_sweep_p.total_time  = wt_active_sweep_p.steptime * (float)(wt_active_sweep_p.n_steps + 1);
  wt_active_sweep_p.stepsize    = (float)(wt_active_sweep_p.max_cmd - wt_active_sweep_p.min_cmd) / (float)wt_active_sweep_p.n_steps;
}

void event_wt_rot_wing(void)
{
  int16_t actuators_temp[11] = {0,0,0,0,0,0,0,0,0,0,0}; 

  // Put prefered actuator commands
  for (uint8_t i = 0; i<11; i++)
  {
    actuators_temp[i] = actuators_slider_wt[i];
  }

  // If sweep is running, put current sweep value on actuator
  if (wt_sweep_running) {
    float sweep_time = get_sys_time_float() - wt_sweep_timer_start;
    if (sweep_time > wt_active_sweep_p.total_time)
    {
      wt_sweep_running = false;
    } else {
      float part_done = sweep_time / wt_active_sweep_p.total_time;
      uint8_t step_number = (uint8_t)(part_done * (float)(wt_active_sweep_p.n_steps + 1));
      int16_t sweep_cmd = wt_active_sweep_p.min_cmd + (int16_t)((float)step_number * wt_active_sweep_p.stepsize);
      actuators_temp[wt_active_sweep_p.sweep_index] = sweep_cmd;
    }
  }

  // Evaluate motor off
  for (uint8_t i = 0; i < 5; i++)
  {
    if (motor_off_wt[i] || motors_off_wt)
    {
      actuators_temp[i] = -9600;
    }
  }

  // Bound actuators_temp and copy to actuators list
  for (uint8_t i = 0; i < 11; i++)
  {
    Bound(actuators_temp[i], -9600, 9600);
    actuators_wt[i] = actuators_temp[i];
  }
}

void evaluate_motor_commands(void)
{
  for (uint8_t i = 0; i < 5; i++)
  {
    if (motor_off_wt[i] || motors_off_wt)
    {
      actuators_wt[i] = -9600;
    }
  }
}

void wind_tunnel_rot_wing_sweep_handler(bool activate)
{
  // Only start when there is a non active sweep
  if (!wt_sweep_running && activate)
  {
    // Copy sweep values to active sweep values
    wt_active_sweep_p.sweep_index = wt_actuator_sweep_index;
    wt_active_sweep_p.min_cmd     = wt_input_min_cmd;
    wt_active_sweep_p.max_cmd     = wt_input_max_cmd;
    wt_active_sweep_p.steptime    = wt_input_steptime;
    wt_active_sweep_p.n_steps     = wt_input_n_steps;
    wt_active_sweep_p.total_time  = wt_active_sweep_p.steptime * (float)(wt_active_sweep_p.n_steps + 1);
    wt_active_sweep_p.stepsize    = (float)(wt_active_sweep_p.max_cmd - wt_active_sweep_p.min_cmd) / (float)wt_active_sweep_p.n_steps;
    wt_sweep_running = true;
    wt_sweep_timer_start = get_sys_time_float();
  } else if (!activate) {
    wt_sweep_running = false;
  }
}


