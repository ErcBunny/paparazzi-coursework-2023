/*
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
 * @file "sw/airborne/modules/system_identification/sys_id_doublet.c"
 * @author Tomaso De Ponti
 * System identification doublet
 */

#include "std.h"

#include "sys_id_doublet.h"
#include "pprz_doublet.h"

#include "subsystems/datalink/telemetry.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"
#include "filters/low_pass_filter.h"
#include "math/pprz_random.h"
#include "subsystems/actuators.h"

#ifndef DOUBLET_ENABLED
#define DOUBLET_ENABLED TRUE
#endif

static struct doublet_t doublet;
uint8_t doublet_active = false;
uint8_t doublet_axis = 0;
pprz_t doublet_amplitude = 0;
float doublet_length_s = 20;

// The axes on which noise and doublet values can be applied
#define DOUBLET_NB_AXES INDI_NUM_ACT // Number of items in ACTIVE_DOUBLET_AXES


// Chirp and noise values for all axes (indices correspond to the axes given in DOUBLET_AXES)
pprz_t current_doublet_values[DOUBLET_NB_AXES];
bool doublet_act_is_servo[INDI_NUM_ACT] = STABILIZATION_INDI_ACT_IS_SERVO;

static void set_current_doublet_values(void)
{
  if (doublet_active) {
    current_doublet_values[doublet_axis] = (int32_t)(doublet_amplitude * doublet.current_value); //removed +=
  } else {
    for (uint8_t i = 0; i < DOUBLET_NB_AXES; i++) {
      current_doublet_values[i] = 0;
    }
  }
}

static void send_doublet(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_DOUBLET(trans, dev, AC_ID, &doublet_active, &doublet.percentage_done,&doublet_axis, &doublet_amplitude);
}

static void start_doublet(void)
{
  doublet_reset(&doublet, get_sys_time_float());
  doublet_active = true;
  set_current_doublet_values();
}

static void stop_doublet(void)
{
  doublet_reset(&doublet, get_sys_time_float());
  doublet_active = false;
  set_current_doublet_values();
}

void sys_id_doublet_activate_handler(uint8_t activate)
{
  doublet_active = activate;
  if (doublet_active) {
    doublet_init(&doublet,doublet_length_s, get_sys_time_float());
    start_doublet();
  } else {
    stop_doublet();
  }
}

extern void sys_id_doublet_axis_handler(uint8_t axis)
{
  if (axis < DOUBLET_NB_AXES) {
    doublet_axis = axis;
  }
}


void sys_id_doublet_init(void)
{
  doublet_init(&doublet, doublet_length_s, get_sys_time_float());
  set_current_doublet_values();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DOUBLET, send_doublet);
}

void sys_id_doublet_run(void)
{
#if DOUBLET_ENABLED

  if (doublet_active) {
    if (!doublet_is_running(&doublet, get_sys_time_float())) {
      stop_doublet();
    } else {
      doublet_update(&doublet, get_sys_time_float());
      set_current_doublet_values();
    }
  }

#endif
}

void sys_id_doublet_add_values(bool motors_on, bool override_on, __attribute__((__unused__)) pprz_t in_cmd[])
{
  (void)(override_on); // Suppress unused parameter warnings

#if DOUBLET_ENABLED

  if (motors_on) {
    for (uint8_t i = 0; i < DOUBLET_NB_AXES; i++) {
      actuators_pprz[i] += current_doublet_values[i];
      if (doublet_act_is_servo[i]) {
        BoundAbs(actuators_pprz[i], MAX_PPRZ);
      } else {
        Bound(actuators_pprz[i], 0, MAX_PPRZ);
      }
    }
  }

#endif
}

extern void sys_id_doublet_set_param (float amp, float time, uint8_t axis)
{
doublet_axis = axis;
doublet_amplitude = amp;
doublet_length_s = time;
} 