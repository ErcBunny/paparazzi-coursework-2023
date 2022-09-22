/*
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef ACTUATORS_UAVCAN_H
#define ACTUATORS_UAVCAN_H

#include "modules/uavcan/uavcan.h"
#include "generated/airframe.h"
#include BOARD_CONFIG

/* uavcan ESC status telemetry structure */
struct actuators_uavcan_telem_t {
  uint8_t node_id;
  float timestamp;
  float voltage;
  float current;
  float temperature;
  int32_t rpm;
  uint32_t error_count;
};

#ifdef SERVOS_UAVCAN1_NB
extern struct actuators_uavcan_telem_t uavcan1_telem[SERVOS_UAVCAN1_NB];
#endif
#ifdef SERVOS_UAVCAN2_NB
extern struct actuators_uavcan_telem_t uavcan2_telem[SERVOS_UAVCAN2_NB];
#endif

/* External functions */
extern void actuators_uavcan_init(struct uavcan_iface_t *iface);
extern void actuators_uavcan_commit(struct uavcan_iface_t *iface, int16_t *values, uint8_t nb);

#endif /* ACTUATORS_UAVCAN_H */