//
// Created by matteo on 09/05/2020.
//

#include "fs_landing.h"
#include "feed_forward.h"
#include "cyclic_control.h"
#include "cyclic_controller.h"
#include "actuator_freq_test.h"

#include "subsystems/datalink/downlink.h"

#ifndef FS_LANDING_FREQ_TEST
#define FS_LANDING_FREQ_TEST TRUE
#endif
uint8_t freq_test_active = false;

struct fs_landing_t fs_landing;
struct fs_landing_t current_actuator_values;

uint8_t is_spinning = false;
//uint8_t pilot_has_control = false;
uint8_t cyclic_control_active = false;
//uint8_t impulse_control_active = false;
uint8_t has_ff_started = false;

float ff_start_time = 0;
float ft_start_time = 0;

uint8_t use_pre_spin = false;
float pre_spin_pitch_coeff = 0.05;
float pre_spin_speed_setpoint = 8;
float pre_spin_trim_percentage = 0.20;
float err_test = 5;

// Horizontal velocity filter
bool is_horizontal_velocity_filter_initialized = false;
float prev_v_filt;
float v_filt, ins_v;

#define FS_V_FILT_ALPHA 0.0012
#define FS_V_FILT_BETA 0.07
#define FS_V_FILT_GAMMA 0.7
#define SEASONAL_L 120  // 2 * pi * module_frequency / (w_z average)
#define CT_DEFAULT 0.1
float seasonal_arr[SEASONAL_L] = {CT_DEFAULT};
int ctr = 0;
float bt = 0;

/*
 * <message name="FRISBEE_CONTROL" id="55">
 *   <field name="is_spinning" type="uint8">Is module currently active</field>
 *   <field name="v_filt" type="float" unit="m/s">Filtered horizontal velocity norm NED</field>
 *   <field name="ml_avg" type="float">Motor LEFT cyclic average</field>
 *   <field name="mr_avg" type="float">Motor RIGHT cyclic average</field>
 *   <field name="ml_delta" type="float">Motor LEFT cyclic half amplitude</field>
 *   <field name="mr_delta" type="float">Motor RIGHT cyclic half amplitude</field>
 *   <field name="el_avg" type="float">Elevon LEFT cyclic average</field>
 *   <field name="er_avg" type="float">Elevon RIGHT cyclic average</field>
 *   <field name="el_delta" type="float">Elevon LEFT cyclic half amplitude</field>
 *   <field name="er_delta" type="float">Elevon RIGHT cyclic half amplitude</field>
 * </message>
 */
#include "subsystems/datalink/telemetry.h"
static void send_frisbee_control(struct transport_tx *trans, struct link_device *dev) {
  // Multiply with boolean to only send the value when variables are actually being used
  float fs_msg_ml_avg = cyclic_control_active * ml_avg;
  float fs_msg_ml_delta = motor_delta_active * ml_delta;
  float fs_msg_mr_avg = cyclic_control_active * mr_avg;
  float fs_msg_mr_delta = motor_delta_active * mr_delta;
  float fs_msg_el_avg = cyclic_control_active * el_avg;
  float fs_msg_el_delta = elevon_delta_active * el_delta;
  float fs_msg_er_avg = cyclic_control_active * er_avg;
  float fs_msg_er_delta = elevon_delta_active * er_delta;
  pprz_msg_send_FRISBEE_CONTROL(trans, dev, AC_ID,
                                &is_spinning,
                                &v_filt,
                                &fs_msg_ml_avg, &fs_msg_ml_delta,
                                &fs_msg_mr_avg, &fs_msg_mr_delta,
                                &fs_msg_el_avg, &fs_msg_el_delta,
                                &fs_msg_er_avg, &fs_msg_er_delta);
}

void fs_landing_init()
{
  uint8_t i;
  for (i = 0; i < ACTUATORS_NB; i++) {
    fs_landing.commands[i] = 0;
    current_actuator_values.commands[i] = 0;
  }
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FRISBEE_CONTROL, send_frisbee_control);

//  cyclic_controller_init();
}

// TODO Make sure all files agree on direction of spin (e.g. assume anti-clockwise rotation)
void fs_landing_run()
{
  if (is_fs_landing_active()) {
#if FS_LANDING_FREQ_TEST
    if (freq_test_active) {
    freq_test(&current_actuator_values, ft_start_time);
    } else {
      ft_start_time = get_sys_time_float();
      freq_test_active = true;
    }
    return;
#endif
    if (is_spinning) {
      horizontal_velocity_filter();
      spin_actuator_values(&current_actuator_values);  // Constant actuator values to maintain spin

      if (cyclic_control_active) {
        cyclic_control_values(&current_actuator_values);
//        cyclic_controller_run();
      }  // TODO Add controller here

    } else {
      if (use_pre_spin) {
        is_spinning = pre_spin_actuator_values();
      } else {

        if (has_ff_started) {
          is_spinning = ff_actuator_values(&current_actuator_values, ff_start_time);
        } else {
          ff_start_time = get_sys_time_float();
          has_ff_started = true;
        }

      }
    }
  } else {
    is_spinning = false;
    has_ff_started = false;
    freq_test_active = false;

    // Reset horizontal velocity filter
    is_horizontal_velocity_filter_initialized = false;
    for (int i = 0; i < SEASONAL_L; i++) {
      seasonal_arr[i] = CT_DEFAULT;
    }
    bt = 0;
    ctr = 0;
  }
  return;
}

bool pre_spin_actuator_values()
{
  float err = pre_spin_speed_setpoint - stateGetHorizontalSpeedNorm_f();
//    err = err_test;
  if (err > 0) {
    // Assuming max value is upward elevon deflection
    float elevon_l_range = 9600;
    float elevon_r_range = 9600;
    float elevon_l_trim = pre_spin_trim_percentage * elevon_l_range;
    float elevon_r_trim = pre_spin_trim_percentage * elevon_r_range;

    current_actuator_values.commands[SERVO_S_THROTTLE_LEFT] = 0;
    current_actuator_values.commands[SERVO_S_THROTTLE_RIGHT] = 0;
    current_actuator_values.commands[SERVO_S_ELEVON_LEFT] = elevon_l_trim + pre_spin_pitch_coeff * err * elevon_l_range;
    current_actuator_values.commands[SERVO_S_ELEVON_RIGHT] = elevon_r_trim + pre_spin_pitch_coeff * err * elevon_r_range;
    return false;
  } else {
    return true;
  }
}

bool is_fs_landing_active()
{
  bool is_active;
  // Map to command so if a different switch is used it will still work
  // if (radio_control.values[FS_LANDING] < 4500) { TODO Check
  if (radio_control.values[RADIO_AUX2] < 4500) {
    is_active = false;
  } else {
    is_active = true;
  }
  return is_active;
}

// Compute matching paparazzi actuator signal value for motor left
// given motor right value so that the rotation axis is at the center
int32_t get_matching_motl_val(int32_t val) {
  float a_rev = 0.1765237;
  float b_rev = 0.1014789;
  float c_rev = -0.0280731;

  float a_fwd = 0.2359004;
  float b_fwd = 0.2339600;
  float c_fwd = -0.0156445;

  // map pprz val to [0, 1]
  float k = val / 9600.;
  // approximate fwd thrust
  float thr = a_fwd * pow(k, 2) + b_fwd * k + c_fwd;
  // calculate [0, 1] value of reverse motor from thrust
  float m = (-b_rev + sqrtf(pow(b_rev,2 )- 4. * a_rev * (c_rev - thr))) / (2. * a_rev);
  // remap to pprz value
  float p = (int32_t) (m * -9600);
  return p;
}

/*
 * Triple Exponential Smoothing (Holt-Winters) filter for horizontal velocity norm
 * Hyper-parameters ALPHA, BETA, GAMMA tuned by hand
 * Seasonal array initialized incorrectly to some value that seems to work
 */
void horizontal_velocity_filter() {
  float ins_vx = stateGetSpeedNed_f()->x;
  float ins_vy = stateGetSpeedNed_f()->y;
  ins_v = sqrtf(pow(ins_vx, 2) + pow(ins_vy, 2));

  // Prevent potential overflow
  if (ctr >= SEASONAL_L * 1000) {
    ctr = 0;
  }

  if (!is_horizontal_velocity_filter_initialized) {
    prev_v_filt = ins_v;
    is_horizontal_velocity_filter_initialized = true;
  } else {
    float ct = seasonal_arr[ctr % SEASONAL_L];

    v_filt = FS_V_FILT_ALPHA * (ins_v - ct) + (1 - FS_V_FILT_ALPHA) * (prev_v_filt + bt);
    bt = FS_V_FILT_BETA * (v_filt - prev_v_filt) + (1 - FS_V_FILT_BETA) * bt;
    seasonal_arr[ctr % SEASONAL_L] = FS_V_FILT_GAMMA * (ins_v - v_filt - bt) + (1 - FS_V_FILT_GAMMA) * ct;

    prev_v_filt = v_filt;
    ctr++;
  }
}

void fs_landing_set_actuator_values()
{
  if (is_fs_landing_active()) {
    fs_landing.commands[SERVO_S_THROTTLE_LEFT] = current_actuator_values.commands[SERVO_S_THROTTLE_LEFT];
    fs_landing.commands[SERVO_S_THROTTLE_RIGHT] = current_actuator_values.commands[SERVO_S_THROTTLE_RIGHT];
    fs_landing.commands[SERVO_S_ELEVON_LEFT] = current_actuator_values.commands[SERVO_S_ELEVON_LEFT];
    fs_landing.commands[SERVO_S_ELEVON_RIGHT] = current_actuator_values.commands[SERVO_S_ELEVON_RIGHT];

    uint8_t i;
    for (i = 0; i < ACTUATORS_NB; i++) {
      BoundAbs(fs_landing.commands[i], MAX_PPRZ);
    }
  }
}
