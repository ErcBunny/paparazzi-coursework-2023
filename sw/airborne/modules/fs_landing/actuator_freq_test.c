//
// Created by matteo on 06/07/2020.
//

#include "actuator_freq_test.h"

#define MAX_TEST_FREQ 15
#define TEST_STEP_DUR 3
#define FREQ_DELTA 0.25

#define CAL_STEP_DUR 2
#define CAL_STEPS 20

float test_freq = 0.;
float test_min_value = 1000;  // -9600 lowest val possible
float test_max_value = 10000 ;  // 9600 highest val possible
uint8_t actuator_idx = 1;  // Actuator being tested
uint8_t automatic_f = false;  // If true, freq will increase without user input
uint8_t is_input_sin = false;  // If true, chirp is sine wave instead of square
uint8_t calibrate = false;  // Calibration procedure (activate before turning on module)
uint8_t cal_reverse = false;  // To test reverse thrust (calibration)
float ramp_min_freq = 0.5;
float ramp_max_freq = 15;
float ramp_const_freq_duration = 2;
float ramp_duration = 120;

enum test_mode freq_test_mode = CALIBRATE;

// Spin the motors in steps to match % power to frequency with FFT
static void motor_freq_calibration(struct fs_landing_t *actuator_values, float time_from_start);
static void motor_discrete_test(struct fs_landing_t *actuator_values, float time_start);
static void motor_continuous_test(struct fs_landing_t *actuator_values, float t);

void motor_freq_calibration(struct fs_landing_t *actuator_values, float time_from_start) {
  // This test is only for the motors
  if (actuator_idx == 1 || actuator_idx == 2) {
    return;
  }
  float step_sign;
  step_sign =  cal_reverse ? -1 : 1;
  float calibration_steps = CAL_STEPS;
  float cs_time = CAL_STEP_DUR;
  float step_value_calib = step_sign * (9600 / calibration_steps);

  uint8_t cal_phase = (uint8_t) (time_from_start / cs_time);
  float test_current_value = cal_phase * step_value_calib;
  if (time_from_start > cs_time * (calibration_steps + 1)) { // Test over
    test_current_value = 0;
  }
  actuator_values->commands[actuator_idx] = (int32_t) test_current_value;
}

void motor_discrete_test(struct fs_landing_t *actuator_values, float ts) {
  uint8_t test_step = 0;
  if (automatic_f) {
    float freq_delta = FREQ_DELTA;
    float test_step_duration = TEST_STEP_DUR;
    test_step = (uint8_t)(ts / test_step_duration);
    test_freq = test_step * freq_delta;
    if (test_freq > MAX_TEST_FREQ) {  // Test over
      actuator_values->commands[actuator_idx] = 0;
      return;
    }
  }
  float test_sin = sinf(ts * 2 * M_PI * test_freq);

  float test_current_value = 0;
  if (is_input_sin) {
    float midpoint = (test_min_value + test_max_value) * 0.5;
    float half_amplitude = (test_min_value - test_max_value) * 0.5;
    test_current_value = midpoint + half_amplitude * test_sin;
  } else {
    if (test_sin < 0) {
      test_current_value = test_min_value;
    } else {
      test_current_value = test_max_value;
    }
  }
  actuator_values->commands[actuator_idx] = (int32_t) test_current_value;
}

void motor_continuous_test(struct fs_landing_t *actuator_values, float ts) {
  float f = ramp_min_freq + ((ramp_max_freq - ramp_min_freq) / ramp_duration) * (ts - ramp_const_freq_duration);

  if (ts < ramp_const_freq_duration) {
    f = ramp_min_freq;
  } else if (ts >= ramp_const_freq_duration + ramp_duration){
    actuator_values->commands[actuator_idx] = 0;
    return;
  }
  float midpoint = (test_min_value + test_max_value) * 0.5;
  float half_amplitude = (test_min_value - test_max_value) * 0.5;
  float test_current_value = midpoint + half_amplitude * sinf(ts * 2 * M_PI * f);

  actuator_values->commands[actuator_idx] = (int32_t) test_current_value;
}

void freq_test(struct fs_landing_t *actuator_values, float start_t) {
  actuator_values->commands[SERVO_S_ELEVON_LEFT] = 0;
  actuator_values->commands[SERVO_S_THROTTLE_LEFT] = 0;
  actuator_values->commands[SERVO_S_THROTTLE_RIGHT] = 0;
  actuator_values->commands[SERVO_S_ELEVON_RIGHT] = 0;

  double t = get_sys_time_float() - start_t;

  if (freq_test_mode == CALIBRATE) {
    motor_freq_calibration(actuator_values, t);
  } else if (freq_test_mode == DISCRETE) {
    motor_discrete_test(actuator_values, t);
  } else if (freq_test_mode == CONTINUOUS) {
    motor_continuous_test(actuator_values, t);
  }
}
