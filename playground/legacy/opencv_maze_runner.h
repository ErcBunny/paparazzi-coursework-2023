#ifndef OPENCV_MAZE_RUNNER_H
#define OPENCV_MAZE_RUNNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"


#define FARNEBACK 0
#define PCAFLOW 1
#define DISMEDIUM 2
#define DISFAST 3
#define DISULTRAFAST 4

// gazebo orange
#if SIM
#define LM_MIN 41
#define LM_MAX 183
#define CB_MIN 53
#define CB_MAX 121
#define CR_MIN 134
#define CR_MAX 249
#else
#define LM_MIN 30
#define LM_MAX 190
#define CB_MIN 70
#define CB_MAX 130
#define CR_MIN 150
#define CR_MAX 190
#endif

void cv_set_img_size(int h, int w);
void cv_set_optflow_params(uint8_t opt, float roi_h_scale, float roi_w_scale);
void cv_set_color_params(
  uint8_t minlm, uint8_t maxlm,
  uint8_t mincb, uint8_t maxcb,
  uint8_t mincr, uint8_t maxcr
);
void cv_handle_input(struct image_t *input);
void cv_src_to_img();
void cv_calc_optflow();
void cv_calc_color(bool draw);

#ifdef __cplusplus
}
#endif

#endif