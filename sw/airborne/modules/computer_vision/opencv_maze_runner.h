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

void opencv_frontend_run(
  struct image_t *input, int optflow_algo, float roi_flow_h_scale, float roi_flow_w_scale, bool draw, bool opencvshow,
  uint8_t lum_min, uint8_t lum_max,
  uint8_t cb_min, uint8_t cb_max,
  uint8_t cr_min, uint8_t cr_max,
  uint32_t *color_count
);


#ifdef __cplusplus
}
#endif

#endif