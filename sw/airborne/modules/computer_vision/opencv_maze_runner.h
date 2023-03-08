#ifndef OPENCV_MAZE_RUNNER_H
#define OPENCV_MAZE_RUNNER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/lib/vision/image.h"

void opencv_frontend_run(struct image_t *input);


#ifdef __cplusplus
}
#endif

#endif