#ifndef CV_MAZE_RUNNER_H
#define CV_MAZE_RUNNER_H

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/cv.h"
#include "pthread.h"

#include "modules/computer_vision/opencv_maze_runner.h"

#define CV_MAZE_RUNNER_VERBOSE TRUE

extern void cv_maze_runner_init(void);
extern void cv_maze_runner_loop(void);

struct image_t *video_cb(struct image_t *img, uint8_t camera_id);

#endif