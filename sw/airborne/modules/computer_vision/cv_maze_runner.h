#ifndef CV_MAZE_RUNNER_H
#define CV_MAZE_RUNNER_H

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/cv.h"
#include "pthread.h"
#include <time.h>
#include <stdio.h>

#include "modules/computer_vision/opencv_maze_runner.h"

#define CV_MAZE_RUNNER_VERBOSE TRUE

#define CB_STATE_FIRST_RUN 0
#define CB_STATE_SECOND_RUN 1
#define CB_STATE_INITIALIZED 2

extern void cv_maze_runner_init(void);
extern void cv_maze_runner_loop(void);

struct image_t *video_cb(struct image_t *img, uint8_t camera_id);

#endif