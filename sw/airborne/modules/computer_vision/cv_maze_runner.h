#ifndef CV_MAZE_RUNNER_H
#define CV_MAZE_RUNNER_H

#include "firmwares/rotorcraft/navigation.h"
#include "generated/airframe.h"
#include "modules/core/abi.h"
#include "modules/computer_vision/cv.h"
#include "pthread.h"
#include <stdio.h>

#include "modules/computer_vision/opencv_maze_runner.h"

#define CV_MAZE_RUNNER_VERBOSE TRUE

#define SIM TRUE

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


extern void cv_maze_runner_init(void);
extern void cv_maze_runner_loop(void);

struct image_t *video_cb(struct image_t *img, uint8_t camera_id);

#endif