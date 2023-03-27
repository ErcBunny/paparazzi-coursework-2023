/*
 * Copyright (C) 2023 Ryan Y. Liu <yueqianliu@outlook.com>
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
 * @file "modules/computer_vision/cv_maze_runner.c"
 * @author Ryan Y. Liu <yueqianliu@outlook.com>
 */

#include "modules/computer_vision/cv_maze_runner.h"

#define PRINT(string, ...) fprintf(stderr, "[cv->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if CV_MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#if MR_THREAD_IS_ASYNC

static pthread_mutex_t mutex;
static struct image_t src[2], src_cpy[2];
static uint8_t cb_state = CB_STATE_FIRST_RUN;
static pthread_t frontend_thread_id;

void cv_maze_runner_init(void)
{
  VERBOSE_PRINT("using async image processing\n");
  cv_add_to_device(&MR_CAMERA, video_cb, MR_MAX_FPS, 0);
  return;
}

struct image_t *video_cb(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  switch (cb_state)
  {
  case CB_STATE_FIRST_RUN:
    image_create(&src[0], img->w, img->h, img->type);
    image_create(&src[1], img->w, img->h, img->type);
    image_create(&src_cpy[0], img->w, img->h, img->type);
    image_create(&src_cpy[1], img->w, img->h, img->type);
    image_copy(img, &src[0]);
    opencv_frontend_init(img->h, img->w, MR_OPTFLOW_ALGO);
    cb_state = CB_STATE_SECOND_RUN;
    break;

  case CB_STATE_SECOND_RUN:
    image_copy(img, &src[1]);
    pthread_create(&frontend_thread_id, NULL, frontend_thread, NULL);
    cb_state = CB_STATE_INITIALIZED;
    break;

  case CB_STATE_INITIALIZED:
    pthread_mutex_lock(&mutex);
    image_switch(&src[0], &src[1]);
    image_copy(img, &src[1]);
    pthread_mutex_unlock(&mutex);
    break;
  }
  return NULL;
}

void *frontend_thread(void *args __attribute__((unused)))
{
  struct timeval tic, toc;
  long t = 0;
  gettimeofday(&tic, NULL);
  while (true)
  {
    gettimeofday(&toc, NULL);
    t = (toc.tv_sec - tic.tv_sec) * 1000000 + (toc.tv_usec - tic.tv_usec);
    tic = toc;

    // copy shared resources for further processing
    pthread_mutex_lock(&mutex);
    image_copy(&src[0], &src_cpy[0]);
    image_copy(&src[1], &src_cpy[1]);
    pthread_mutex_unlock(&mutex);

    opencv_frontend_run(&src_cpy[0], &src_cpy[1]);
    struct opencv_frontend_return_t ret = opencv_frontend_return();

    AbiSendMsgCV_MAZE_RUNNER(0, ret.lmag, ret.rmag, ret.leof, ret.reof, ret.grad_sum, (int)((float)1e6 / t));
  }

  pthread_exit(NULL);
}

#else

static struct image_t src[2];
static uint8_t cb_state = CB_STATE_FIRST_RUN;

void cv_maze_runner_init(void)
{
  VERBOSE_PRINT("using sync image processing\n");
  cv_add_to_device(&MR_CAMERA, video_cb, MR_MAX_FPS, 0);
  return;
}

struct image_t *video_cb(struct image_t *img, uint8_t camera_id __attribute__((unused)))
{
  switch (cb_state)
  {
  case CB_STATE_FIRST_RUN:
    image_create(&src[0], img->w, img->h, img->type);
    image_create(&src[1], img->w, img->h, img->type);
    image_copy(img, &src[0]);
    opencv_frontend_init(img->h, img->w, MR_OPTFLOW_ALGO);
    cb_state = CB_STATE_SECOND_RUN;
    break;

  case CB_STATE_SECOND_RUN:
    image_copy(img, &src[1]);
    cb_state = CB_STATE_INITIALIZED;
    break;

  case CB_STATE_INITIALIZED:
    image_switch(&src[0], &src[1]);
    image_copy(img, &src[1]);
    opencv_frontend_run(&src[0], &src[1]);
    struct opencv_frontend_return_t ret = opencv_frontend_return();
    AbiSendMsgCV_MAZE_RUNNER(0, ret.lmag, ret.rmag, ret.leof, ret.reof, ret.grad_sum, 0);
    break;
  }
  return NULL;
}

#endif