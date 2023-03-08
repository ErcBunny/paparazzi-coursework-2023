#include "modules/computer_vision/cv_maze_runner.h"

#define PRINT(string, ...) fprintf(stderr, "[cv->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if CV_MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;
static bool first_cb = true;

struct image_t *video_cb(struct image_t *img, uint8_t camera_id)
{
  
  opencv_frontend_run(img);
  return NULL;
}

void cv_maze_runner_init(void)
{
  cv_add_to_device(&CAMERA, video_cb, MAX_FPS, 0);
  return;
}

void cv_maze_runner_loop(void)
{
  return;
}