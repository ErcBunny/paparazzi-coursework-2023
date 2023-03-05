#include "modules/computer_vision/cv_maze_runner.h"

#define PRINT(string, ...) fprintf(stderr, "[cv->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if CV_MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif


static pthread_mutex_t mutex;

struct image_t *video_cb(struct image_t *img, uint8_t camera_id)
{
  // camera_id is set by user
  // using front or down cam is set in cv_maze_runner.xml and bebop_course_maze_runner.xml
  
  
  
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