#include "modules/computer_vision/cv_maze_runner.h"
#if SIM
#include <sys/time.h>
#endif

#define PRINT(string, ...) fprintf(stderr, "[cv->%s()] " string, __FUNCTION__, ##__VA_ARGS__)
#if CV_MAZE_RUNNER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

static pthread_mutex_t mutex;

struct image_t *video_cb(struct image_t *img, uint8_t camera_id)
{


#if SIM
  struct timeval start_time, end_time;
  long elapsed_time;
  gettimeofday(&start_time, NULL);
#endif


  uint32_t color_count;
  opencv_frontend_run(
    img, DISMEDIUM, 1.0, 0.5, true, true,
    LM_MIN, LM_MAX,
    CB_MIN, CB_MAX,
    CR_MIN, CR_MAX,
    &color_count
  );


  
#if SIM
  gettimeofday(&end_time, NULL);
  elapsed_time = (end_time.tv_sec - start_time.tv_sec) * 1000000 +
                  (end_time.tv_usec - start_time.tv_usec);
  VERBOSE_PRINT("%d hz, %d\n", (int)((float)1e6/elapsed_time), color_count);
#endif


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